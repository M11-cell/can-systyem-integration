// Standalone ROS 2 node that prints BAB (Battery Arbiter Board) telemetry and
// optionally records it to a CSV file for later analysis.
//
// Usage:
//   ros2 run bab-board bab_telemetry_node
//   ros2 run bab-board bab_telemetry_node --ros-args
//       -p can_interface:=can0
//       -p print_period_ms:=500
//       -p log_unknown_bab_frames:=true
//
//   # Record every telemetry snapshot to CSV (path is auto-generated):
//   ros2 run bab-board bab_telemetry_node --ros-args
//       -p record_csv:=true
//       -p print_to_console:=false
//
// Parameters:
//   can_interface           (string, default "can0") SocketCAN interface.
//   print_period_ms         (int,    default 1000)   Telemetry sample period.
//                                                    Also the CSV row period.
//   print_to_console        (bool,   default true)   When false, suppress the
//                                                    periodic ROS log print and
//                                                    only record to CSV.
//   record_csv              (bool,   default false)  When true, write one row
//                                                    per sample to an auto-named
//                                                    file under
//                                                    ./bab_telemetry_logs/. The
//                                                    file is flushed every row
//                                                    and closed cleanly on
//                                                    shutdown (Ctrl+C).
//   log_unknown_bab_frames  (bool,   default false)  If true, every received
//                                                    frame whose DeviceType
//                                                    field equals 0x01 (BAB)
//                                                    but does not match the
//                                                    documented Manufacturer
//                                                    or DeviceID is logged
//                                                    (throttled). Use this to
//                                                    diagnose CAN-ID mismatch
//                                                    on the bus.

#include <array>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <string_view>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>

#include "bab-board/battery_board.hpp"
#include "can_util/can_util.hpp"

// Match the values actually used by the BAB firmware on the rover
// (see comments at the top of battery_board.cpp).
static constexpr auto BAB_DEVICE_TYPE = can_util::constants::DeviceType::BROADCAST_MESSAGE; // should really be POWER_DISTRIBUTION_MODULE
static constexpr auto BAB_MANUFACTURER = can_util::constants::Manufacturer::TEAM_USE; // 0x08
static constexpr auto BAB_ID = 0x00;

namespace {
    std::string formatRow(
        std::string_view label,
        const bool fresh,
        const bool ever_received,
        const float v,
        const float i,
        std::string_view p_or_t_label,
        const float p_or_t,
        std::string_view extra = {}) {
        const std::string_view status =
            !ever_received ? "  [NO FRAMES YET]" :
            !fresh ? "  [STALE]" :
            "  [ok]";
        if (extra.empty()) {
            return fmt::format(
                "  {:<14} V={:7.2f} I={:7.2f} {}={:7.2f}{}",
                label, v, i, p_or_t_label, p_or_t, status);
        }
        return fmt::format(
            "  {:<14} V={:7.2f} I={:7.2f} {}={:7.2f} {} {}",
            label, v, i, p_or_t_label, p_or_t, extra, status);
    }

    const char* railLabel(const size_t idx) {
        return idx < BAB::RAILS_COUNT ? BAB::RAIL_SUBSYSTEM_NAMES[idx] : "Unknown";
    }

    // CSV-safe column slugs aligned with BAB::RAIL_SUBSYSTEM_NAMES
    // (idx0=Wheel, idx1=5V, idx2=Arm on the deployed rover).
    constexpr std::array<const char*, BAB::RAILS_COUNT> RAIL_SLUGS = {{
        "wheel", "5v", "arm",
    }};

    // BMS critical-low voltage band, matching BAB::getBMSHealth() in
    // battery_board.cpp (above a dead-sensor floor but below the safe minimum).
    constexpr float BMS_CRITICAL_LOW_VOLTAGE_V = 10.0f;
    constexpr float BMS_MIN_VALID_VOLTAGE_V = 0.5f;

    // Format a ROS time as local-wall-clock ISO 8601 with microseconds, e.g.
    // "2026-06-15T14:32:01.123456". Self-contained per row so a variable sample
    // period does not affect interpretation.
    std::string isoTimestamp(const rclcpp::Time& t) {
        const int64_t ns = t.nanoseconds();
        const std::time_t secs = static_cast<std::time_t>(ns / 1'000'000'000);
        const int micros = static_cast<int>(ns / 1000 % 1'000'000);
        std::tm tm_buf{};
        localtime_r(&secs, &tm_buf);
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm_buf);
        return fmt::format("{}.{:06d}", buf, micros);
    }

    // Compact local-time stamp for filenames, e.g. "20260615_143201".
    std::string fileStamp() {
        const std::time_t now = std::time(nullptr);
        std::tm tm_buf{};
        localtime_r(&now, &tm_buf);
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_buf);
        return buf;
    }

    int b2(const bool v) { return v ? 1 : 0; }
}


class BabTelemetryNode : public rclcpp::Node {
public:
    BabTelemetryNode()
        : Node("bab_telemetry_node"),
          logger(this->get_logger().get_child("bab_telemetry_node"), *get_clock()) {
        can_interface = declare_parameter<std::string>("can_interface", "can0");
        const int period_ms = declare_parameter<int>("print_period_ms", 1000);
        print_to_console = declare_parameter<bool>("print_to_console", true);
        const bool record_csv = declare_parameter<bool>("record_csv", false);
        log_unknown = declare_parameter<bool>("log_unknown_bab_frames", false);

        can_controller = std::make_shared<can_util::CANController>(can_interface, this->get_logger());
        if (!can_controller->initialize()) {
            logger.fatal("Failed to initialize canbus");
            throw std::runtime_error("CAN failed to initialize");
        }

        bab = std::make_shared<BAB>(
            get_logger(),
            can_controller,
            BAB_ID
        );

        if (log_unknown) {
            // Independent callback so we can warn about BAB-typed frames whose
            // Manufacturer / DeviceID do not match docs/BAB-docs.md (i.e. frames
            // that the BAB parser will silently ignore).
            unknown_frame_callback = can_controller->registerFrameCallback(
                [this](const uint32_t id, const std::vector<uint8_t>& data) {
                    onAnyFrame(id, data);
                }
            );
        }

        if (record_csv) {
            openCsv();
            // Flush and close cleanly when the node is shut down (Ctrl+C).
            rclcpp::on_shutdown([this] { closeCsv(); });
        }

        timer = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            [this] {
                const rclcpp::Time now = get_clock()->now();
                if (print_to_console) {
                    printTelemetry();
                }
                if (csv_file.is_open()) {
                    appendCsvRow(now);
                }
            }
        );

        logger.info(
            "bab_telemetry_node ready — interface={}, period={} ms, "
            "print_to_console={}, record_csv={}, log_unknown={}",
            can_interface, period_ms, print_to_console, record_csv, log_unknown);
    }

    ~BabTelemetryNode() override {
        closeCsv();
    }

private:
    // ----------------------------- CSV recording ----------------------------

    void openCsv() {
        namespace fs = std::filesystem;
        const fs::path dir = "bab_telemetry_logs";
        std::error_code ec;
        fs::create_directories(dir, ec);
        if (ec) {
            logger.fatal("Failed to create CSV directory {}: {}", dir.string(), ec.message());
            throw std::runtime_error("Could not create CSV output directory");
        }

        const std::string stamp = fileStamp();
        fs::path path = dir / fmt::format("bab_telemetry_{}.csv", stamp);
        for (int suffix = 2; fs::exists(path); ++suffix) {
            path = dir / fmt::format("bab_telemetry_{}_{}.csv", stamp, suffix);
        }

        csv_file.open(path);
        if (!csv_file.is_open()) {
            logger.fatal("Failed to open CSV file {}", path.string());
            throw std::runtime_error("Could not open CSV output file");
        }
        csv_path = path.string();
        writeCsvHeader();
        logger.info("recording CSV to {}", csv_path);
    }

    void closeCsv() {
        if (!csv_file.is_open()) {
            return;
        }
        csv_file.flush();
        csv_file.close();
        logger.info("CSV saved: {} ({} rows)", csv_path, csv_rows);
    }

    void writeCsvHeader() {
        std::string header = "timestamp";
        for (size_t i = 0; i < BAB::BATTERIES_COUNT; ++i) {
            fmt::format_to(
                std::back_inserter(header),
                ",battery_{0}_voltage_v,battery_{0}_current_a,battery_{0}_temp_c",
                i + 1);
        }
        for (size_t i = 0; i < BAB::RAILS_COUNT; ++i) {
            fmt::format_to(
                std::back_inserter(header),
                ",rail_{0}_voltage_v,rail_{0}_current_a,rail_{0}_power_w",
                RAIL_SLUGS[i]);
        }
        header += ",tcu_temp_c,tcu_fan_on,relay_1_closed,relay_2_closed,bms_critical_low";
        csv_file << header << '\n';
    }

    void appendCsvRow(const rclcpp::Time& stamp) {
        std::string row = isoTimestamp(stamp);

        for (size_t i = 0; i < BAB::BATTERIES_COUNT; ++i) {
            fmt::format_to(
                std::back_inserter(row),
                ",{:.2f},{:.2f},{:.2f}",
                bab->getBatteryVoltageLevel(i),
                bab->getBatteryCurrentLevel(i),
                bab->getBatteryTemp(i));
        }

        for (size_t i = 0; i < BAB::RAILS_COUNT; ++i) {
            fmt::format_to(
                std::back_inserter(row),
                ",{:.2f},{:.2f},{:.1f}",
                bab->getRailVoltageLevel(i),
                bab->getRailCurrent(i),
                bab->getRailPower(i));
        }

        const float bat1_v = bab->getBatteryVoltageLevel(0);
        const bool bms_critical_low =
            bat1_v < BMS_CRITICAL_LOW_VOLTAGE_V && bat1_v > BMS_MIN_VALID_VOLTAGE_V;
        fmt::format_to(
            std::back_inserter(row),
            ",{:.2f},{},{},{},{}",
            bab->getTCUTemp(),
            b2(bab->getTCUStatus() == "TCU ON"),
            b2(bab->getRelayClosed(0)),
            b2(bab->getRelayClosed(1)),
            b2(bms_critical_low));

        csv_file << row << '\n';
        csv_file.flush();
        ++csv_rows;
    }

    void printTelemetry() const {
        std::string report = fmt::format("\n--- BAB Telemetry (iface={}) ---", can_interface);

        for (size_t i = 0; i < BAB::BATTERIES_COUNT; ++i) {
            fmt::format_to(
                std::back_inserter(report),
                "\n{}",
                formatRow(
                    fmt::format("Battery {}", i + 1),
                    bab->batteryFresh(i),
                    bab->batteryEverReceived(i),
                    bab->getBatteryVoltageLevel(i),
                    bab->getBatteryCurrentLevel(i),
                    "T",
                    bab->getBatteryTemp(i)));
        }

        for (size_t i = 0; i < BAB::RAILS_COUNT; ++i) {
            const std::string sw =
                !bab->railEverReceived(i) ? "?" :
                bab->getRailSwitchOn(i) ? "ON" : "OFF";
            const std::string extra = fmt::format(
                "P={:.1f}W sw={}",
                bab->getRailPower(i),
                sw);
            fmt::format_to(
                std::back_inserter(report),
                "\n{}",
                formatRow(
                    fmt::format("Rail {} ({})", i + 1, railLabel(i)),
                    bab->railFresh(i),
                    bab->railEverReceived(i),
                    bab->getRailVoltageLevel(i),
                    bab->getRailCurrent(i),
                    "P",
                    bab->getRailPower(i),
                    extra));
        }

        fmt::format_to(
            std::back_inserter(report),
            "\n  {:<14} T={:7.2f} fan={}",
            "TCU", bab->getTCUTemp(), bab->getTCUStatus() == "TCU ON" ? "ON" : "OFF");

        for (size_t i = 0; i < BAB::RELAYS_COUNT; ++i) {
            fmt::format_to(
                std::back_inserter(report),
                "\n  {:<14} {}",
                fmt::format("Relay {}", i + 1),
                bab->getRelayClosed(i) ? "CLOSED" : "OPEN");
        }

        logger.info("{}", report);
    }

    void onAnyFrame(const uint32_t id, const std::vector<uint8_t>& data) const {
        if (const uint8_t device_type = id >> 24 & 0x1F; device_type != static_cast<uint8_t>(BAB_DEVICE_TYPE)) {
            return;
        }
        const uint8_t manufacturer = id >> 16 & 0xFF;
        const uint8_t device_id = id & 0x3F;
        if (manufacturer == static_cast<uint8_t>(BAB_MANUFACTURER) && device_id == BAB_ID) {
            return; // already handled by BAB parser
        }

        using namespace std::chrono_literals;
        logger.warn_throttle(2s, "Unmatched BAB-typed frame id={:#08X} (mfr={:#02X}, devid={:#02X}) data={:#02X}", id, manufacturer, device_id, data);
    }

    ros2_fmt_logger::Logger logger;
    std::string can_interface;
    bool print_to_console = true;
    bool log_unknown = false;
    std::shared_ptr<can_util::CANController> can_controller;
    std::shared_ptr<BAB> bab;
    std::shared_ptr<can_util::CANFrameCallback> unknown_frame_callback;
    rclcpp::TimerBase::SharedPtr timer;

    std::ofstream csv_file;
    std::string csv_path;
    uint64_t csv_rows = 0;
};


int main(const int argc, char** argv) {
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        const auto node = std::make_shared<BabTelemetryNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        ros2_fmt_logger::Logger(rclcpp::get_logger("bab_telemetry_node"))
            .fatal("Node failed to start: {}", e.what());
        exit_code = 1;
    } catch (...) {
        ros2_fmt_logger::Logger(rclcpp::get_logger("bab_telemetry_node"))
            .fatal("Node failed to start: unknown exception");
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}
