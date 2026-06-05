"""joy_index_echo — print /joy button indices beside their values.

Subscribes to /joy and prints one line whenever the buttons array changes:

    0:0  1:0  ...  15:1  16:0  17:0  18:0  ...

Usage:
    ros2 run joy joy_node
    ros2 run joy_mux_controller_py joy_index_echo

Parameters:
    topic (string, default /joy)
    pressed_only (bool, default false) — only print index:N pairs where N is 1
    servo_only (bool, default false) — only print C1 hat indices 15–18
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from .vkb_layout import VKBButtonLayout

_C1_INDICES = (
    VKBButtonLayout.C1_UP,
    VKBButtonLayout.C1_RIGHT,
    VKBButtonLayout.C1_DOWN,
    VKBButtonLayout.C1_LEFT,
    VKBButtonLayout.C1_PRESS,
)


class JoyIndexEcho(Node):

    def __init__(self):
        super().__init__('joy_index_echo')
        topic = self.declare_parameter('topic', '/joy').value
        self._pressed_only = self.declare_parameter('pressed_only', False).value
        self._servo_only = self.declare_parameter('servo_only', False).value
        self._prev_buttons: list[int] | None = None

        self._sub = self.create_subscription(Joy, topic, self._on_joy, 10)

        self.get_logger().info(
            f'joy_index_echo ready — topic={topic}, '
            f'pressed_only={self._pressed_only}, servo_only={self._servo_only}. '
            'Press buttons; Ctrl+C to exit.'
        )

    def _format_line(self, buttons: list[int]) -> str:
        if self._servo_only:
            pairs = (
                (i, buttons[i])
                for i in _C1_INDICES
                if i < len(buttons)
            )
        else:
            pairs = enumerate(buttons)

        if self._pressed_only:
            pairs = ((i, b) for i, b in pairs if b)

        return '  '.join(f'{i}:{b}' for i, b in pairs)

    def _on_joy(self, msg: Joy) -> None:
        current = list(msg.buttons)
        if current == self._prev_buttons:
            return
        self._prev_buttons = current
        print(self._format_line(current), flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = JoyIndexEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
