import rclpy
from rclpy.node import Node
import curses

from example_interfaces.msg import String

class KeyboardNode(Node):

    def __init__(self) -> None:
        super().__init__("keyboard")

        self.screen = curses.initscr()
        curses.noecho()
        curses.cbreak()
        self.screen.keypad(True)

        self.keyboard_monitor_publisher = self.create_publisher(String,"keyboard",10)
        self.prev_char = None
        self.keyboard()

    def keyboard(self):
        msg = String()
        try:
            while True:
                char = self.screen.getch()
                # Modes
                if self.prev_char != char:
                    if char == ord('T'):
                        msg.data = "takeoff"
                        self.keyboard_monitor_publisher.publish(msg)
                    elif char == ord('L'):
                        msg.data = "land"
                        self.keyboard_monitor_publisher.publish(msg)
                    elif char == ord('A'):
                        msg.data = "arm"
                        self.keyboard_monitor_publisher.publish(msg)
                    elif char == ord('O'):
                        msg.data = "offboard"
                        self.keyboard_monitor_publisher.publish(msg)
                    elif char == ord('R'):
                        msg.data = "return"
                        self.keyboard_monitor_publisher.publish(msg)
                
                if char == curses.KEY_DOWN:
                    msg.data = "back"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == curses.KEY_UP:
                    msg.data = "front"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == curses.KEY_RIGHT:
                    msg.data = "right"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == curses.KEY_LEFT:
                    msg.data = "left"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == ord('w'):
                    msg.data = "up"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == ord('a'):
                    msg.data = "rotate_left"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == ord('s'):
                    msg.data = "down"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == ord('d'):
                    msg.data = "rotate_right"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == 43: # + symbol ascii
                    msg.data = "plus"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == 45: # - symbol ascii
                    msg.data = "minus"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == " ": # - symbol ascii
                    msg.data = "stop"
                    self.keyboard_monitor_publisher.publish(msg)

                self.prev_char = char
                
        finally:
            curses.nocbreak()
            self.screen.keypad(0)
            curses.echo()
            curses.endwin()

def main(args=None) -> None:
    rclpy.init(args=args)
    keyboard_control = KeyboardNode()
    rclpy.spin(keyboard_control)
    keyboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)