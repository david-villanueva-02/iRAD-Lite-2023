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
                print(char)
                # Modes
                if char == ord('T'):
                    msg.data = "takeoff"
                elif char == ord('L'):
                    msg.data = "land"
                elif char == ord('A'):
                    msg.data = "arm"
                elif char == ord('O'):
                    msg.data = "offboard"
                elif char == ord('R'):
                    msg.data = "return"
                elif char == curses.KEY_DOWN:
                    msg.data = "back"
                elif char == curses.KEY_UP:
                    msg.data = "front"
                elif char == curses.KEY_RIGHT:
                    msg.data = "right"
                elif char == curses.KEY_LEFT:
                    msg.data = "left"
                elif char == ord('w'):
                    msg.data = "up"
                elif char == ord('a'):
                    msg.data = "rotate_left"
                elif char == ord('s'):
                    msg.data = "down"
                elif char == ord('d'):
                    msg.data = "rotate_right"
                elif char == 43: # + symbol ascii
                    msg.data = "plus"
                elif char == 45: # - symbol ascii
                    msg.data = "minus"
                elif char == ord(' '):
                    msg.data = "stop"

                self.keyboard_monitor_publisher.publish(msg)
                
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
