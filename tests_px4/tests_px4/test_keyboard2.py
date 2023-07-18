import sys, termios, rclpy, tty
from rclpy.node import Node
from example_interfaces.msg import String
import select

class KeyBoardNode(Node):

    def __init__(self):
        super().__init__("keyboard_node")
        self.msg = String()
        self.key = ""
        self.settings = self.saveTerminalSettings()
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.key_pub = self.create_publisher(String, "key", 10)
    
    def getKey(self, settings,n):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(n)
        return key
    
    def saveTerminalSettings(self):
        return termios.tcgetattr(sys.stdin)
    
    def restoreTerminalSettings(self,old_settings):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    def timer_callback(self):
        try:
            key1 = self.getKey(self.settings,1)
            print(key1)
            match key1:
                case 'q': #exit
                    self.key = "q"
                    self.restoreTerminalSettings(self.settings)
                    exit(0)
                case ' ':
                    self.key = "up"
                case 'a':
                    self.key = "arm"
                case 't':
                    self.key = "takeoff"
                case 'l':
                    self.key = "land"
                case 'o':
                    self.key = "offboard"
                case 'd':
                    self.key = "down"
                case '\x1b':
                    key2 = self.getKey(self.settings,2)
                    match key2:
                        case '[A':
                            self.key = "forward"
                        case '[B':
                            self.key = "backward"
                        case '[C':
                            self.key = "right"
                        case '[D':
                            self.key = "left"
                        
                case other:
                    self.key = "stop"

            self.msg.data = self.key
            self.key_pub.publish(self.msg)
            
    
        except Exception as e:
            print(e)            

def main(args=None):
    rclpy.init(args=args)
    node = KeyBoardNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()