# This node will listen to drive_topic that would contain driving
# directions for drone based on image processing done by opencv_node
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import sys, select, termios, tty

import cv2

import threading

class OverrideNode(Node):
    """
    Create a OverrideNode class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('override_node')
        self.settings = termios.tcgetattr(sys.stdin)

        self.should_land = Int8()
        self.publisher_ = self.create_publisher(Int8, 'override_topic', 10)
        self.drone_override()
    
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def drone_override(self):
        """
        Function to keep listening to keyboard inerrupts
        """
        self.get_logger().info('Listening for keyboard interrupt')

        def keyboard_interrupt_thread():
            while True:
                key = self.getKey()
                print(f"key is: {key}")
                if key == 'q' or key == 'Q':
                    self.should_land.data = 1
                    self.publisher_.publish(self.should_land)
                    print("published q")
                # break if 'ctrl + c' received
                if (key == '\x03'):
                    break

        thread = threading.Thread(target=keyboard_interrupt_thread)
        thread.start()
        return thread



def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    override_node = OverrideNode()

    # Spin the node so the callback function is called.
    rclpy.spin(override_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    override_node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
