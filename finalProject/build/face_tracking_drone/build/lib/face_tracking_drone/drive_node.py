# This node will listen to drive_topic that would contain driving
# directions for drone based on image processing done by opencv_node
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

from djitellopy import Tello

class DriveNode(Node):
	"""
	Create a DriveNode class, which is a subclass of the Node class.
	"""
	def __init__(self):
		"""
		Class constructor to set up the node
		"""
		# Initiate the Node class's constructor and give it a name
		super().__init__('drive_node')
		# Create the subscriber. This subscriber will receive an Image
		# from the video_frames topic. The queue size is 10 messages.
		self.subscription = self.create_subscription(
		  Int16MultiArray, 
		  'drive_topic', 
		  self.listener_callback, 
		  10)
		self.subscription # prevent unused variable warning

		self.array = Int16MultiArray()

	def listener_callback(self, data):
	    """
	    Callback function.
	    """
	    # Display the message on the console
	    self.get_logger().info('Receiving drone driving instructions')
	    speed = data.data[0]
	    fb = data.data[1]
	    self.send_driving_directions(speed, fb)

	def send_driving_directions(self, speed, fb):
		speed = speed
		fb = fb
		print(speed, fb)


def main(args=None):
    # Initialize the rclpy library
	rclpy.init(args=args)

	# Create the node
	drive_node = DriveNode()

	# Spin the node so the callback function is called.
	rclpy.spin(drive_node)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	drive_node.destroy_node()

	# Shutdown the ROS client library for Python
	rclpy.shutdown()
  
if __name__ == '__main__':
  	main()
