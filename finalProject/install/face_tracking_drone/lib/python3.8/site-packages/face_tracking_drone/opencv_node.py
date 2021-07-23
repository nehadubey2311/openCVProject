# This will subscribe to /camera_frame topic to
# receive camera frames from drone
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
 
class OpenCVNode(Node):
  """
  Create an OpenCVNode class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('opencv_node')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      'camera_frame', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning

    # Create a publisher. This will publish forward/backward
    # and yaw speed to drive_topic
    self.publisher_ = self.create_publisher(Int16MultiArray, 'drive_topic', 10)
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    self.width, self.height = 360, 240
    self.fbRange = [6200, 6800]
    self.pid = [0.4, 0.4, 0]
    self.pError = 0
    # initialize array to send speed data to drive_topic
    self.array = Int16MultiArray()
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    # print("received current frame: {}".format(type(current_frame)))

    if len(current_frame):
      # Display image
      img, info = self.captureFace(current_frame)
      self.pError, speed, fb = self.trackFace(info, self.width, self.pid, self.pError)
      self.array.data = [speed, fb]
      self.publisher_.publish(self.array)
      self.get_logger().info('Publishing fb and yaw speed')
      cv2.imshow("Image", img)
      cv2.waitKey(1)

  def captureFace(self, img):
  	faceCascade = cv2.CascadeClassifier("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")
  	imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  	# faces = faceCascade.detectMultiScale(imgGray, 1.2, 8)
  	faces = faceCascade.detectMultiScale(imgGray)

  	myFaceListCenter = []
  	myFaceListArea = []
  	# print("faces received are: {}".format(type(faces)))
  	# print(len(faces))
  	for (x, y, w, h) in faces:
  		cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
  		cx = x + w // 2
  		cy = y + h //2
  		area = w * h
  		cv2.circle(img, (cx, cy), 5, (0, 0,255), cv2.FILLED)
  		myFaceListCenter.append([cx, cy])
  		myFaceListArea.append(area)
  		if len(myFaceListArea) != 0:
  			index = myFaceListArea.index(max(myFaceListArea))
  			return img, [myFaceListCenter[index], myFaceListArea[index]]
  		else:
  			return 0, [[0, 0], 0]
  	# When no faces were detected return below
  	return 0, [[0, 0], 0]

  def trackFace(self, info, w, pid, pError):
  	x, y = info[0]
  	area = info[1]
  	fb = 0

  	error = x - w // 2
  	speed = pid[0] * error + pid[1] * (error - pError)
  	speed = int(np.clip(speed, -100, 100))

  	if area > self.fbRange[0] and area < self.fbRange[1]:
  		fb = 0
  	elif area > self.fbRange[1]:
  		fb = -20
  	elif area < self.fbRange[0] and area != 0:
  		fb = 20

  	# if drone didn't find any face then land
  	if x == 0:
  		speed = 0
  		error = 0

  	print(speed, fb)

  	# tello.send_rc_control(0, fb, 0, speed)
  	return error, speed, fb

def main(args=None):
    # Initialize the rclpy library
	rclpy.init(args=args)

	# Create the node
	opencv_node = OpenCVNode()

	# Spin the node so the callback function is called.
	rclpy.spin(opencv_node)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	opencv_node.destroy_node()

	# Shutdown the ROS client library for Python
	rclpy.shutdown()
  
if __name__ == '__main__':
  	main()
