# This will subscribe to /camera_frame topic to
# receive camera frames from drone
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
 
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
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    
    # Display image
    img, info = self.captureFace(current_frame)
    cv2.imshow("camera", img)
    cv2.waitKey(1)

  def captureFace(self, img):
  	print("entered captureFace function...")
  	faceCascade = cv2.CascadeClassifier("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")
  	imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  	faces = faceCascade.detectMultiScale(imgGray, 1.2, 8)

  	myFaceListC = []
  	myFaceListArea = []

  	for (x, y, w, h) in faces:
  		print("Inside for loop now...")
  		cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
  		cx = x + w // 2
  		cy = y + h //2
  		area = w * h
  		cv2.circle(img, (cx, cy), 5, (0, 0,255), cv2.FILLED)
  		myFaceListC.append([cx, cy])
  		myFaceListArea.append(area)
  		print("length of myFaceListArea is: {}".format(len(myFaceListArea)))
  		if len(myFaceListArea) != 0:
  			index = myFaceListArea.index(max(myFaceListArea))
  			return img, [myFaceListC[index], myFaceListArea[index]]
  		else:
  			print('Here')
  			return 0, [[0, 0], 0]
  
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