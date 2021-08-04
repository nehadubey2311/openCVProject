# This node connects with drone and captures video frames
# it will then publishs frames to /camera_frame topic
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Int8
from std_msgs.msg import Int16MultiArray

from djitellopy import Tello
import cv2, math, time
import threading


class DroneNode(Node):
    """
    Create a DroneNode class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('drone_node')
        self.drone = Tello()
        self.drone.connect()
        self.drone.streamon()
        self.drone.takeoff()
        self.publisher_ = self.create_publisher(Image, 'camera_frame', 10)

        self.i = 0

        # Create a VideoCapture object
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
          Int16MultiArray, 
          'drive_topic', 
          self.listener_callback, 
          10)

        # Create second subscriber. This subscriber will keep listening
        # for keyboard interrupts for landing
        self.subscription = self.create_subscription(
          Int8, 
          'override_topic', 
          self.override_callback, 
          10)

        self.array = Int16MultiArray()

        self.capture_frames()


    def listener_callback(self, data):
        """
        Callback function.
        """
        # Commenting out debugging logs
        # self.get_logger().info('Receiving drone driving instructions')
        speed = data.data[0]
        fb = data.data[1]
        self.drone.send_rc_control(0, fb, 0, speed)

    def override_callback(self, data):
        """
        Callback function for manual override
        """
        should_land = data.data
        print(f"heard landing: {should_land}")
        if should_land:
            self.drone.land()
            pass

    def capture_frames(self, rate=0.03):
        """
        Function that captures frames from drone and publishes
        them to 'camera_frame' topic
        """
        def video_capture_thread():
            frame_read = self.drone.get_frame_read()
            while True:
                # Capture frame-by-frame
                img = frame_read.frame
                img = cv2.resize(img, (360, 240))
                image_message = self.br.cv2_to_imgmsg(img)
                self.publisher_.publish(image_message)

                time.sleep(rate)

        self.i += 1
        thread = threading.Thread(target=video_capture_thread)
        thread.start()
        return thread
        


def main(args=None):
    rclpy.init(args=args)

    drone_node = DroneNode()

    rclpy.spin(drone_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drone_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
