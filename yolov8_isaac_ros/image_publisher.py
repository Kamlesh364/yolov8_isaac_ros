import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sys import exit


class ImagePublisher(Node):
    def __init__(self):
        super().__init__("image_publisher")
        self.bridge = CvBridge()
        try:
            self.cap = cv2.VideoCapture("/dev/video2")
            # self.cap = cv2.VideoCapture(
            #     "/workspaces/isaac_ros-dev/src/yolov8_isaac_ros/data/videoplayback.mp4")
        except Exception as e:
            print(e)
            exit()
        # print("image_publisher...")
        # print("Topics: /camera, /camera/rgb, /camera/bgr, /camera/mono")
        self.pub = self.create_publisher(Image, "/camera", 100)
        self.bgr8pub = self.create_publisher(Image, "/camera/image_raw", 100)

        self.streamer = self.create_timer(0.5, self.run)

    def run(self):
        while True:
            try:
                r, frame = self.cap.read()
                frame = cv2.resize(frame, (640, 640))
                if not r:
                    self.cap.release()
                    return
                self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

                # BGR8
                self.bgr8pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

                # # RGB8
                # frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                # self.rgb8pub.publish(
                #     self.bridge.cv2_to_imgmsg(frame_rgb, "rgb8"))

                # # MONO8
                # frame_mono = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # self.mono8pub.publish(
                #     self.bridge.cv2_to_imgmsg(frame_mono, "mono8"))
                # # print("published")

            except CvBridgeError as e:
                print(e)
                self.cap.release()


def main():
    try:
        rclpy.init()
        node = ImagePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


