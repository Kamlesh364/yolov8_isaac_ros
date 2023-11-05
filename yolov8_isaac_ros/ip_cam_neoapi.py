from sys import exit
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pypylon import pylon


class ImagePublisher(Node):
    def __init__(self):
        super().__init__("image_publisher")
        self.bridge = CvBridge()
        try:
            # "GigE" or "USB or usb-port-number or camera_serial_number
            self.cap = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
            # Conversion
            self.converter = pylon.ImageFormatConverter()
            self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
            self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
            self.cap.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)  
        except Exception as e:
            print(e)
            exit()
        # print("image_publisher...")
        # self.pub = self.create_publisher(Image, "/color", 100)
        self.bgr8pub = self.create_publisher(Image, "/color/image_raw", 100)

        self.streamer = self.create_timer(0.5, self.run)

    def run(self):
        while self.cap.IsGrabbing():
            # self.cap.Gain.SetValue(1.0)
            try:
                grabResult = self.cap.RetrieveResult(
                    5000, pylon.TimeoutHandling_ThrowException)
                
                if grabResult.GrabSucceeded():
                    frame = self.converter.Convert(grabResult).GetArray()
                    frame = cv2.resize(frame, (640, 640))
                    cv2.imshow("frame", frame)
                    cv2.waitKey(1)
                    # self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                    # BGR8
                    self.bgr8pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                grabResult.Release()
            except CvBridgeError as e:
                print(e)
                self.cap.Close()

    def set_camera_parameters(self):
        self.cap.Gain.SetValue(1.0)
        self.cap.ExposureTime.SetValue(10000.0)
        return


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
