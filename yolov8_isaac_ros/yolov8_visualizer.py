from yolov8_isaac_ros.utils.engine import TRTModule

import message_filters
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from os import environ

from time import time

import cv2
import torch

from yolov8_isaac_ros.utils.config import CLASSES, COLORS
from yolov8_isaac_ros.utils.torch_utils import det_postprocess
from yolov8_isaac_ros.utils.utils import blob, letterbox

environ['CUDA_MODULE_LOADING'] = 'LAZY'

class Yolov8Engine(Node):
    QUEUE_SIZE = 100
    bbox_thickness = 3

    def __init__(self, engine_path, device='cuda:0', show=True):
        super().__init__('yolov8_visualizer')
        self._bridge = CvBridge()
        self._engine_path = engine_path
        self._device = device
        self._show = show
        self._engine = TRTModule(engine_path, device)
        # set desired output names order
        self._engine.set_desired(['num_dets', 'bboxes', 'scores', 'labels'])
        
        self._processed_image_pub = self.create_publisher(
            Image, 'yolov8_processed_image',  self.QUEUE_SIZE)
        
        self._image_subscription = message_filters.Subscriber(
            self, Image, 'color/image_raw') #

        self.time_synchronizer = message_filters.TimeSynchronizer(
            [self._image_subscription], self.QUEUE_SIZE)

        self.time_synchronizer.registerCallback(self.detections_callback)

        self.frame_rate = 0
        self.start_time = time()
        self.frame_count = 0

    def detections_callback(self, img_msg):
        bgr_image = self._bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        bgr_image = cv2.resize(bgr_image, (640, 640))
        H, W = self._engine.inp_info[0].shape[-2:]
        draw = bgr_image.copy()
        _, ratio, dwdh = letterbox(bgr_image, (W, H))
        rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        tensor = blob(rgb, return_seg=False)
        dwdh = torch.asarray(dwdh * 2, dtype = torch.float32, device = self._device)
        tensor = torch.asarray(tensor, device = self._device)
        
        # inference
        data = self._engine(tensor)

        bboxes, scores, labels = det_postprocess(data)
        bboxes -= dwdh
        bboxes /= ratio

        self.frame_count += 1

        for (bbox, score, label) in zip(bboxes, scores, labels):
            bbox = bbox.round().int().tolist()
            cls_id = int(label)
            cls = CLASSES[cls_id]
            color = COLORS[cls]
            cv2.rectangle(draw, (bbox[0],bbox[1]), (bbox[2],bbox[3]), color, 2)
            cv2.putText(draw,
                        f'{cls}:{score:.3f}', (bbox[0], bbox[1] - 2),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.75, [225, 255, 255],
                        thickness=2)

        if self.frame_count > 10:
            self.frame_rate = self.frame_count / (time() - self.start_time)
            self.start_time = time()
            self.frame_count = 0

        self._processed_image_pub.publish(
            self._bridge.cv2_to_imgmsg((draw), 'bgr8'))
        # if self._show:
        #     cv2.putText(draw,
        #                 f'FPS: {self.frame_rate:.2f}', (10, 30),
        #                 cv2.FONT_HERSHEY_SIMPLEX,
        #                 0.75, [225, 255, 255],
        #                 thickness=2)
        #     cv2.imshow('result', draw)
        #     if cv2.waitKey(1) & 0xFF == ord('q'):
        #         cv2.destroyAllWindows()
        return


def main(engine_path='/workspaces/isaac_ros-dev/src/yolov8s.engine', device='cuda:0', show=True):
    rclpy.init()
    rclpy.spin(Yolov8Engine(engine_path, device, show))
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()