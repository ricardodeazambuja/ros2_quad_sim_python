import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from syncros2messages import SyncROS2Messages

RESX = 640
RESY = 480
CX = 320
CY = 240
FX = 432.455
FY = 432.455

LABELS = {
'Unlabeled':    (0, 0, 0),
'Building':     (70, 70, 70),
'Fence':        (100, 40, 40),
'Other':        (55, 90, 80),
'Pedestrian':   (220, 20, 60),
'Pole':         (153, 153, 153),
'RoadLine':     (157, 234, 50),
'Road':         (128, 64, 128),
'SideWalk':     (244, 35, 232),
'Vegetation':   (107, 142, 35),
'Vehicles':     (0, 0, 142),
'Wall':         (102, 102, 156),
'TrafficSign':  (220, 220, 0),
'Sky':          (70, 130, 180),
'Ground':       (81, 0, 81),
'Bridge':       (150, 100, 100),
'RailTrack':    (230, 150, 140),
'GuardRail':    (180, 165, 180),
'TrafficLight': (250, 170, 30),
'Static':       (110, 190, 160),
'Dynamic':      (170, 120, 50),
'Water':        (45, 60, 150),
'Terrain':      (145, 170, 100)
}


MIN_DIST = 0.01 # to avoid 1/zero

class CostMapGenNode(Node):

    def __init__(self, max_delta_t=0.1):
        super().__init__('costmapgen_node')
        self.get_logger().info("Starting!")
        
        self.xa = ((CX-np.arange(0,RESX))/FX)
        self.ya = ((CY-np.arange(0,RESY))/FY)[:,np.newaxis]

        self.cv_bridge = CvBridge()

        topic_dict = {"semantic_segmentation_down": {"msg_type": Image, 
                                                "topic_name": '/carla/flying_sensor/semantic_segmentation_down/image'},
                      "depth_down": {"msg_type": Image, 
                                                "topic_name": '/carla/flying_sensor/depth_down/image'}
                     }

        self.synced_msgs = SyncROS2Messages(self, topic_dict, self.process_msgs, max_delta_t=max_delta_t)

        self.costmap_pub = self.create_publisher(Image, f'/costmapgen_node/image', 10)


    def get_xmatrix_from_depth(self, zmatrix):
        return zmatrix * self.xa

    def get_ymatrix_from_depth(self, zmatrix):
        return zmatrix * self.ya

    @staticmethod
    def get_mask(img, label):
        # img is BGR!
        return (img[...,2] == LABELS[label][0]) & (img[...,1] == LABELS[label][1]) & (img[...,0] == LABELS[label][2])


    def process_msgs(self, msg_dict):
        
        depth_down_msg = msg_dict['depth_down']
        semantic_segmentation_down_msg = msg_dict['semantic_segmentation_down']

        # process received msgs
        self.get_logger().info("Processing msgs!")
        depth_img = self.cv_bridge.imgmsg_to_cv2(depth_down_msg)
        xmatrix = self.get_xmatrix_from_depth(depth_img)
        ymatrix = self.get_ymatrix_from_depth(depth_img)
        radius = xmatrix**2 + ymatrix**2

        one_over_radius = 1/(radius+MIN_DIST)

        segm_img = self.cv_bridge.imgmsg_to_cv2(semantic_segmentation_down_msg)[:,:,:3]

        pedestrians_mask = self.get_mask(segm_img, 'Pedestrian')

        vehicles_mask = self.get_mask(segm_img, 'Vehicles')
        
        cost_map = np.zeros(one_over_radius.shape)
        if pedestrians_mask.any():
            cost_map += one_over_radius*pedestrians_mask
        
        if vehicles_mask.any():
            cost_map += one_over_radius*vehicles_mask

        cost_map = 255*(cost_map-cost_map.min())/cost_map.max()
        self.get_logger().info(f"cost_map - max:{cost_map.max()}, min: {cost_map.min()}")
        msg = self.cv_bridge.cv2_to_imgmsg((cost_map).astype('uint8'), 
                                            encoding='mono8')

        msg.header.stamp = self.get_clock().now().to_msg()
        self.costmap_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        costmapgen_node = CostMapGenNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(costmapgen_node)

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            costmapgen_node.synced_msgs.stop = True
            costmapgen_node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()