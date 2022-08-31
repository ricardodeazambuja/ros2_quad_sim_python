import numpy as np
import cv2 as cv

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
QUAD_RADIUS = 0.5

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

PLACES2LAND = ["Terrain", "Ground", "Other", "Building", "SideWalk", "Unlabeled"]

MIN_DIST = 0.01 # to avoid 1/zero

class CostMapGenNode(Node):

    def __init__(self, max_delta_t=0.1):
        super().__init__('costmapgen_node')
        self.get_logger().info("Starting!")
        
        self.xa = ((CX-np.arange(0,RESX))/FX)
        self.ya = ((CY-np.arange(0,RESY))/FY)[:,np.newaxis]

        self.x_indices = np.vstack([np.arange(640) for i in range(480)])
        self.y_indices = np.vstack([np.arange(480) for i in range(640)]).T

        self.cv_bridge = CvBridge()

        self.topic_dict = {
                      "semantic_segmentation_down": {"msg_type": Image, 
                                                "topic_name": '/carla/flying_sensor/semantic_segmentation_down/image'},
                      "depth_down": {"msg_type": Image, 
                                                "topic_name": '/carla/flying_sensor/depth_down/image'},
                      "semantic_segmentation_front": {"msg_type": Image, 
                                                "topic_name": '/carla/flying_sensor/semantic_segmentation_front/image'},
                      "depth_front": {"msg_type": Image, 
                                                "topic_name": '/carla/flying_sensor/depth_front/image'},
                      "semantic_segmentation_back": {"msg_type": Image, 
                                                "topic_name": '/carla/flying_sensor/semantic_segmentation_back/image'},
                      "depth_back": {"msg_type": Image, 
                                                "topic_name": '/carla/flying_sensor/depth_back/image'},
                      "semantic_segmentation_left": {"msg_type": Image, 
                                                "topic_name": '/carla/flying_sensor/semantic_segmentation_left/image'},
                      "depth_left": {"msg_type": Image, 
                                                "topic_name": '/carla/flying_sensor/depth_left/image'},
                      "semantic_segmentation_right": {"msg_type": Image, 
                                                "topic_name": '/carla/flying_sensor/semantic_segmentation_right/image'},
                      "depth_right": {"msg_type": Image, 
                                                "topic_name": '/carla/flying_sensor/depth_right/image'}
                     }

        self.synced_msgs = SyncROS2Messages(self, self.topic_dict, self.process_msgs, max_delta_t=max_delta_t)

        self.costmap_pub = self.create_publisher(Image, f'/costmapgen_node/image', 10)


    def get_xmatrix_from_depth(self, zmatrix):
        return zmatrix * self.xa

    def get_ymatrix_from_depth(self, zmatrix):
        return zmatrix * self.ya

    @staticmethod
    def get_mask(img, label):
        # img is BGR!
        return (img[...,2] == LABELS[label][0]) & (img[...,1] == LABELS[label][1]) & (img[...,0] == LABELS[label][2])

    def get_heat(self, img, thickness=5):
        # Find contours
        kernel = np.ones((5,5),np.uint8)
        dilation = cv.dilate(img,kernel,iterations = 5)
        erosion = cv.erode(dilation,kernel,iterations = 5)
        contours, hierarchy = cv.findContours(dilation, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        # Draw contours
        output = np.zeros((img.shape[0], img.shape[1]), dtype=np.float64)
        max_val = 1
        for i,c in enumerate(contours):
            M = cv.moments(c)
            if M["m00"] != 0:
                px = int(M["m10"] / M["m00"])
                py = int(M["m01"] / M["m00"])
                r = max_val/(np.sqrt((self.x_indices-px)**2+(self.y_indices-py)**2)+1)**(1/2)
                print(r.max(), r.min())
                output += r
                output[output>max_val]=max_val
                #cv.circle(output, (px, py), 5, (255, 0, 0), -1)

            # for ci in c:
            #     cv.line(output,(CX,CY),(ci[0][0],ci[0][1]),(255,0,0),thickness)

        return output

    def process_msgs(self, msg_dict):
        
        depth_front_msg = msg_dict['depth_front']
        depth_front_img = self.cv_bridge.imgmsg_to_cv2(depth_front_msg)
        depth_back_msg = msg_dict['depth_back']
        depth_back_img = self.cv_bridge.imgmsg_to_cv2(depth_back_msg)
        depth_left_msg = msg_dict['depth_left']
        depth_left_img = self.cv_bridge.imgmsg_to_cv2(depth_left_msg)
        depth_right_msg = msg_dict['depth_right']
        depth_right_img = self.cv_bridge.imgmsg_to_cv2(depth_right_msg)

        depth_down_msg = msg_dict['depth_down']
        semantic_segmentation_down_msg = msg_dict['semantic_segmentation_down']

        # process received msgs
        self.get_logger().info("Processing msgs!")
        depth_img = self.cv_bridge.imgmsg_to_cv2(depth_down_msg)
        xmatrix = self.get_xmatrix_from_depth(depth_img)
        ymatrix = self.get_ymatrix_from_depth(depth_img)
        radius = xmatrix**2 + ymatrix**2
        # one_over_radius = 1/(radius+MIN_DIST)

        max_y_down = ymatrix[0,:].mean()
        diff_y = np.linspace(0,max_y_down,240)
        max_x_down = xmatrix.max()
        min_x_down = xmatrix.min()
        diff_x = np.linspace(max_x_down,min_x_down,640)

        xmatrix_front = self.get_xmatrix_from_depth(depth_front_img)
        visible_depth_front = depth_front_img <= max_y_down
        visible_depth_front &= xmatrix_front <=  max_x_down
        visible_depth_front &= xmatrix_front >=  min_x_down

        cost_map = np.zeros(depth_img.shape)
        #cost_map[visible_depth_front] = 1            
        for i in range(depth_img.shape[0]):
            for j in range(depth_img.shape[1]):
                if visible_depth_front[i,j]:
                    x = xmatrix_front[i,j]
                    y = depth_front_img[i,j]
                    xj = np.argmin(abs(diff_x-x))
                    yi = 240-np.argmin(abs(diff_y-y))
                    #_ , xj = np.unravel_index(np.argmin(abs(xmatrix-x)), xmatrix.shape)
                    #yi, _  = np.unravel_index(np.argmin(abs(ymatrix-y)), ymatrix.shape)
                    cost_map[:yi,xj] = 1

        segm_img = self.cv_bridge.imgmsg_to_cv2(semantic_segmentation_down_msg)[:,:,:3]

        pedestrians_mask = self.get_mask(segm_img, 'Pedestrian')

        vehicles_mask = self.get_mask(segm_img, 'Vehicles')

        # places to land mask
        places2land_mask = np.ones(segm_img.shape[:2])==1
        for label in PLACES2LAND:
            places2land_mask &= self.get_mask(segm_img, label)
        places2land_mask = np.logical_not(places2land_mask).astype(float)


        # cost for pedestrians
        if pedestrians_mask.any():
            cost_map += self.get_heat((pedestrians_mask*255).astype('uint8'))

        
        # cost for vehicles
        if vehicles_mask.any():
            cost_map += self.get_heat((vehicles_mask*255).astype('uint8'))

        # cost for landing terrain label
        cost_map += places2land_mask

        # cost for depth roughness
        roughness_cost = abs(depth_img - depth_img.mean())
        roughness_cost = cv.blur(roughness_cost,(64,64))
        cost_map += roughness_cost/roughness_cost.max()

        # cost_map = cv.resize(cost_map, (640,640))

        # df = 1/(depth_front_img.sum(axis=0)+MIN_DIST)
        # db = 1/(depth_back_img.sum(axis=0)+MIN_DIST)
        # dl = 1/(depth_left_img.sum(axis=0)+MIN_DIST)
        # dr = 1/(depth_right_img.sum(axis=0)+MIN_DIST)

        # for i in range(320):
        #     cost_map[i,:] += df/df.max()
        #     cost_map[cost_map.shape[0]-1-i,:] += db/db.max()

        # for i in range(320):
        #     cost_map[:,i] += dl/dl.max()
        #     cost_map[:,cost_map.shape[1]-1-i] += dr/dr.max()


        self.get_logger().info(f"cost_map - max:{cost_map.max()}, min: {cost_map.min()}")

        cost_map = 255*cost_map/cost_map.max()

        # Visualize quad radius (the area the quad would fill)
        # cost_map[radius<=QUAD_RADIUS] = 255
        
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