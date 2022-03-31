from threading import Lock

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy # https://docs.ros2.org/latest/api/rclpy/api/node.html
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# from visualization_msgs.msg import MarkerArray, Marker
from carla_msgs.msg import CarlaStatus
from sensor_msgs.msg import PointCloud2, PointField

from point_cloud2 import read_points

quad_params = {}
quad_params["map_frame"] = 'map'
quad_params["target_frame"] = 'flying_sensor'

idx2semantic_tags = []
idx2semantic_tags.append("Unlabeled")   # 0
idx2semantic_tags.append("Building")    # 1
idx2semantic_tags.append("Fence")       # 2
idx2semantic_tags.append("Other")       # 3
idx2semantic_tags.append("Pedestrian")  # 4
idx2semantic_tags.append("Pole")        # 5
idx2semantic_tags.append("RoadLine")    # 6
idx2semantic_tags.append("Road")        # 7
idx2semantic_tags.append("SideWalk")    # 8
idx2semantic_tags.append("Vegetation")  # 9
idx2semantic_tags.append("Vehicles")    # 10
idx2semantic_tags.append("Wall")        # 11
idx2semantic_tags.append("TrafficSign") # 12
idx2semantic_tags.append("Sky")         # 13
idx2semantic_tags.append("Ground")      # 14
idx2semantic_tags.append("Bridge")      # 15
idx2semantic_tags.append("RailTrack")   # 16
idx2semantic_tags.append("GuardRail")   # 17
idx2semantic_tags.append("TrafficLight")# 18
idx2semantic_tags.append("Static")      # 19
idx2semantic_tags.append("Dynamic")     # 20
idx2semantic_tags.append("Water")       # 21
idx2semantic_tags.append("Terrain")     # 22

semantic_tags2idx = {t:i for i,t in enumerate(idx2semantic_tags)}
idx2semantic_tags = dict(enumerate(idx2semantic_tags))

PLACES2LAND = ["Terrain", "Ground", "Other", "Building", "SideWalk"]
PLACES2LAND = [float(semantic_tags2idx[i]) for i in PLACES2LAND]


class Agent(Node):
    def __init__(self):
        super().__init__('agent', 
                         allow_undeclared_parameters=True, # necessary for using set_parameters
                         automatically_declare_parameters_from_overrides=True) # allows command line parameters

        self.started = False
        self.curr_pos = None
        self.curr_rpy = None
        self.carla_server_time = None
        self.time_lock = Lock()
        self.lidar_lock = Lock()

        # Get one CarlaStatus msg to learn time diff between system and carla
        self.get_carlastatus = self.create_subscription(
            CarlaStatus,
            '/carla/status',
            self.get_carlastatus_cb,
            1)


    def get_carlastatus_cb(self, msg):
        s = msg.header.stamp.sec
        ns = msg.header.stamp.nanosec
        with self.time_lock:
            self.carla_server_time = s*1E9 + ns

        self.destroy_subscription(self.get_carlastatus) # we don't need this subscriber anymore...

        if not self.started:
            self.started = True
            # Timer for the tf
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            self.receive_semantic_lidar = self.create_subscription(
                PointCloud2,
                "/carla/flying_sensor/lidar",
                self.receive_semantic_lidar_cb,
                1)


    def receive_semantic_lidar_cb(self, msg):
        try:
            s = msg.header.stamp.sec
            ns = msg.header.stamp.nanosec
            with self.time_lock:
                self.carla_server_time = s*1E9 + ns

            now = Time(nanoseconds=0.0) #get the latest tf

            # Check for tf
            trans = self.tf_buffer.lookup_transform(
                quad_params["map_frame"],
                quad_params["target_frame"],
                now)

            self.get_logger().info(f'TF received {trans}')
            self.curr_pos = np.array([trans.transform.translation.x, 
                                      trans.transform.translation.y, 
                                      trans.transform.translation.z])

            init_quat = [trans.transform.rotation.x,
                        trans.transform.rotation.y,
                        trans.transform.rotation.z,
                        trans.transform.rotation.w]

            self.curr_rpy = Rotation.from_quat(init_quat).as_euler('xyz')


            # Read semantic lidar
            recv_data = []
            for p in read_points(msg):
                recv_data.append([float(i) for i in p])
                # x,y,z,cos_angle,idx,tag = p
            
            recv_data = np.asanyarray(recv_data)

            labels = recv_data[:, 5]
            indices = np.arange(recv_data.shape[0])

            xy_dists = np.linalg.norm(recv_data[:,:2] - self.curr_pos[:2], ord=2, axis=1)

            z_dists = np.linalg.norm(recv_data[:,2] - self.curr_pos[2], ord=2)

            # These are the points that we care in relation to a collision 
            # when the quad is moving on the XY plane
            quad_plane = indices[z_dists <= 1.0] # indices for points within quad z +/-1.0m

            LANDING_RADIUS = 2
            landing_pts = indices[(xy_dists < LANDING_RADIUS) & ((recv_data[:,2] - self.curr_pos[2]) < 0)]

            pedestrian_pts = indices[labels == float(semantic_tags2idx['Pedestrian'])]
            vehicle_pts = indices[labels == float(semantic_tags2idx['Vehicles'])]
            
            pedestrian_min_dist = xy_dists[pedestrian_pts].min() if pedestrian_pts.any() else 10000
            vehicle_min_dist = xy_dists[vehicle_pts].min() if vehicle_pts.any() else 10000
            quad_plane_min_dist = xy_dists[quad_plane].min() if quad_plane.any() else 10000


            MIN_DIST = 0.01
            pedestrian_cost = 0
            if pedestrian_min_dist < 30:
                pedestrian_cost = 1/(pedestrian_min_dist + MIN_DIST)

            vehicle_cost = 0
            if vehicle_min_dist < 30:
                vehicle_cost = 1/(vehicle_min_dist + MIN_DIST)

            collision_cost = 0
            if quad_plane_min_dist < 2:
                collision_cost = 1/(quad_plane_min_dist + MIN_DIST)

            FIXED_LANDING_COST = 1
            landing_cost = 0
            if landing_pts.any():
                # Landing cost related to the label of things below the drone
                landing_cost = any(np.isin(recv_data[landing_pts,5], PLACES2LAND, invert=True)) * FIXED_LANDING_COST

                # Landing cost related to the flatness
                landing_cost += recv_data[landing_pts,2].std()

            print(f"pedestrian_cost: {pedestrian_cost}, vehicle_cost: {vehicle_cost}, collision_cost: {collision_cost}, landing_cost: {landing_cost}")

            with self.lidar_lock:
                pass

        except TransformException as ex:
            self.get_logger().error(f'Could not transform {quad_params["map_frame"]} to {quad_params["target_frame"]}: {ex}')


def main():
    print("Starting Agent...")
    rclpy.init()
    agent = Agent()
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        print('Node stopped cleanly!')
    finally:
        print("Shutting down Agent...")
        rclpy.shutdown()


if __name__ == '__main__':
   main()