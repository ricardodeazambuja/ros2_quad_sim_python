from webbrowser import get
import numpy as np

import rclpy # https://docs.ros2.org/latest/api/rclpy/api/node.html
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Pose
from carla_msgs.srv import SpawnObject, DestroyObject, GetBlueprints, SpawnPoints
# from carla_msgs.msg import CarlaSpawnPoint
from carla_waypoint_types.srv import GetWaypoint


NUMBER_OF_CARS = 10
NUMBER_OF_WALKERS = 30
MAX_DIST_FROM_CENTER = 50

class SpawnEpisode(Node):
    def __init__(self):
        super().__init__('spawn_episode', 
                         allow_undeclared_parameters=True, # necessary for using set_parameters
                         automatically_declare_parameters_from_overrides=True) # allows command line parameters

        self.spawned_vehicles = []
        self.spawned_walkers = []


        # Get spawn points available for cars in the current map
        response = self.call_service(SpawnPoints, "/carla/get_spawn_points", SpawnPoints.Request())

        csp_locations = np.asarray([csp.location_xyz for csp in response.spawn_points])
        csp_rotations = np.asarray([csp.rotation_rpy for csp in response.spawn_points])

        # Randomly select one of the spawn points
        rs = np.random.RandomState()
        rand_spawn_point = csp_locations[rs.choice(csp_locations.shape[0])]

        # Calculates the distances between the selected spawn point to all others
        dists = np.linalg.norm(csp_locations - rand_spawn_point, ord=2, axis=1)
        
        # Sort the spawn points according to the distance to the selected one
        sorted_indices = np.argsort(dists)
        sorted_locations = csp_locations[sorted_indices]

        # Get the blueprints available in the current map
        response = self.call_service(GetBlueprints, "/carla/get_blueprints", GetBlueprints.Request())
        vehicles = [v for v in response.blueprints if "vehicle" in v]
        walkers = [w for w in response.blueprints if "walker" in w]

        central_location = sorted_locations[0]

        # Spawn vehicles
        i = 0
        while i < NUMBER_OF_CARS:
            x,y = self.randXY(rs, MAX_DIST_FROM_CENTER)
            pose = self.get_waypoint(central_location[0]+x, 
                                     central_location[1]+y, 
                                     central_location[2]).waypoint.pose
            res_id = self.spawn_actor(rs.choice(vehicles), f"vehicle_{i:03d}", pose)
            if res_id != -1:
                self.spawned_vehicles.append(res_id)
                i += 1
                

        # Spawn walkers
        i = 0
        while i < NUMBER_OF_WALKERS:
            x,y = self.randXY(rs, MAX_DIST_FROM_CENTER)
            pose = Pose()
            pose.position.x = float(central_location[0]+x)
            pose.position.y = float(central_location[1]+y)
            pose.position.z = float(central_location[2])
            res_id = self.spawn_actor(rs.choice(walkers), f"walker_{i:03d}", pose)
            if res_id != -1:
                self.spawned_walkers.append(res_id)
                i += 1

    
    @staticmethod
    def randXY(randstate, maxlenght):
        radius = np.sqrt(randstate.uniform(0, maxlenght))
        angle = np.pi * randstate.uniform(0, 2)
        return radius * np.cos(angle), radius * np.sin(angle)

    
    def get_waypoint(self, x, y, z):
        request = GetWaypoint.Request()
        request.location.x = float(x)
        request.location.y = float(y)
        request.location.z = float(z)
        response = self.call_service(GetWaypoint, "/carla/get_waypoint", request)
        # response.waypoint.pose
        # response.waypoint.is_junction
        # response.waypoint.road_id
        # response.waypoint.section_id
        # response.waypoint.lane_id
        return response


    def spawn_actor(self, actor_type, actor_id, pose):
            request = SpawnObject.Request()
            request.type = actor_type
            request.id = actor_id
            request.transform = pose
            response = self.call_service(SpawnObject, "/carla/spawn_object", request)
            if response.id == -1 :
                print(response.error_string)
                print(f"Pose {pose} failed to spawn {request.id}, {request.type}")
                return response.id
            else:
                print(f"Pose {pose} spawned {response.id}, {request.type}")
                return response.id


    def destroy_actors(self):
        spawned_actors = self.spawned_vehicles + self.spawned_walkers
        for i, actor_id in enumerate(spawned_actors):
            request = DestroyObject.Request()
            request.id = actor_id
            response = self.call_service(DestroyObject, "/carla/destroy_object", request)
            print(f"[{i+1}], {request}, {response}")


    def call_service(self, service, path, request):
        client = self.create_client(service, path)
        # call as soon as ready
        ready = client.wait_for_service(timeout_sec=10.0)
        if not ready:
            raise RuntimeError(f'Wait for service {service} at {path} timed out')

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # handle response
        response = future.result()
        if response is None:
            e = future.exception()
            raise RuntimeError(
                f'Exception while calling service of node '
                "'{node_name}': {e}")

        self.destroy_client(client) # ignoring the response since it shouldn't return False, right???

        return response



def main():
    print("Starting SpawnEpisode...")
    rclpy.init()
    spawn_episode = SpawnEpisode()
    try:
        rclpy.spin(spawn_episode)
    except KeyboardInterrupt:
        print('Node stopped cleanly!')
    finally:
        print("Shutting down SpawnEpisode...")
        spawn_episode.destroy_actors()
        spawn_episode.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
   main()