import math

import rclpy # https://docs.ros2.org/latest/api/rclpy/api/node.html
from rclpy.node import Node

from geometry_msgs.msg import Pose
from carla_msgs.srv import SpawnObject, DestroyObject, GetBlueprints, SpawnPoints

class SpawnActorServer(Node):
    def __init__(self):
        super().__init__('spawn_actor')

        # Get the blueprints available in the current map
        response = self.call_service(GetBlueprints, "/carla/get_blueprints", GetBlueprints.Request())
        vehicles = [v for v in response.blueprints if "vehicle" in v]
        walkers = [w for w in response.blueprints if "walker" in w]
        self.get_logger().info(f'Available vehicles (this map): {vehicles}')
        self.get_logger().info(f'Available walkers (this map): {walkers}')

        self.spawned_actors = {}
        actors2spawn = input('Input the actors to spawn separated by \';\':')
        self.actors2spawn = actors2spawn.replace(" ", "").split(';')
        actorspose = input('Input the actor position+yaw (x,y,z,yaw) separated by \';\':')
        self.actorspose = [eval(i) for i in actorspose.split(';')]

        if len(self.actors2spawn)>0 and len(self.actors2spawn)==len(self.actorspose):
            for i,(actor,pose) in enumerate(zip(self.actors2spawn,self.actorspose)):
                posemsg = Pose()
                posemsg.position.x = float(pose[0])
                posemsg.position.y = float(pose[1])
                posemsg.position.z = float(pose[2])
                posemsg.orientation.w = float(math.cos(math.pi*pose[3]*0.5/180))
                posemsg.orientation.z = float(math.sin(math.pi*pose[3]*0.5/180))
                res_id = self.spawn_actor(actor, f"{actor.split('.')[0]}_{i:03d}", posemsg)
                if res_id != -1:
                    self.spawned_actors[res_id] = pose
                    self.get_logger().info(f'{actor}_{i:03d} spawned!')



    def spawn_actor(self, actor_type, actor_id, pose):
        request = SpawnObject.Request()
        request.type = actor_type
        request.id = actor_id
        request.transform = pose
        response = self.call_service(SpawnObject, "/carla/spawn_object", request)
        if response.id == -1 :
            self.get_logger().warn(response.error_string)
            self.get_logger().warn(f"Pose {pose} failed to spawn {request.id}, {request.type}")
            return response.id
        else:
            self.get_logger().info(f"Pose {pose} spawned {response.id}, {request.type}")
            return response.id
            

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
    

    def destroy_actors(self):
        spawned_actors = list(self.spawned_actors.keys())
        for i, actor_id in enumerate(spawned_actors):
            request = DestroyObject.Request()
            request.id = actor_id
            response = self.call_service(DestroyObject, "/carla/destroy_object", request)
            self.get_logger().warn(f"[{i+1}], {request}, {response}")

def main():
    print("Starting SpawnActorServer...")
    rclpy.init()
    spawn_episode = SpawnActorServer()
    try:
        rclpy.spin(spawn_episode)
    except KeyboardInterrupt:
        print('Node stopped cleanly!')
    finally:
        print("Shutting down SpawnActorServer...")
        spawn_episode.destroy_actors()
        spawn_episode.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
   main()
