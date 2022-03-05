import sys, os
curr_path = os.getcwd()
if os.path.basename(curr_path) not in sys.path:
    sys.path.append(os.path.dirname(os.getcwd()))

from time import sleep
from threading import Lock
import numpy as np
from scipy.spatial.transform import Rotation

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from quad_sim_python_msgs.msg import QuadMotors, QuadWind, QuadState

import rclpy # https://docs.ros2.org/latest/api/rclpy/api/node.html
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


from quad_sim_python import Quadcopter
from rclpy_param_helper import Dict2ROS2Params, ROS2Params2Dict

quad_params = {}
# Moments of inertia:
# (e.g. from Bifilar Pendulum experiment https://arc.aiaa.org/doi/abs/10.2514/6.2007-6822)
Ixx = 0.0123
Iyy = 0.0123
Izz = 0.0224
IB  = np.array([[Ixx, 0,   0  ],
                [0,   Iyy, 0  ],
                [0,   0,   Izz]]) # Inertial tensor (kg*m^2)

IRzz = 2.7e-5   # Rotor moment of inertia (kg*m^2)
quad_params["mB"]   = 1.2    # mass (kg)
quad_params["g"]    = 9.81   # gravity (m/s^2)
quad_params["dxm"]  = 0.16   # arm length (m) - between CG and front
quad_params["dym"]  = 0.16   # arm length (m) - between CG and right
quad_params["dzm"]  = 0.05   # motor height (m)
quad_params["IB"]   = IB
quad_params["IRzz"] = IRzz
quad_params["Cd"]         = 0.1      # https://en.wikipedia.org/wiki/Drag_coefficient
quad_params["kTh"]        = 1.076e-5 # thrust coeff (N/(rad/s)^2)  (1.18e-7 N/RPM^2)
quad_params["kTo"]        = 1.632e-7 # torque coeff (Nm/(rad/s)^2)  (1.79e-9 Nm/RPM^2)
quad_params["minThr"]     = 0.1*4    # Minimum total thrust
quad_params["maxThr"]     = 9.18*4   # Maximum total thrust
quad_params["minWmotor"]  = 75       # Minimum motor rotation speed (rad/s)
quad_params["maxWmotor"]  = 925      # Maximum motor rotation speed (rad/s)
quad_params["tau"]        = 0.015    # Value for second order system for Motor dynamics
quad_params["kp"]         = 1.0      # Value for second order system for Motor dynamics
quad_params["damp"]       = 1.0      # Value for second order system for Motor dynamics
quad_params["motorc1"]    = 8.49     # w (rad/s) = cmd*c1 + c0 (cmd in %)
quad_params["motorc0"]    = 74.7
# Select whether to use gyroscopic precession of the rotors in the quadcopter dynamics
# ---------------------------
# Set to False if rotor inertia isn't known (gyro precession has negigeable effect on drone dynamics)
quad_params["usePrecession"] = False

quad_params["Ts"] = 1/200 # state calculation time step (current ode settings run faster using a smaller value)
quad_params["Tp"] = 1/25 # period it publishes the current pose
quad_params["Tfs"] = 1/50 # period it publishes the full state
quad_params["orient"] = "ENU"
quad_params["target_frame"] = 'flying_sensor'
quad_params["map_frame"] = 'map'

class QuadSim(Node):
    def __init__(self):
        super().__init__('quadsim', 
                         allow_undeclared_parameters=True, # necessary for using set_parameters
                         automatically_declare_parameters_from_overrides=True) # allows command line parameters

        # Read ROS2 parameters the user may have set 
        # E.g. (https://docs.ros.org/en/foxy/How-To-Guides/Node-arguments.html):
        # --ros-args -p init_pose:=[0,0,0,0,0,0])
        # --ros-args --params-file params.yaml
        read_params = ROS2Params2Dict(self, 'quadsim', list(quad_params.keys()) + ["init_pose"])
        for k,v in read_params.items():
            # Update local parameters
            quad_params[k] = v
        
        # Update ROS2 parameters
        Dict2ROS2Params(self, quad_params) # the controller needs to read some parameters from here


        self.w_cmd_lock = Lock()
        self.wind_lock = Lock()
        self.sim_pub_lock = Lock()

        # pos[3], quat[4], rpy[3], vel[3], vel_dot[3], omega[3], omega_dot[3]
        self.curr_state = np.zeros(22, dtype='float64')

        self.wind = [0,0,0]
        self.prev_wind = [0,0,0]


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer for the tf
        # I couldn't find a way to receive it without using a timer 
        # to allow me to call lookup_transform after rclpy.spin(quad_node)
        self.tf_timer = self.create_timer(1.0, self.on_tf_timer)

    def on_tf_timer(self):
        if "init_pose" not in quad_params:
            # Look up for the transformation between target_frame and map_frame frames
            try:
                now = rclpy.time.Time()
                trans = self.tf_buffer.lookup_transform(
                    quad_params["map_frame"],
                    quad_params["target_frame"],
                    now)

                self.get_logger().info(f'TF received {trans}')
                init_pos = [trans.transform.translation.x, 
                            trans.transform.translation.y, 
                            trans.transform.translation.z]

                init_quat = [trans.transform.rotation.x,
                             trans.transform.rotation.y,
                             trans.transform.rotation.z,
                             trans.transform.rotation.w]

                init_rpy = Rotation.from_quat(init_quat).as_euler('xyz')
                
                quad_params["init_pose"] = np.concatenate((init_pos,init_rpy))
                # Update ROS2 parameters
                Dict2ROS2Params(self, {"init_pose": quad_params["init_pose"]}) # the controller needs to read some parameters from here

            except TransformException as ex:
                self.get_logger().error(f'Could not transform {quad_params["map_frame"]} to {quad_params["target_frame"]}: {ex}')
                quad_params["init_pose"] = [0,0,0,0,0,0]
                self.get_logger().error(f'init_pose not available... using {quad_params["init_pose"]}')
                # Update ROS2 parameters
                Dict2ROS2Params(self, {"init_pose": quad_params["init_pose"]})
        else:
            self.start_sim()


    def start_sim(self):   
        self.destroy_timer(self.tf_timer)

        params = ROS2Params2Dict(self, 'quadsim', quad_params.keys())
        init_pose = np.array(params['init_pose']) # x0, y0, z0, phi0, theta0, psi0
        init_twist = np.array([0,0,0,0,0,0]) # xdot, ydot, zdot, p, q, r
        init_states = np.hstack((init_pose,init_twist))
        self.t = 0
        self.Ts = params['Ts']
        self.quad = Quadcopter(self.t, init_states, params=params.copy(), orient=params['orient'])
        self.w_cmd = [self.quad.params['w_hover']]*4
        new_params = {key: self.quad.params[key] for key in self.quad.params if key not in params}
        Dict2ROS2Params(self, new_params) # some parameters are created by the quad object

        self.sim_loop_timer = self.create_timer(self.Ts, self.on_sim_loop)
        self.sim_publish_full_state_timer = self.create_timer(params['Tfs'], self.on_sim_publish_fs)
        self.sim_publish_pose_timer = self.create_timer(params['Tp'], self.on_sim_publish_pose)

        self.get_logger().info(f'Simulator started!')

        self.quadpos_pub = self.create_publisher(Pose, f'/carla/{quad_params["target_frame"]}/control/set_transform',1)
        self.quadstate_pub = self.create_publisher(QuadState, f'/quadsim/{quad_params["target_frame"]}/state',1)
        self.imu_pub = self.create_publisher(Imu, f'/quadsim/{quad_params["target_frame"]}/imu/data',1)

        self.receive_w_cmd = self.create_subscription(
            QuadMotors,
            f'/quadsim/{quad_params["target_frame"]}/w_cmd',
            self.receive_w_cmd_cb,
            1)

        self.receive_wind = self.create_subscription(
            QuadWind,
            f'/quadsim/{quad_params["target_frame"]}/wind',
            self.receive_wind_cb,
            1)

    def receive_w_cmd_cb(self, motor_msg):
        with self.w_cmd_lock:
            self.w_cmd = [motor_msg.m1, 
                          motor_msg.m2,
                          motor_msg.m3,
                          motor_msg.m4]
        self.get_logger().info(f'Received w_cmd: {self.w_cmd}')

    def receive_wind_cb(self, wind_msg):
        with self.wind_lock:
            self.wind = [wind_msg.vel_w, 
                         wind_msg.head_w,
                         wind_msg.elev_w]
        self.get_logger().info(f'Received wind: {self.wind}')

    def on_sim_loop(self):
        if self.wind_lock.acquire(blocking=False):
            self.prev_wind[:] = self.wind[:]
            self.wind_lock.release()

        with self.w_cmd_lock:
            self.quad.update(self.t, self.Ts, self.w_cmd, self.prev_wind)

        if self.sim_pub_lock.acquire(blocking=False):
            self.curr_state[0:3] = self.quad.pos[:]
            self.curr_state[3:7] = self.quad.quat[[1,2,3,0]] # the sim uses w x y z
            self.curr_state[7:10] = self.quad.euler[:]
            self.curr_state[10:13] = self.quad.vel[:]
            self.curr_state[13:16] = self.quad.vel_dot[:]
            self.curr_state[16:19] = self.quad.omega[:]
            self.curr_state[19:22] = self.quad.omega_dot[:]
            self.t += self.Ts
            self.sim_pub_lock.release()
        self.get_logger().info(f'Quad State: {self.curr_state}')

    def on_sim_publish_pose(self):
        pose_msg = Pose()
        with self.sim_pub_lock:
            pose_msg.position.x = float(self.curr_state[0])
            pose_msg.position.y = float(self.curr_state[1])
            pose_msg.position.z = float(self.curr_state[2])
            pose_msg.orientation.x = float(self.curr_state[3])
            pose_msg.orientation.y = float(self.curr_state[4])
            pose_msg.orientation.z = float(self.curr_state[5])
            pose_msg.orientation.w = float(self.curr_state[6])

        self.quadpos_pub.publish(pose_msg)

    def on_sim_publish_fs(self):
        state_msg = QuadState()
        imu_msg = Imu()
        with self.sim_pub_lock:
            now = rclpy.time.Time().to_msg()
            state_msg.header.stamp = now
            state_msg.t = self.t
            state_msg.pos = self.curr_state[0:3][:]
            state_msg.quat = self.curr_state[3:7][:]
            state_msg.rpy = self.curr_state[7:10][:]
            state_msg.vel = self.curr_state[10:13][:]
            state_msg.vel_dot = self.curr_state[13:16][:]
            state_msg.omega = self.curr_state[16:19][:]
            state_msg.omega_dot = self.curr_state[19:22][:]

        imu_msg.header.stamp = now
        imu_msg.orientation.x = state_msg.quat[0]
        imu_msg.orientation.y = state_msg.quat[1]
        imu_msg.orientation.z = state_msg.quat[2]
        imu_msg.orientation.w = state_msg.quat[3]
        imu_msg.angular_velocity.x = state_msg.omega[0]
        imu_msg.angular_velocity.y = state_msg.omega[1]
        imu_msg.angular_velocity.z = state_msg.omega[2]
        imu_msg.linear_acceleration.x = state_msg.vel[0]
        imu_msg.linear_acceleration.y = state_msg.vel[1]
        imu_msg.linear_acceleration.z = state_msg.vel[2]

        self.quadstate_pub.publish(state_msg)
        self.imu_pub.publish(imu_msg)
        self.get_logger().debug(f'Quad State: {self.curr_state}')

def main():
    print("Starting QuadSim...")
    rclpy.init()

    quad_node = QuadSim()
    try:
        rclpy.spin(quad_node)
    except KeyboardInterrupt:
        pass


    print("Shutting down QuadSim...")
    rclpy.shutdown()


if __name__ == '__main__':
   main()