import time

import sys, os
curr_path = os.getcwd()
if os.path.basename(curr_path) not in sys.path:
    sys.path.append(os.path.dirname(os.getcwd()))

from threading import Lock
from copy import copy
import numpy as np
from numpy.linalg import norm

from geometry_msgs.msg import Twist
from quad_sim_python_msgs.msg import QuadMotors, QuadState, QuadControlSetPoint

import rclpy # https://docs.ros2.org/latest/api/rclpy/api/node.html
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rclpy_param_helper import Dict2ROS2Params, ROS2Params2Dict

from quad_sim_python import Controller


ctrl_params = {
            # Position P gains
            "Px"    : 2.0,
            "Py"    : 2.0,
            "Pz"    : 1.0,

            # Velocity P-D gains
            "Pxdot" : 5.0,
            "Dxdot" : 0.5,
            "Ixdot" : 5.0,

            "Pydot" : 5.0,
            "Dydot" : 0.5,
            "Iydot" : 5.0,

            "Pzdot" : 4.0,
            "Dzdot" : 0.5,
            "Izdot" : 5.0,

            # Attitude P gains
            "Pphi"   : 8.0,
            "Ptheta" : 8.0,
            "Ppsi"   : 1.5,

            # Rate P-D gains
            "Pp" : 1.5,
            "Dp" : 0.04,

            "Pq" : 1.5,
            "Dq" : 0.04 ,

            "Pr" : 1.0,
            "Dr" : 0.1,

            # Max Velocities (x,y,z) [m/s]
            "uMax" : 50.0,
            "vMax" : 50.0,
            "wMax" : 50.0,

            "saturateVel_separately" : True,

            # Max tilt [degrees]
            "tiltMax" : 50.0,

            # Max Rate [rad/s]
            "pMax" : 200.0,
            "qMax" : 200.0,
            "rMax" : 150.0,

             # Minimum velocity for yaw follow to kick in [m/s]
            "minTotalVel_YawFollow" : 0.1,

            "useIntegral" : True,    # Include integral gains in linear velocity control
            }

class QuadCtrl(Node):

    def __init__(self):
        super().__init__('quadctrl', 
                         allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)

        self.started = False
        self.quad_state = False
        self.t = None

        self.quadstate_lock = Lock()
        self.ctrl_sp_lock = Lock()

        self.curr_sp = QuadControlSetPoint()
        self.prev_sp = copy(self.curr_sp)

        # Read ROS2 parameters the user may have set 
        # E.g. (https://docs.ros.org/en/foxy/How-To-Guides/Node-arguments.html):
        # --ros-args -p Px:=5
        # --ros-args --params-file params.yaml
        read_params = ROS2Params2Dict(self, 'quadctrl', ctrl_params.keys())
        for k,v in read_params.items():
            # Update local parameters
            ctrl_params[k] = v
        
        # Update ROS2 parameters
        Dict2ROS2Params(self, ctrl_params) # the controller needs to read some parameters from here

        parameters_received = False
        while not parameters_received:
            quad_params_list = ['mB', 'g', 'IB', 'maxThr', 'minThr', 'orient', 'mixerFMinv', 'minWmotor', 'maxWmotor', 
                                'target_frame', 'carla_time_diff_ns']
            self.quad_params = ROS2Params2Dict(self, 'quadsim', quad_params_list)
            if quad_params_list == list(self.quad_params.keys()):
                parameters_received = True
            else:
                self.get_logger().warn(f'Waiting for quadsim parameters!')
                time.sleep(1)
        
        self.carla_time_diff_ns = self.quad_params['carla_time_diff_ns']

        self.receive_control_sp = self.create_subscription(
            QuadControlSetPoint,
            f"/quadctrl/{self.quad_params['target_frame']}/ctrl_sp",
            self.receive_control_sp_cb,
            1)

        self.receive_control_twist = self.create_subscription(
            Twist,
            f"/quadctrl/{self.quad_params['target_frame']}/ctrl_twist_sp",
            self.receive_control_twist_cb,
            1)

        self.receive_quadstate = self.create_subscription(
            QuadState,
            f"/quadsim/{self.quad_params['target_frame']}/state",
            self.receive_quadstate_cb,
            1)

        self.w_cmd_pub = self.create_publisher(QuadMotors, f"/quadsim/{self.quad_params['target_frame']}/w_cmd",1)

    def start_ctrl(self):
        params = ROS2Params2Dict(self, 'quadctrl', ctrl_params.keys())
        self.ctrl = Controller(self.quad_params, orient=self.quad_params['orient'], params=params)

        
    def receive_control_sp_cb(self, sp_msg):
        if not self.started:
            self.start_ctrl()
            self.started = True
            self.get_logger().info(f'Controller started!')

        with self.ctrl_sp_lock:
            self.curr_sp = sp_msg

        self.get_logger().info(f'Received control setpoint: {self.curr_sp}')


    def receive_control_twist_cb(self, twist):
        if not self.started:
            self.start_ctrl()
            self.started = True
            self.get_logger().info(f'Controller started!')

        with self.ctrl_sp_lock:
            now = Time(nanoseconds=(rclpy.clock.Clock().now().nanoseconds - self.carla_time_diff_ns)).to_msg()
            self.curr_sp.header.stamp = now
            self.curr_sp.ctrltype = "xyz_vel"
            self.curr_sp.pos = [0.0,0.0,0.0]
            self.curr_sp.vel = [twist.linear.x, twist.linear.y, twist.linear.z]
            self.curr_sp.acc = [0.0,0.0,0.0]
            self.curr_sp.thr = [0.0,0.0,0.0]
            self.curr_sp.yawtype = "twist"
            self.curr_sp.yawrate = twist.angular.z

        self.get_logger().info(f'Received twist setpoint: {self.curr_sp}')


    def receive_quadstate_cb(self, state_msg):
        self.quad_state = True
        if self.t == None:
            self.prev_t = self.t = state_msg.t
            return
        else:
            self.prev_t = self.t
            self.t = state_msg.t
        self.get_logger().info(f'Received QuadState: {state_msg}')

        if self.started:
            if self.ctrl_sp_lock.acquire(blocking=False):
                if self.curr_sp.yawtype == "twist":
                    self.curr_sp.yaw += (self.t-self.prev_t)*self.curr_sp.yawrate
                self.prev_sp = self.curr_sp
                self.ctrl_sp_lock.release()

            # Quaternion arrives as q = x,y,z,w [0,1,2,3]
            # So it needs to change to q = w,x,y,z [3,0,1,2]
            self.ctrl.control((self.t-self.prev_t), self.prev_sp.ctrltype, self.prev_sp.yawtype, 
                               self.prev_sp.pos, self.prev_sp.vel, self.prev_sp.acc, self.prev_sp.thr, 
                               self.prev_sp.yaw, self.prev_sp.yawrate,
                               state_msg.pos, state_msg.vel, state_msg.vel_dot, 
                               state_msg.quat[[3,0,1,2]], state_msg.omega, state_msg.omega_dot, state_msg.rpy[2])

            w_cmd = self.ctrl.getMotorSpeeds()
            motor_msg = QuadMotors()
            motor_msg.header.stamp = Time(nanoseconds=(rclpy.clock.Clock().now().nanoseconds - self.carla_time_diff_ns)).to_msg()
            motor_msg.m1 = int(w_cmd[0])
            motor_msg.m2 = int(w_cmd[1])
            motor_msg.m3 = int(w_cmd[2])
            motor_msg.m4 = int(w_cmd[3])
            self.w_cmd_pub.publish(motor_msg)


def main():
    print("Starting QuadCtrl...")
    rclpy.init()
    ctrl_node = QuadCtrl()
    try:
        rclpy.spin(ctrl_node)
    except KeyboardInterrupt:
        pass


    print("Shutting down QuadCtrl...")
    rclpy.shutdown()


if __name__ == '__main__':
   main()