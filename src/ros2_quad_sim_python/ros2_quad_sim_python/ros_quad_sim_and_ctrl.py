import time

import sys, os
curr_path = os.getcwd()
if os.path.basename(curr_path) not in sys.path:
    sys.path.append(os.path.dirname(os.getcwd()))

from threading import Lock
from copy import copy

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time

from geometry_msgs.msg import Twist
from quad_sim_python_msgs.msg import QuadControlSetPoint

from rclpy_param_helper import Dict2ROS2Params, ROS2Params2Dict

from quad_sim_python import Controller

from . import ros_quad_sim as rqs

ctrl_params = {
            # Position P gains
            "Px"    : 5.0,
            "Py"    : 5.0,
            "Pz"    : 2.0,

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
            "Pphi"   : 4.0,
            "Ptheta" : 4.0,
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
            "tiltMax" : 30.0,

            # Max Rate [rad/s]
            "pMax" : 200.0,
            "qMax" : 200.0,
            "rMax" : 150.0,

             # Minimum velocity for yaw follow to kick in [m/s]
            "minTotalVel_YawFollow" : 0.1,

            "useIntegral" : True,    # Include integral gains in linear velocity control
            }

class QuadSimAndCtrl(rqs.QuadSim):
    def __init__(self):
        # NOTE: you MUST set the parameters while launching the node as it won't read those ever again!
        # --ros-args -p Px:=5
        # --ros-args --params-file params.yaml
        
        self.receive_w_cmd = None # needed to wait for the quadsim

        super().__init__()

        self.quadsim_timer = self.create_timer(1.0, self.on_quadsim_timer)

    def on_quadsim_timer(self):
        # a far from elegant way to wait for quadsim...
        if self.receive_w_cmd is None:
            self.get_logger().info(f'Waiting for quadsim...')
            return
        # Now we don't need the time anymore
        self.destroy_timer(self.quadsim_timer)
        
        # the motor commands (w_cmd) will be generated in this node, 
        # therefore we don't need this subscriber
        self.destroy_subscription(self.receive_w_cmd)

        # the quad state will be directly accessed in this node, 
        # therefore we don't need this timer publishing it
        self.destroy_timer(self.sim_publish_full_state_timer)
        self.destroy_publisher(self.quadstate_pub)

        self.started = False
        self.ctrl_t = None

        self.ctrl_sp_lock = Lock()

        self.curr_sp = QuadControlSetPoint()
        self.prev_sp = QuadControlSetPoint()

        # Read ROS2 parameters the user may have set 
        # E.g. (https://docs.ros.org/en/foxy/How-To-Guides/Node-arguments.html):
        # --ros-args -p Px:=5
        # --ros-args --params-file params.yaml
        # Here 'quadsim' is used because this class inherits from ros_quad_sim.QuadSim
        read_params = ROS2Params2Dict(self, 'quadsim', list(ctrl_params.keys())+["Tfs"])
        for k,v in read_params.items():
            # Update local parameters
            ctrl_params[k] = v
        
        # Update ROS2 parameters
        Dict2ROS2Params(self, ctrl_params) # the controller needs to read some parameters from here

        parameters_received = False
        while not parameters_received:
            quad_params_list = ['mB', 'g', 'IB', 'maxThr', 'minThr', 'orient', 'mixerFMinv', 'minWmotor', 'maxWmotor', 
                                'target_frame']
            self.quad_params = ROS2Params2Dict(self, 'quadsim', quad_params_list)
            if quad_params_list == list(self.quad_params.keys()):
                parameters_received = True
            else:
                self.get_logger().warn(f'Waiting for quadsim parameters!')
                time.sleep(1)

        # Start the controller
        self.ctrl = Controller(self.quad_params, orient=self.quad_params['orient'], params=ctrl_params)

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
        
        self.ctrl_loop_timer = self.create_timer(ctrl_params['Tfs'], self.on_ctrl_loop_timer)

    def receive_control_sp_cb(self, sp_msg):
        with self.ctrl_sp_lock:
            self.curr_sp = sp_msg
        self.get_logger().info(f'Received control setpoint: {self.curr_sp}')


    def receive_control_twist_cb(self, twist):
        with self.ctrl_sp_lock:
            now = Time(nanoseconds=self.t*1E9).to_msg()
            self.curr_sp.header.stamp = now
            self.curr_sp.ctrltype = "xyz_vel"
            self.curr_sp.pos = [0.0,0.0,0.0]
            self.curr_sp.vel = [twist.linear.x, twist.linear.y, twist.linear.z]
            self.curr_sp.acc = [0.0,0.0,0.0]
            self.curr_sp.thr = [0.0,0.0,0.0]
            self.curr_sp.yawtype = "twist"
            self.curr_sp.yawrate = twist.angular.z

        self.get_logger().info(f'Received twist setpoint: {self.curr_sp}')

    def on_ctrl_loop_timer(self):
        # Lock from quadsim...
        # It doesn't make sense to generate new motor speed values
        # without having an update for the quad states
        with self.sim_pub_lock:
            if self.ctrl_t is None:
                self.ctrl_prev_t = self.ctrl_t = self.t
                self.curr_sp.pos = copy(self.curr_state[0:3][:])
                return
            else:
                self.ctrl_prev_t = self.ctrl_t
                self.ctrl_t = self.t
            ctrl_pos = copy(self.curr_state[0:3][:])
            ctrl_quat = copy(self.curr_state[3:7][:])
            ctrl_rpy = copy(self.curr_state[7:10][:])
            ctrl_vel = copy(self.curr_state[10:13][:])
            ctrl_vel_dot = copy(self.curr_state[13:16][:])
            ctrl_omega = copy(self.curr_state[16:19][:])
            ctrl_omega_dot = copy(self.curr_state[19:22][:])


        if self.ctrl_sp_lock.acquire(blocking=False):
            if self.curr_sp.yawtype == "twist":
                self.curr_sp.yaw += (self.ctrl_t-self.ctrl_prev_t)*self.curr_sp.yawrate
            self.prev_sp = self.curr_sp
            self.ctrl_sp_lock.release()

        # Quaternion arrives as q = x,y,z,w [0,1,2,3]
        # So it needs to change to q = w,x,y,z [3,0,1,2]
        self.ctrl.control((self.ctrl_t-self.ctrl_prev_t), self.prev_sp.ctrltype, self.prev_sp.yawtype, 
                            self.prev_sp.pos, self.prev_sp.vel, self.prev_sp.acc, self.prev_sp.thr, 
                            self.prev_sp.yaw, self.prev_sp.yawrate,
                            ctrl_pos, ctrl_vel, ctrl_vel_dot, 
                            ctrl_quat[[3,0,1,2]], ctrl_omega, ctrl_omega_dot, ctrl_rpy[2])

        w_cmd = self.ctrl.getMotorSpeeds()
        # Lock from quadsim
        # Send the new motor speed values
        with self.w_cmd_lock:
            self.w_cmd = [int(m) for m in w_cmd]
            self.get_logger().debug(f'Sending w_cmd: {self.w_cmd}')


def main(args=None):
    print("Starting QuadSimAndCtrl...")
    rclpy.init(args=args)
    
    try:
        quad_node = QuadSimAndCtrl()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(quad_node)

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            quad_node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()