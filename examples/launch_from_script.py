"""Launch ROS2 launch files using subprocess

This script allows to send "ctrl+c" to automate the gracefully killing of launch files (nodes).

I suppose this could be done using Executors... but I think it's still easier using subprocess.
"""

from subprocess import Popen, PIPE, TimeoutExpired
import signal
from time import sleep
import os

town = "Town01"
ros_cmd = f"ros2 launch carla_ros_bridge carla_ros_bridge.launch.py town:={town}"

# First option will use the same shell as the parent
proc = Popen(ros_cmd.split(" "), stdout=PIPE, stderr=PIPE, universal_newlines=True)

# The second option manually launches a new bash shell
# proc = Popen(['/bin/bash', '-i', '-c', ros_cmd], stdout=PIPE, stderr=PIPE, universal_newlines=True)

# The third option tells Popen to use a new shell with /bin/bash and it sources ros2 stuff
# ros_cmd = f"source /home/ros2user/carla-ros/install/setup.bash && ros2 launch carla_ros_bridge carla_ros_bridge.launch.py town:={town}"
# proc = Popen(ros_cmd, stdout=PIPE, stderr=PIPE, universal_newlines=True, shell=True, executable="/bin/bash")

print(proc.args)


class TimeoutError(Exception):
    pass

def handler(signum, frame):
    raise TimeoutError()

signal.signal(signal.SIGALRM, handler)


try:
    print("terminating")
    try:
        timeout = 5
        signal.alarm(timeout)
        stdout  = proc.stdout
        i=next(stdout, None)
        while i is not None:
            signal.alarm(timeout) # reset alarm
            print(i, end="")
            i=next(stdout, None)
    except TimeoutError as exc:
        print("Node stopped printing outputs?")
    finally:
        signal.alarm(0) # deactivate alarm
    
    print(f"Is it dead? {proc.poll() is not None}")
    proc.send_signal(signal.SIGINT) # ctrl+c
    proc.wait(timeout=10) # allow the ros2 to gracefully exit...
    print(f"Is it dead? {proc.poll() is not None}")
    proc.terminate()
    proc.kill()
    proc.wait()
    print("done")
except KeyboardInterrupt:
    print(f"Is it dead? {proc.poll() is not None}")
    try:
        outs, errs = proc.communicate(timeout=10)
    except TimeoutExpired:
        proc.send_signal(signal.SIGINT)
        outs, errs = proc.communicate()
    finally:
        print(outs.decode())
        print(errs.decode())