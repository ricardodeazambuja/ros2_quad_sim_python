# ros2_quad_sim_python
ROS2 packages to simulate (and control) a quadcopter. These packages were created to be used with a flying sensor in CARLA. The flying sensor is a `bodiless` actor that only has sensors and it will go through the scenario without collisions. The `flying_sensor` definition can be found [here](https://github.com/ricardodeazambuja/carla-ros/blob/master/carla_spawn_objects/config/flying_sensor.json).


# How to use:

## CARLA Simulator
Start the CARLA simulator in headless mode (https://github.com/ricardodeazambuja/carla-simulator-python):
```
$ docker run --rm -it \
--name carla-container \
--hostname=carla-container \
--user carla \
-p 2000-2002:2000-2002 \
--gpus 0 \
ricardodeazambuja/carlasim:0.9.13_headless ./launch_headless.sh
```

## ROS2 and the necessary packages
All the instructions can be found in the Dockerfile: 
* https://github.com/ricardodeazambuja/ros2_quad_sim_python/blob/main/docker/Dockerfile


### Get the script to launch ROS2
* https://github.com/ricardodeazambuja/ros2-playground/blob/main/launch_ros2_desktop.sh

Don't forget to make the script executable to make life easier (`$ chmod +x launch_ros2_desktop.sh`).

### Launch the container
If you don't want to create your own docker image, you can spawn containers using this command:
```
$ ./launch_ros2_desktop.sh -g --image ricardodeazambuja/ros2_quad_sim_python
```
It will mount the current directory inside the container and try to source `install/setup.bash`. 

You can launch as many containers as you need. It's easier than using `docker exec` for most situations. Inside one of the containers, launch everything:

```
$ ros2 launch ros2_quad_sim_python launch_everything.launch.py
```

These are the commands to launch things individually:
```
$ ros2 launch carla_ros_bridge carla_ros_bridge.launch.py passive:=False town:=Town01
$ ros2 launch carla_spawn_objects carla_spawn_objects.launch.py objects_definition_file:=src/ros-bridge/carla_spawn_objects/config/flying_sensor.json
$ ros2 run quad_sim_python quadsim --ros-args -p init_pose:=[0,0,2,0,0,0]
$ ros2 run quad_sim_python quadctrl --ros-args -p Px:=2 -p Py:=2 -p Pz:=1
$ ros2 run rviz2 rviz2 --ros-args -d ~/carla-ros/install/ros2_quad_sim_python/share/ros2_quad_sim_python/cfg/rviz_flying_sensor.rviz
```

Publish a new setpoint by using `$ros2 topic pub /quadctrl/flying_sensor/ctrl_sp quad_sim_python_msgs/msg/QuadControlSetPoint  "h` and hitting the `tab` twice so it will fill the rest of the message. Don't forget to remote the `-` the autocomplete insists adding at the end.

## Making changes
Don't forget to run `colcon build --symlink-install` if you change anything that is not just a Python script.

# TODO
* Add all the parameters to the launch files.
* Add an example publishing setpoints to quadctrl, reading the current state, etc.
* Add an example using the wind.
* Create a message for the potential field and update ros_quad_ctrl to subscribe to a topic receiving that message.
* Make the topic where the simulator publishes the poses a parameter so it can be easily used with other things than CARLA.
