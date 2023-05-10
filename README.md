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

Don't forget to turn the script executable to make life easier (`$ chmod +x launch_ros2_desktop.sh`).

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

The launchfile above exposes just a few parameters. You can verify using:
```
$ ros2 launch ros2_quad_sim_python launch_everything.launch.py --show-args
```

If you still want to use the same container to save some memory, you need to take note of the containers name (it will be `ros2-` plus five random characters). Here is how to check the topics without launching a new container (the container's name in this caase is `ros2-8589d9bd2d`):
```
$ docker exec -t ros2-8589d9bd2d bash -i -c "ros2 topic list"
```
Docker exec is a *oneway* thing, so I don't recommend using it with anything that needs to catch the `ctrl+c` (SIGINT) to gracefully stop (like `carla_spawn_objects.launch.py` needs to remove the actors it created from the CARLA server), it will not work.

Another useful command to use (considering your container's name is `ros2-bb93a782c4`) is:
```
$ docker exec -t ros2-bb93a782c4 bash -i -c "ros2 run plotjuggler plotjuggler"
```

If you just want to start a new terminal using the same container (useful when making changes to that container only during debugging):
```
$ docker exec -it ros2-bb93a782c4 bash
```

These are the commands to launch things individually:
```
$ ros2 launch carla_ros_bridge carla_ros_bridge.launch.py passive:=False town:=Town01
$ ros2 launch carla_spawn_objects carla_spawn_objects.launch.py objects_definition_file:=src/ros-bridge/carla_spawn_objects/config/flying_sensor.json
$ ros2 run rviz2 rviz2 --ros-args -d ~/carla-ros/install/ros2_quad_sim_python/share/ros2_quad_sim_python/cfg/rviz_flying_sensor.rviz
```
and
```
$ ros2 run ros2_quad_sim_python quad --ros-args -p init_pose:=[0,0,2,0,0,0] -p Px:=2 -p Py:=2 -p Pz:=1
```

or
```
$ ros2 run ros2_quad_sim_python quadsim --ros-args -p init_pose:=[0,0,2,0,0,0]
$ ros2 run ros2_quad_sim_python quadctrl --ros-args -p Px:=2 -p Py:=2 -p Pz:=1
```


Publish a new setpoint by using `$ros2 topic pub /quadctrl/flying_sensor/ctrl_sp quad_sim_python_msgs/msg/QuadControlSetPoint  "h` and hitting the `tab` twice so it will fill the rest of the message. Don't forget to remove the `-` the autocomplete insists adding at the end.   
Or publish a new twist setpoint by using `ros2 topic pub /quadctrl/flying_sensor/ctrl_twist_sp geometry_msgs/msg/Twist "l` and following the same hints from above.

Another option is to control the simulated quadcopter using the keyboard with the `teleop_twist_keyboard` package (where `flying_sensor` is the default name):
```
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r teleop_twist_keyboard:cmd_vel:=/quadctrl/flying_sensor/ctrl_twist_sp
```

## Time synchronisation
Now the `quadsim` node is calculating the difference between the machine time and the time CARLA ros-bridge publishes. This time difference is available as the parameter `carla_time_diff_ns`. The node `quadctrl` reads this parameter from `quadsim`. All messages are synchronised with CARLA ros-bridge, but the assumption is the hosts have their system times synchronised.

## Training Episode Generator
* [spawn_episode.py](https://github.com/ricardodeazambuja/ros2_quad_sim_python/blob/main/examples/spawn_episode.py): randomly choose a spawn point from the map and populate the area around with vehicles and walkers.


## Making changes
Don't forget to run `colcon build --symlink-install` if you change anything that is not just a Python script.

# TODO
* Add all the parameters to the launch files.
* Add an example publishing setpoints to quadctrl, reading the current state, etc.
* Add an example using the wind.
* Create a message for the potential field and update ros_quad_ctrl to subscribe to a topic receiving that message.
* Make the topic where the simulator publishes the poses a parameter so it can be easily changed allowing it to be used with other things than CARLA.
* Consider a better way to [synchronise time between the ROS2 nodes](https://design.ros2.org/articles/clock_and_time.html).
