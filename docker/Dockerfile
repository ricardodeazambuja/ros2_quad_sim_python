FROM ricardodeazambuja/ros2-galactic-desktop

USER ros2user
RUN mkdir -p /home/ros2user/carla-ros/src
WORKDIR /home/ros2user/carla-ros
RUN git clone --recurse-submodules https://github.com/ricardodeazambuja/carla-ros.git src/ros-bridge
RUN git clone --recurse-submodules https://github.com/ricardodeazambuja/ros2_quad_sim_python src/quad_sim_python

RUN sudo -H python3 -m pip install carla==0.9.13
RUN sudo -H python3 -m pip install -r src/ros-bridge/requirements.txt
RUN sudo -H python3 -m pip install quad_sim_python

RUN sudo apt-get update
RUN /bin/bash -c 'source /opt/ros/galactic/setup.bash; rosdep update; rosdep install --from-paths src --ignore-src -r; colcon build --symlink-install'

RUN echo "export PATH=$PATH:/home/ros2user/.local/bin" >> /home/ros2user/.bashrc
RUN echo "source /home/ros2user/carla-ros/install/setup.bash" >> /home/ros2user/.bashrc