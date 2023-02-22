FROM px4io/px4-dev-ros-melodic:2021-12-11

WORKDIR /root

RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive

WORKDIR /root/PX4-Autopilot

# RUN bash ./Tools/setup/ubuntu.sh

# RUN wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh \
#     && bash ubuntu_sim_ros_melodic.sh \
#     && rm ubuntu_sim_ros_melodic.sh

CMD ["sleep", "infinity"]
