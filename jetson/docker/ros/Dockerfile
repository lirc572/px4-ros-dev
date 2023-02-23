FROM dustynv/ros:melodic-ros-base-l4t-r32.7.1

WORKDIR /root

RUN apt-get update && apt-get install -y \
    ros-melodic-mavros \
    ros-melodic-mavros-extras \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | bash

CMD ["sleep", "infinity"]
