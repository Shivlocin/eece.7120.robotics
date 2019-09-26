FROM ros:melodic-ros-core-bionic

# Add the default users
RUN useradd -ms /bin/bash student

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-melodic-ros-base=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*
