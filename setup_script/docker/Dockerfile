FROM ros:melodic-ros-core-bionic

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-melodic-ros-base=1.4.1-0* sudo   \
    && rm -rf /var/lib/apt/lists/*

RUN useradd -ms /bin/bash student && echo "student:student" | chpasswd && adduser student sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
