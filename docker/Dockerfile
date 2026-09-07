
FROM dustynv/ros:humble-desktop-l4t-r32.7.1

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN apt-get update -y && apt-get upgrade -y && \
apt-get install -y curl wget zip vim 

SHELL ["/bin/bash", "-c", "-l"]


copy requirements.txt /tmp/requirements.txt

RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install -r /tmp/requirements.txt


WORKDIR /home/robotcont/
COPY bash_aliases  /root/.bash_aliases
RUN source /root/.bash_aliases

ENV ROS_DOMAIN_ID=10
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
