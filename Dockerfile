FROM ros:humble-ros-base

ENV DPKG_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y libgeometric-shapes-dev \
                       meshlab \
                       ros-humble-gazebo-ros-pkgs \
                       ros-humble-gazebo-ros2-control \
                       ros-humble-joint-state-broadcaster \
                       ros-humble-joint-state-publisher \
                       ros-humble-joint-trajectory-controller \
                       ros-humble-moveit \
                       ros-humble-robot-state-publisher \
                       ros-humble-ros2-control \
                       ros-humble-ros2-controllers \
                       ros-humble-rqt-graph

RUN apt-get update && \
    apt-get install -y bash-completion \
                       sudo \
                       vim

RUN apt-get update && \
    apt-get install -y pip \
                       python3-venv

