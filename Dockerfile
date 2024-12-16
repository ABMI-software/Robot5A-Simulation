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
                       ros-humble-ros2-controllers

ARG USER
ARG UID
ARG GROUP
ARG HOME
RUN groupadd --non-unique --gid "$GROUP" "$USER" && \
    useradd  --non-unique --gid "$GROUP" --uid "$UID" --create-home --home-dir "$HOME" --shell /bin/sh "$USER"
