FROM ros:humble-ros-base-jammy

# Set up timezone
ENV TZ=Etc/UTC
RUN echo $TZ > /etc/timezone && \
  ln -fs /usr/share/zoneinfo/$TZ /etc/localtime

# Install required utilities
RUN apt update \
&& apt install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    gnupg2 \
    nano \
    python3-dbg \
    python3-setuptools \
    python3-vcstool \
    ruby \
    sudo \
&& rm -rf /var/lib/apt/lists/* \
&& apt clean -qq

# Set up locale
RUN sudo apt update && sudo apt install locales \
&& sudo locale-gen en_US en_US.UTF-8 \
&& sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ARG ROSDIST=humble

# Install example ROS package
RUN apt update \
&& apt install -y --no-install-recommends \
  ros-${ROSDIST}-actuator-msgs \
&& rm -rf /var/lib/apt/lists/* \
&& apt clean -qq

# Copy over script to Docker container
COPY ./run_my_system.bash /

# Use your ros_entrypoint
COPY ./ros_entrypoint.sh /

