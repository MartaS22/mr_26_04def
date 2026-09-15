FROM ros:jazzy
 
ENV DEBIAN_FRONTEND=noninteractive
 
# 1. Configurazione APT e strumenti base di compilazione e sviluppo
RUN apt-get update -y && apt-get install -y \
    wget \
    curl \
    git \
    vim \
    cmake \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*
 
# 2. Gazebo Sim (Harmonic) e ROS-GZ Bridge
RUN apt-get update -y && apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-xacro \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-gz-sim-vendor \
    ros-jazzy-gz-ros2-control \
    && rm -rf /var/lib/apt/lists/*
 
# 3. TurtleBot3 Packages (modelli e simulazioni)
RUN apt-get update -y && apt-get install -y \
    ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-turtlebot3-simulations \
    && rm -rf /var/lib/apt/lists/*
 
# 4. Navigazione autonoma (Nav2) e SLAM
RUN apt-get update -y && apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-simple-commander \
    ros-jazzy-slam-toolbox \
    ros-jazzy-tf-transformations \
    && rm -rf /var/lib/apt/lists/*
 
# 5. Visione artificiale e riconoscimento (per i token)
RUN apt-get update -y && apt-get install -y \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-apriltag-ros \
    python3-opencv \
    python3-numpy \
    python3-scipy \
    python3-transforms3d \
    && rm -rf /var/lib/apt/lists/*
 
# 6. Inizializzazione di rosdep
RUN rosdep init || true && rosdep update
 
# 7. Configurazione workspace ROS 2
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws
 
# 8. Variabili d'ambiente nel .bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /ros_ws/install/setup.bash 2>/dev/null || true" >> /root/.bashrc
RUN echo "export TURTLEBOT3_MODEL=waffle" >> /root/.bashrc
RUN echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc
RUN echo "export GZ_SIM_RESOURCE_PATH=/ros_ws/src:\$GZ_SIM_RESOURCE_PATH" >> /root/.bashrc
 

CMD ["bash"]