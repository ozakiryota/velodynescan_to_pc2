########## Pull ##########
FROM ros:noetic
########## Non-interactive ##########
ENV DEBIAN_FRONTEND=noninteractive
########## Common tool ##########
RUN apt-get update && \
	apt-get install -y \
		vim \
		wget \
		unzip \
		git \
        python-tk
########## ROS setup ##########
RUN mkdir -p ~/catkin_ws/src && \
	cd ~/catkin_ws && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make" && \
	echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
	echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
	echo "export ROS_WORKSPACE=~/catkin_ws" >> ~/.bashrc
## cmk
RUN echo 'function cmk(){ \n\
		lastpwd=$OLDPWD \n\
		cpath=$(pwd) \n\
		cd ~/catkin_ws \n\
		catkin_make $@ \n\
		cd $cpath \n\
		OLDPWD=$lastpwd \n\
        }' >> ~/.bashrc
########## Velodyne ##########
RUN apt-get update && \
	apt-get install -y \
		libpcap-dev && \
	cd ~/catkin_ws/src && \
	git clone https://github.com/ros-drivers/velodyne.git && \
	cd ~/catkin_ws/src/velodyne && \
	git checkout 8feed85a30026c6be5196ae015e5f115c1624459 && \
	cd ~/catkin_ws/ && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make"
########## Cache busting ##########
ARG CACHEBUST=1
########## velodynescan_to_pc2 ##########
RUN apt-get update && \
    apt-get install -y \
		ros-noetic-pcl-conversions \
		ros-noetic-rviz && \
	cd ~/catkin_ws/src && \
	git clone https://github.com/ozakiryota/velodynescan_to_pc2.git && \
	cd ~/catkin_ws && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make"
########## Initial position ##########
WORKDIR /root/catkin_ws/src/velodynescan_to_pc2