#!/bin/bash

###############################################################################
# SpiriGo dev envrionment installer for nVidia Jetson TK1 
# --------------------------------
# This script installs a SpiriGo system on a Jetson computer meant to be a
# companion computer on a flying ArduPilot quadcopter. DO NOT run this on
# any other systems as it is meant specifically for the Jetson.
#
# By default, this script will install the SpiriGo workspace in the current user's
# home directory and all of its dependencies (including libraries and database
# servers) at the system level. 
#
# This script is meant to run from a fresh Grinch kernel. We have a Grinch 
# provision script but that one requires a reboot.
#
#
###############################################################################
set -e

###############################################################################
# Configuration
###############################################################################
# which user to install the code for; defaults to the user invoking this script
SPIRI_USER=${SPIRI_USER:-$SUDO_USER}

# the root directory to base the install in. must exist already
SPIRI_HOME=${SPIRI_HOME:-/home/$SPIRI_USER}

# check that the data directory exists, if not create it
if !([ -d $SPIRI_HOME/temp ]); then
    mkdir -p $SPIRI_HOME/temp
fi
DATA=${DATA:-$SPIRI_HOME/temp}

###############################################################################
# Sanity Checks
###############################################################################
if [[ $EUID -ne 0 ]]; then
    echo "ERROR: Must be run with root privileges."
    exit 1
fi

# seriously! these checks are here for a reason. the packages from the
# reddit ppa aren't built for anything but trusty (14.04) right now, so
# if you try and use this install script on another release you're gonna
# have a bad time.
source /etc/lsb-release
if [ "$DISTRIB_ID" != "Ubuntu" -o "$DISTRIB_RELEASE" != "14.04" ]; then
    echo "ERROR: Only Ubuntu 14.04 is supported."
    exit 1
fi

###############################################################################
# Initial Configuration
###############################################################################
set -x

# set some warning colors 
export RED='\033[0;31m'
export GREEN='\033[0;32m'
export YELLOW='\033[0;33m'
export NO_COLOR='\033[0m'

# aptitude, git, and CUDA configuration
APTITUDE_OPTIONS="-y"
export DEBIAN_FRONTEND=noninteractive
export GIT_SSL_NO_VERIFY=1
export CUDA_VERSION="cuda-repo-l4t-r21.3-6-5-prod_6.5-42_armhf"

# do a hold here to prevent OpenGL from being overwritten
apt-mark hold xserver-xorg-core

# run an aptitude update 
apt-get install $APTITUDE_OPTIONS dpkg git
apt-add-repository universe
apt-add-repository multiverse
apt-add-repository restricted
sudo apt-get update


###############################################################################
# Install CUDA 6.5
###############################################################################

# source: JetsonHacks
if (($(dpkg -l | grep cuda-toolkit-6-5 | wc -l) == 0))
then
    echo "Installing CUDA 6.5"
    if [ -f $DATA/$CUDA_VERSION".deb" ]
    then
        CSUM="7652f8afcd01e5c82dc5c202a902469a"
        MD5=$(md5sum $DATA/$CUDA_VERSION".deb" | cut -d ' ' -f 1)
        #echo $CSUM
        #echo $MD5
        if [ "$MD5" != "$CSUM" ]
        then
        #   rm $CUDA_VERSION".deb"
            printf ${YELLOW}"Found invalid file "$CUDA_VERSION".deb. Program will attempt to download it again.\N"${NO_COLOR}
            wget http://developer.download.nvidia.com/embedded/L4T/r21_Release_v3.0/$CUDA_VERSION".deb" -P $DATA
        else
            printf ${GREEN}"Found valid file "$CUDA_VERSION".deb\n"${NO_COLOR}
        fi
    else
        wget http://developer.download.nvidia.com/embedded/L4T/r21_Release_v3.0/$CUDA_VERSION".deb" -P $DATA
    fi

    dpkg -i $DATA/$CUDA_VERSION".deb"
    # Download & install the actual CUDA Toolkit including the OpenGL toolkit from NVIDIA. 
    apt-get update
    apt-get install $APTITUDE_OPTIONS cuda-toolkit-6-5

    # Add yourself to the "video" group to allow access to the GPU
    sudo usermod -a -G video $SPIRI_USER
    #Add the 32-bit CUDA paths to your .bashrc login script, and start using it in your current console:

    echo "# Add CUDA bin & library paths:" >> $SPIRI_HOME/.bashrc
    echo "export PATH=/usr/local/cuda-6.5/bin:$PATH" >> $SPIRI_HOME/.bashrc
    echo "export LD_LIBRARY_PATH=/usr/local/cuda-6.5/lib:$LD_LIBRARY_PATH" >> $SPIRI_HOME/.bashrc
    source $SPIRI_HOME/.bashrc
    printf ${GREEN}"CUDA 6.5 installed \n"${NO_COLOR}
else
    printf ${GREEN}"CUDA 6.5 is already installed on your system \n"${NO_COLOR}
fi

###############################################################################
# Install OpenCV4Tegra and configure it correctly
###############################################################################
# https://devtalk.nvidia.com/default/topic/835118/embedded-systems/incorrect-configuration-in-opencv4tegra-debian-packages-and-solution

# do the official OpenCV4Tegra installation
wget http://developer.download.nvidia.com/embedded/OpenCV/L4T_21.1/libopencv4tegra-repo_l4t-r21_2.4.10.1_armhf.deb -P $DATA
dpkg -i $DATA/libopencv4tegra-repo_l4t-r21_2.4.10.1_armhf.deb
apt-get update
apt-get install $APTITUDE_OPTIONS libopencv4tegra libopencv4tegra-dev libopencv4tegra-python

# then UNINSTALL OpenCV4Tegra to fix the debian files
apt-get remove --force-yes libopencv4tegra libopencv4tegra-dev libopencv4tegra-python 
dpkg --purge libopencv4tegra libopencv4tegra-dev libopencv4tegra-python

# source the prepared debian files to reinstall OpenCV4Tegra properly
dpkg -i $DATA/libopencv4tegra_2.4.10.1_armhf_mod.deb 
dpkg -i $DATA/libopencv4tegra-dev_2.4.10.1_armhf_mod.deb 
dpkg -i $DATA/libopencv4tegra-python_2.4.10.1_armhf_mod.deb

# install OpenCV4Tegra again
echo "Installing OpenCV4Tegra"
apt-get update
apt-get install $APTITUDE_OPTIONS libopencv4tegra libopencv4tegra-dev libopencv4tegra-python

echo "To avoid future conflicts, tell the system to retain the present installation of the OpenCV4Tegra library"
apt-mark hold libopencv4tegra libopencv4tegra-dev libopencv4tegra-python libopencv4tegra-repo
printf ${GREEN}"OpenCV4Tegra installed and debian files fixed \n"${NO_COLOR}

###############################################################################
# Install ROS 
###############################################################################
# taken from https://pixhawk.org/dev/ros/installation

# add the ROS repository for Trusty(14.04):
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'

# det the official ROS key
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

# and install the packages we recommend for having a development environment good for MAVs. 
# you can install any other project-specific dependencies via rosdep or manually.
sudo apt-get update
sudo apt-get install $APTITUDE_OPTIONS \
                    ros-indigo-ros-base \
                    ros-indigo-usb-cam \
                    ros-indigo-mavlink \
                    ros-indigo-mavros \
                    ros-indigo-cv-bridge \
                    ros-indigo-image-proc \
                    ros-indigo-tf

# Setup rosdep which makes life a lot easier by auto-installing dependencies wherever possible.
sudo apt-get install $APTITUDE_OPTIONS python-rosdep
sudo rosdep init
rosdep update

# Gotta source the ROS binary path
echo "source /opt/ros/indigo/setup.bash" >> $SPIRI_HOME/.bashrc
source /opt/ros/indigo/setup.bash

###############################################################################
# Install SpiriGo repository
###############################################################################
WORKSPACE=$SPIRI_HOME/spiri_ws

# Make workspace directory then "compile" to initialize the ROS workspace
mkdir -p $WORKSPACE/src
cd $WORKSPACE/src
catkin_init_workspace
cd $WORKSPACE
catkin_make 

# Download spiri_go package into source 
cd $WORKSPACE/src
git clone https://github.com/Pleiades-Spiri/spiri_go.git

# Now compile the one package we have
cd $WORKSPACE
catkin_make

# Gotta source the package paths for roslaunch/rosrun to find them
sh -c "echo 'source $WORKSPACE/devel/setup.bash' >> $SPIRI_HOME/.bashrc"
source $WORKSPACE/devel/setup.bash


