#!/bin/bash

# Update and install packages required to install dependencies
apt-get update --qq
apt install -y --no-install-recommends software-properties-common

# Add the PCL 1.9 PPA
sudo add-apt-repository -y ppa:felix928/pcl-1.9
apt update -qq
apt install -y --no-install-recommends libvtk7-jni
apt install -y --no-install-recommends libpcl-dev
