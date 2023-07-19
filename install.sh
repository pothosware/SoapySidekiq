#!/bin/bash

set -x
set -e

sudo rm -rf Soapy*

sudo apt update

sudo apt-get install -y git

sudo apt-get install -y cmake g++ libpython3-dev python3-numpy swig

sudo apt install -y soapysdr-tools

sudo apt-get install -y python3-dev python3-pip 

pip install numpy

pip install pynmea


echo "cloning SoapySDR"
git clone https://github.com/pothosware/SoapySDR.git
cd SoapySDR
git checkout master
mkdir build
cd build
echo "Making SoapySDr"
cmake ../
make
echo "Installing SoapySDr"
sudo make install

cd ../../



echo "cloning SoapySidekiq"
git clone https://github.com/epiqsolutions/SoapySidekiq.git
cd SoapySidekiq
mkdir build
cd build
echo "Making SoapySidekiq"
cmake ../
make
echo "Installing SoapySidekiq"
sudo make install

sudo ldconfig

cd ../

export LD_LIBRARY_PATH=/usr/lib/epiq:$LD_LIBRARY_PATH
