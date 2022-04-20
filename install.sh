#!/bin/bash

set -x
set -e

echo "cloning SoapySDr"
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
git checkout change_rx_tx
mkdir build
cd build
echo "Making SoapySidekiq"
cmake ../
make
echo "Installing SoapySidekiq"
sudo make install

cd ../


