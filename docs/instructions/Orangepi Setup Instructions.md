# Step 1: Orangepi OS Setup

* Download and install Orangpi OS Arch  
  * [https://drive.google.com/drive/folders/1MADshGYBZWTPIeDg7XofnZZidyBbGOaB](https://drive.google.com/drive/folders/1MADshGYBZWTPIeDg7XofnZZidyBbGOaB)   
* Use BalenaEtcher to install on microsd card and insert into OrangePi 5 pro before powering on  
* Open OrangepiOS on a monitor and open the terminal emulator app

`ip a`

* Record the ip and use it to connect via PuTTY or SSH  
* Setup Kernel Headers

`git clone https://github.com/orangepi-xunlong/orangepi-build.git`   
`cd orangepi-build`  
`sudo ./build.sh BOARD=orangepi5pro BRANCH=current BUILD_OPT=kernel`  
`sudo dpkg -i output/debs/linux-headers-*.deb`  
`sudo apt-get install gasket-dkms`  
`sudo modprobe gasket`   
`sudo modprobe apex`  
`lsmod | grep gasket`   
`lsmod | grep apex`  
`ls /dev/apex_0`  
`sudo apt-get update`   
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list

curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add \-

sudo apt-get update

sudo apt-get install gasket-dkms libedgetpu1-std

sudo sh \-c "echo 'SUBSYSTEM==\\"apex\\", MODE=\\"0660\\", GROUP=\\"apex\\"' \>\> /etc/udev/rules.d/65-apex.rules"

sudo groupadd apex

sudo adduser $USER apex

`sudo apt-get install python3-pycoral`  
`sudo apt-get install fiona`

* Create FSD Repository and setup python virtual environment

`sudo apt update`   
`sudo apt install software-properties-common`  
**`sudo add-apt-repository ppa:deadsnakes/ppa`**  
**`sudo apt update`**  
**`sudo apt install python3.9-dev`**  
**`python3.9 --version`**  
**`sudo apt install python3.9-venv`**  
**`python3.9 -m venv FSD`**  
**`source FSD/bin/activate`**

* Install GDown and download the python script

`pip install gdown`  
`gdown 1Xxhrcmj2vQ8nA1_0AThj7u6qYy9Jn3EQ`  
`pip install numpy opencv-python tflite-runtime`

`gdown 1lKXSH7tzjoGs91PGNln3yHtk7pjlS2-g`  
`tar -xf linuxSDK_V2.1.0.43(240126).tar.gz`  
`cd linuxSDK_V2.1.0.43(240126)`  
`sudo ./install.sh`  
`sudo reboot now`

`mv linuxSDK_V2.1.0.43(240126)/demo/python_demo/mvsdk.py ../../../FSD`

`gdown 1UtsNgHtq4b4xJy7N-mMhpoTLv_yQdsGn`  
`unzip unilidar_sdk-main.zip`

`gdown 1DfN4gWNDDalJySRlGgY2P8CBOO46PCgm`

`sudo apt-get install cmake`  
`mv lidarphoto.cpp unilidar_sdk-main/unitree_lidar_sdk/examples/example_lidar.cpp`  
`cd unilidar_sdk-main/unitree_lidar_sdk`  
`mkdir build`  
`cd build`  
`cmake .. && make -j2`  
`mv ../bin/example_lidar ~/FSD/lidarphoto`  
`cd ~`  
