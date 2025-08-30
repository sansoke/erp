# Run avt_vimba_camera on ROS Kinetic
## 0. Pre-requisite
0. vimba sdk
	- [Vimba: The SDK for Allied Vision Cameras  -  Allied Vision](https://www.alliedvision.com/en/products/software.html)
	- Download proper SDK version in the above link.
	- `cd ~/Downloads`
	- `tar -xvzf Vimba_v2.1.3_Linux.tgz`
	- `cd Vimba_2_1/VimbaGigETL/`
	- `sudo ./install.sh`

## 1. Install
0. `cd ~/catkin_ws/src`
1. `git clone https://github.com/hdh7485/avt_vimba_camera`
2. `cd ../..`
3. `source devel/setup.bash`
4. `rosdep install -ary`
5. `catkin_make`

## 2. Edit launch file
`rosed avt_vimba_camera mono_camera.launch`
change ip address for your camera and save.

## 3. Test
`roslaunch avt_vimba_camera mono_camera.launch`
