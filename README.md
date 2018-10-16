# ares_slam
ROS package for 2D real-time simultaneous localization and mapping.

## Requirements
* eigen
* g2o

## Example
An example map build by sick lidar. 

<img src="map/map.pgm" height="512pix" /> 

## Build
  ```shell 
  cd ${catkin_workspace}/src
  git clone https://github.com/ningwang1028/ares_slam.git
  cd ..
  catkin_make
  ```
## Run 
  ```shell  
  roslaunch ares_slam ares_slam.launch  
  ```
