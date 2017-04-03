[//]: # (Image References)
[image001]:./resource/doc/example.png  "example conversion"
 
# didi-challenge

## Kitti database labels
resource/data_object_label_2.zip 

This is the label file downloaded from the kitti benchmark. Please first extract the training dataset in the resource directory and then use the ipython notebook to read the labels

### ROS packages
rodsws -> ros workspace

#### velodye2image 
convert velodye point cloud to a 2D height image

* compile:
```
cd rosws
catkin_make
```
* run:
```
source devel/setup.bash
rosrun velodye2image velodye2image_node
```
After running this, a node is spawned which listen on the topic '/kitti/velo/pointcloud'

use rosbag to publish point cloud, you will find image data are generated.

Example image:
![image001]

