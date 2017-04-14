[//]: # (Image References)
[image001]:./resource/doc/example.png  "example conversion"
[image002]:./resource/doc/play-bag.png  "play bag"

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

#### Update 07.04 add package didi-playbag
to visualize the output of bag file do following
```
roslaunch didi-playbag play_bag_all.launch
```

then open a new terminal play bag file 
```
rosbag play --pause 17.bag --clock
```

You should see some similar in rviz

![image002]




### Vehicle classifier
How to use saved model.
Notes:
1. The model only accepts (64,64,3) so will have to do a resize function
2. The model was trained on inputs with value between 0 - 1 rather than 0 - 255 so make sure inputs are in this value range
see [a relative car_id_nn.ipynb](car_id_nn.ipynb) for more details
```
from car_id_model import CarIDModel
carIdModel = CarIDModel(loadweights=True)
prediction = carIdModel.predictSingle(image)
```

training was done using Udacity's P5 Vehicle detection datasets.
