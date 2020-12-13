# AnyNet_ROS
This is ros realization of [AnyNet model](https://github.com/mileyan/AnyNet).

# Installation 
```
cd docker
sh build.sh
sh start.sh
sh into.sh
cd catkin_ws
catkin_make
source devel/setup.bash
cd src/anynet_ros/scripts
sudo sh compile_spn.sh
cd $home
```

# Launch
```
roslaunch anynet_ros anynet.launch --screen \
    left_images:=/left/image_raw \
    right_images:=/right/image_raw \
    output_topic:=/anynet_disparities \
    input_w:=1200 input_h:=576
```

Note that images will be resized to `(input_w, input_h)`. These parameters should be divisible by 16 due to model's architecture.
