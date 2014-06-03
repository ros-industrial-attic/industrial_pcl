## industrial_pcl_utilities

Provides nodes that allowed publishing sensor_msgs/PointCloud2 messages from either a .pcd or description .yaml files

#### pcl_msg_generator_node

- Generates a point cloud constructed with the parameters provided in the point_cloud_description.yaml file
- Run the pcl_msg_generate.launch file for an example
```
roslaunch industrial_pcl_utilities pcl_msg_generate.launch
```

#### pcd_to_msg_node

- Reads a pcd file and publishes the data as a sensor_msgs/PointCloud2 message
- Takes optional arguments that modify certain properties of the point cloud:
  
  * -f : path to pcd file
  * -r : publishing rate (optional)
  * -n : noise (meters) (optional)
  * -i : frame id (optional)
  * -h : help description
  
- It also looks for the optional "~/cloud_transform" private parameter for position and orientation information.  The
node will  apply the transform to all the points in the file.  Also, these parameters can be changed while the node is running
and the point cloud will be updated accordingly.
- For instance, changing the parameter /pcd_publisher_node/cloud_transform/x in the parameter server from 0.5 to 0.89 will
move the point cloud 0.39 meters along the x axis relative to its own frame (the frame set with the -i argument).
-  Run the pcd_to_msg.launch file for an example
```
roslaunch industrial_pcl_utilities pcd_to_msg.launch
```
