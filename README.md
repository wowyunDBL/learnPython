/* context */
1. publish image 
2. publish point cloud
3. show car module and the range of camera
4. use detection results to plot 2D box in the picture
5. use detection retults to plot 3D box in the point cloud


/* convert datatype */
1. use cv2 and bridge to convert .png to sensor_msgs/Image
2. use pcl2 to convert .bin to sensor/PointCloud



/* technique */
1. use os.path.join to read file
2. use ROS visualization marker of mesh resource to display car
3. use pandas to deal with data
4. plot figure


/* issue */
1. from (module) import (class or function) ?

/* note */
1. tf::Quaternion q(yaw, pitch, roll)  v.s  q = tf.transformations.quaternion_from_euler(i,j,k)

