# imageconverter
Light Interception
ROS cpp codes to extract light interception from RGB image topic
Contains 3 nodes:
1. Sync - Extracts and syncs three topics from rosbag: camera info, image, gps
2. Imageconverter-Rectifies image and finds the light interception 
3.Logger- Logs the light interception and gps tag into a csv file


Running the launch file loads all three nodes
