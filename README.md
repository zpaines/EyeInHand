Getting Started
=====

### Dependencies
To use any of this code you are going to need the following:
  1. [cisstICP by Seth Billings](https://git.lcsr.jhu.edu/sbillin3/cissticp)
  2. [The cisst Libary](https://github.com/jhu-cisst/cisst)
  2. [Point Cloud Library](http://pointclouds.org/downloads/)
  3. [Intel RS SDK](https://software.intel.com/en-us/intel-realsense-sdk)

### Hardware
This program currently only supports the Intel RealSense camera. Plug the camera into your computer's USB port to use any camera-based features.

### Running the Code
To run the program, simply build and run the main.cpp file. A text-based menu should pop up, and you're ready to go!

### Documentation
For further information on the structure of the code, please see [our documentation](http://zpaines.github.io/EyeInHand/annotated.html).


Information Flow
==============

![](images/design_flow.png?raw=true)

Menu Options
========

### Point Cloud from Camera
Use the menu option Take Point Cloud from Camera to use the input camera to capture a depth image and turn the image into a PCL Point Cloud. This will then load this point cloud as the current working point cloud.

### Point Cloud from File
To read in a point cloud from a file, select Read Point Cloud from File and enter the path to a .pcd file. This will load the point cloud as the current working point cloud.

### Mesh from File
Select Load Mesh File for Registration and enter the path to a cisst mesh file. There is no particular file extension necessary, but the contents of the file should be as follows:

`POINTS #points
...
VERTICES #vertices
...
NORMALS #normals
...`

### Perform Registration
To perform a registration, first ensure that you have both a point cloud and a mesh file loaded. Then, select Register Point Cloud to Mesh. This will run the ICP algorithm and also apply the obtained frame transformation to the current working point cloud; however, it will write this tranformed point cloud to a separate cloud and will NOT overwrite the current working point cloud.

### Test Calibration
To test the AX=XB calibration algorithm, select Test AX = XB Calibration.

### Compare Clouds
To visualize both the original point cloud and the transformed point cloud, perform a registration, and then select Visualize Point Cloud and Transformed Point Cloud. This will open up a visualizer with both clouds loaded.

### Save Point Cloud
To save the current working point cloud, select Write Point Cloud to File, and enter your desired path and filename to save the cloud as a PCD file.

### Save Transformed Point Cloud
To save a transformed point cloud, perform a registration, then select Write Transformed Point Cloud to File. Enter your desired path and filename to save the transformed cloud as a PCD.

### Save Frame Transformation
To save a frame transformation, perform a registration, and then select Save F\_reg to File. Enter your desired path and filename to save the frame transformation output by the registration.

### Downsampling Clouds
To downsample the currently loaded cloud, select Downsample Cloud. This will prompt you for a leaf size for your KD Tree. Once you enter a number, the program will inform you by how many points your current cloud has been simplified.

### Plane Segmentation
To remove the background plane of a given point cloud, select Remove Plane. This will remove the largest plane in your current working point cloud (which is usally a surface behind your desired point cloud), and overwrite the new point cloud as the current working point cloud.

### Quit
To exit the program, select Quit.
