## Project Follow Me
This project is for course of Robotic Perception and Action.
 - In CHEATSHEET you will find troubleshooting for some encountered issues
 - In AUTHORS you will find a replica of the contributors written below
 - CMakeLists.txt is an example of how should be the final compiler instruction set

project authors:
Andrea Carollo andrea.carollo@studenti.unitn.it
Giovanni Ferrara giovanni.ferrara@studenti.unitn.it
Gianni Lunardi gianni.lunardi@studenti.unitn.it
Matteo Tomasi matteo.tomasi-1@studenti.unitn.it

## Path Planning - DEMO
The path planning demo uses a Realsense example .bag file, that can be in https://github.com/IntelRealSense/librealsense/blob/master/doc/sample-data.md ( Outdoor scene with D435i pre-production sample (Depth from Stereo with IMU) ). Download this file and put it in the main folder of the demo. Compile the project inside a "build" folder and execute Demo_PP. 
The code will show three window:
  - The RGB frame with a red marker in the "target point" (2D)
  - The point cloud viewer with the original and the transformed reference system and a red marker in the "target point" (2D)
  - The build human interface

To stop the code, quit the point cloud viewer. At the end of the execution, the mean computation time for each part of the code is printed.
