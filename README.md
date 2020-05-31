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

## Target Identification - DEMO
The target identification demo uses one of the two video contained in the drive folder https://drive.google.com/drive/folders/1WtdYyrYelBe7RNGn29iIWYpMJgJqAd4f?usp=sharing. Compile the project in a "build" folder inside the demo one. Then execute Demo_TI, specifying as second argument "(path_to_the_video_folder)/video_name.mp4".
The code displays a video in which the marker on the target (Matteo) is detected.

## Path Planning - DEMO
The path planning demo uses a Realsense example .bag file, that can be find at the link https://github.com/IntelRealSense/librealsense/blob/master/doc/sample-data.md - named as "Outdoor scene with D435i pre-production sample (Depth from Stereo with IMU)" -. Download this file and put it in the main folder of the demo. Compile the project inside a "build" folder and execute Demo_PP. 
The code will show three window:
  - The RGB frame with a red marker in the "target point" (2D)
  - The point cloud viewer with the original and the transformed reference system and a red marker in the "target point" (2D)
  - The build human interface

To stop the code, quit the point cloud viewer. At the end of the execution, the mean computation time for each part of the code is printed.

## Gesture Recognition - DEMO
The gesture recognition demo uses a video, which can be find inside the folder at the link https://drive.google.com/drive/folders/1TqXJpjsKeTqzXTB9uVQv9fMZvPE-AGP5?usp=sharing. Once the video has been downloaded, put it in the main folder of the demo. Create a "build" folder inside the demo one, then compile the project inside "build" and execute Demo_GR. 
The code shows three windows:
  - The RGB frame
  - The region of interest for the hand
  - The binary mask of the ROI
