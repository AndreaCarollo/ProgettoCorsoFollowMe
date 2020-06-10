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

## Detection - DEMO
The detection demo shown the identified people in the frame using the 3 detector used in the main code.
Is uses 3 pre-trained haar-cascade method: Fullbody, Pedestrian, Uperbody.
The people found are highlited with a rectangle. On the frame will be shown the legenda of the used colours.
Compile the project in a "build" folder inside the demo one.
A video that can be used for the demo is provide at the following drive folder: https://drive.google.com/drive/folders/1KhEnGBXV-ncVonFk5p3cmrCbCsjQD0Uu?usp=sharing.
To run the code execute Demo_dect with the path of the video: $ ./Demo_dect <path_of_video>

Note: to use completely the OpenCV functions, you need also the OpenCV-contrib packages

## Target Identification - DEMO
The target identification demo uses one of the two video contained in the drive folder https://drive.google.com/drive/folders/1KhEnGBXV-ncVonFk5p3cmrCbCsjQD0Uu?usp=sharing. Compile the project in a "build" folder inside the demo one. Then execute Demo_TI, specifying as second argument "(path_to_the_video_folder)/video_name.mp4".
The code displays a video in which the marker on the target (Matteo) is detected.

## Path Planning - DEMO
The path planning demo uses a Realsense example .bag file, that can be found at the link https://github.com/IntelRealSense/librealsense/blob/master/doc/sample-data.md - named as "Outdoor scene with D435i pre-production sample (Depth from Stereo with IMU)" -.
The path planning demo videos can be found in the drive folder at https://drive.google.com/open?id=1KAc75xrD2diaJRUN_Te4FWHkSmN9lR7I  Download this file and put it in the main folder of the demo. Compile the project inside a "build" folder and execute Demo_PP. 
The code will show three window:
  - The RGB frame with a red marker in the "target point" (2D)
  - The point cloud viewer with the original and the transformed reference system and a red marker in the "target point" (2D)
  - The build human interface

To stop the code, quit the point cloud viewer. At the end of the execution, the mean computation time for each part of the code is printed.

## Gesture Recognition - DEMO
The gesture recognition demo uses a video, which can be find inside the folder at the link https://drive.google.com/drive/folders/1KhEnGBXV-ncVonFk5p3cmrCbCsjQD0Uu?usp=sharing. Once the video has been downloaded, put it in the main folder of the demo. Create a "build" folder inside the demo one, then compile the project inside "build" and execute Demo_GR. 
The code shows three windows:
  - The RGB frame
  - The region of interest for the hand
  - The binary mask of the ROI
