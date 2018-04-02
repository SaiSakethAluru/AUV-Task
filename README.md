# AUV-Task

color. cpp - Code for detecting the red bouy's centre in the video.

Kalman.cpp - Code for tracking the red bouy's centre using Kalman filter in the video. 


## To Run the code:

Set the path of the video to appropriate value in the parameter of the VideoCapture function. 
Compile the file and execute it. 

In case of color.cpp, the detected bouy is indicated by the red circle.

In case of Kalman.cpp, the detected bouy's current location, predicted location and observed location are printed in crosses of various colours. 

White - current point

Red - Corrected point

Yello - Predicted point
