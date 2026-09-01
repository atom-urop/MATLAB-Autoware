** This folder is created before changing the control module.
We have recorded two ros2 bags: 2ws and 4WS.
For the first bag 2ws: We have launched the simple planning simulator and
we did not sent any message to the rear steer input of the vehicle model
(the lateral control module still do not send anything to the rear (we
have not modified it yet
For the second bag 4WS: We have launched the simulation again, but now at
two arbitrary points while the car is moving we set at one the rear steer
to be 0.3 and then at another point to be -0.3. This is done as a sort of
checking if the vehicle model can hold rear steer angle and how it will
react
