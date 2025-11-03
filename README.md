This is a joint project between NYP and SUTD. My task is to study autonomous driving principles and integrate them into a RC car. I will be uploading my codes into the workspace titled "sutd_ws"

The workspace is made up of a few different algorithms i tested one by one. In the end, I went with the final_code.py that combines both the remote control and autonomous codes, using a Logitech controller. 

The wall follower code relies on PID values to correct the error offsets to 0, using the distance from both walls. The car will then steer along the center line as much as possible. 

The gap follower code scans the environment and filters out what it senses as obstacles. It will then score the gaps and steers towards the best gap. 

The autonomous driving code combines both the wall following code and the gap following code, and alternates between the two depending on the situation. 

The remote control code is just for me to test out the speed and steering. 

The final code combines both the remote control and the gap following algorithm. I eventually decided to go with gap following as it seems to be the best method for autonomous driving. 
