# ECE-565-Autonomous Robotics Lab Final Competition
- NOTE:  All exploits were done with permission, and within a sandboxed competition environment.
The file "main.c" contains the code that runs on the robot, while the submodule folder "wifi-disable" contains the code that runs on a laptop in the same room.
# Theory of Operation:
- The code on the robot:
  - Establishes link to laptop
  - Waits for user to press "start" on the robot
  - Sends signal via wifi for laptop to begin sending shutdown commands to enemy robot
  - Uses computer vision and odometry to identify and retrieve block, and bring block back to base
- The code on the laptop:
  - Waits for robot to send start signal
  - Loops through enemy robot wifi credentials, sending POST request to shutdown endpoint to all know robots
# Nature of the exploit:
Each robot contains a raspberry pi 2b+ with a wifi AP that students connect to via SSID and password.  Once connected, students can access the web IDE in a browser.  The web IDE contains a "shutdown" button.  When pressed in the browser, this sends a POST request to the robot, and the raspberry pi processes this request by shutting down.  Once the enemy robot is shut down, our robot will win by scoring at least a single point.  Because we cannot know who our opponent is before the match, we needed to shut down all possible robots with our exploit.  To get a list of SSIDs and passwords, we collected WPA2 handshake data while students were programming their robots in the lab room prior to the competition.  Because the OS code running on the robots is open source, we know how the AP password is set.  It is of the form XXXXXX00, where X is lowercase hexadecimal (0-f) and the two zeros are hard-coded.  This meant that there are only 16^6 possible passwords, taking seconds to crack on 3070 Ti.  Once we had the list of passwords and SSIDs, the laptop would iterate this list when the competition started, sending the shutdown command to each robot.  This exploit won us the competition, and allowed us to have a physically much simpler robot (we only needed to earn 1 point, because the enemy was guaranteed to get 0).
