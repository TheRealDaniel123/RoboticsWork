The robot navigates the maze via its sensors which detect its distance from the walls. Colours are also detected by the robot. The red colour informs the robot to turn around and the blue colour tells the robot to move forwards (indicating that its on the right track). Finally, the green colour informs the robot to move forwards where it subsequently solves the maze.  

To run the program, first ensure the software is installed with the command "sudo apt update && sudo apt upgrade && sudo apt install ros-melodic-uol-cmp3103m".
To open the maze run the command "roslaunch uol_turtlebot_simulator maze1.launch".
Then navigate to the directory where the script is installed and call "python followLinev12.py" to run the program.