# MTRX3760 Project 2: Where's the beef?

## Project Cloning Instructions

1. Navigate to `cd ~/turtlebot3_ws/src/turtlebot3_simulations`. Note your install location may differ.
2. Delete `turtlebot3_gazebo` directory
3. Use `git clone git@github.com:TheFlamingBadger/mtrx3760-project-1.git turtlebot3_gazebo`

## Project Running Instructions

1. Rebuild workspace: `cd ~/turtlebot3_ws && colcon build --symlink-install`
2. Select TurtleBot model: `export TURTLEBOT3_MODEL=burger`
3. Launch Gazebo (default world): `ros2 launch turtlebot3_gazebo empty_world.launch.py`
4. On left hand side of Gazebo interface select `insert` -> `add path` then add the `maze_models` directory under
5. Select maze from left hand side and place with robot in blue-marked area
6. Run image processing node: `ros2 run turtlebot3_gazebo CImageProcessor`
7. Run drive logic node: `ros2 run turtlebot3_gazebo CDriveLogic`

## Links

- [Software Install List](https://docs.google.com/spreadsheets/d/1X3_AnZvCP2jTWK1Q7EVujKh0q1d0yrmBEny1rSNteX4/edit?gid=0#gid=0)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Gazebo Simulation Tutorial](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)

## Command Reminders

### Git CLI Workflows

Update code:

1. `cd ~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/`
2. `git pull`

Push changes to GitHub:

1. Go to local repoistory: `cd ~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/`
2. Ensure you have latest version: `git pull`
   - **Note:** Conflicts resolved here, if needed.
3. Check changed files are appropriate: `git status`
4. Add relevant files for the commit `git add <file_1> <file_2> ...` or `git add .` to add all
   - **Note**: Commits should be logically atomic (e.g. "Refactored movement logic and fixed header file structure" should be two separate commits)
5. Create a commit with `git commit -m "2024-Sep-<DD> - <your-name> - <commit-message>”`ot
6. Push commit to remote (GitHub) main branch: `git push origin main`

### Run TurtleBot3 Drive Node in Gazebo

1. Rebuild workspace: `cd ~/turtlebot3_ws && colcon build --symlink-install`
2. Select TurtleBot model: `export TURTLEBOT3_MODEL=burger`
3. Launch Gazebo (default world): `ros2 launch turtlebot3_gazebo empty_world.launch.py`
4. Run turtlebot3_drive node: `ros2 run turtlebot3_gazebo turtlebot3_drive`
5. Run turtlebot3_image_processing node: `ros2 run turtlebot3_gazebo turtlebot3_image_processing`

### Set Up Automatic Sourcing

Adds commands to system shell startup script:

1. `echo "source install/setup.sh" >> ~/.bashrc`
2. `echo "source /opt/ros/humble/setup.sh" >> ~/.bashrc`
3. `echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc`

### Manual Source Commands

Required to be run every terminal session:

`source install/setup.sh && source /opt/ros/humble/setup.sh && source /usr/share/gazebo/setup.sh`

### Running code on bot

1. Open terminal for robot (run bringup and other tests)
2. Open another terminal to confirm communication using ssh command
3. Open terminal for computer and then colcon build respective files (DON'T DO THIS IN ROBOT TERMINALLLLL!!!!!)
4. Bash and run the file

### Resources
1. April/QR codes manual: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
2. April/QR codes GitHub: https://github.com/PlusToolkit/aruco
