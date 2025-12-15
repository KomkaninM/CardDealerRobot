# CardDealerRobot


## Table of Contents
- [CardDealerRobot](#carddealerrobot)
  - [Table of Contents](#table-of-contents)
  - [Turtlebot3 Drive Setup](#turtlebot3-drive-setup)
  - [Github Pull \& Update](#github-pull--update)


## Turtlebot3 Drive Setup

This step configures and verifies that your Turtlebot3 is connected and working properly.

1. **Check which port your LiDAR is connected to.**  
   It should appear as `/dev/ttyACM0` or `/dev/ttyACM1`.

    ```bash
    ls /dev/ttyACM* 
    ```

 2. **Set the robot model and launch:**

 ```bash
 export TURTLEBOT3_MODEL=burger
 sudo chmod 666 /dev/ttyACM1
 ros2 launch turtlebot3_bringup robot.launch.py
 ```

3. **If you see errors or the motors don’t move, click RESET on OpenCR and edit usb_port:**

 ```bash
   nano ~/turtlebot3_ws/src/turtlebot3/turtlebot3_bringup/launch/robot.launch.py 
 ```

> ✅ **Check:** When successful, you should hear a startup sound and see log messages like  
> `[turtlebot3_ros-3] [INFO] [diff_drive_controller]: Init Odometry`
> `[turtlebot3_ros-3] [INFO] [diff_drive_controller]: Run!`

4. **For Testing with Teleop:**
```bash
  export TURTLEBOT3_MODEL=burger
  ros2 run turtlebot3_teleop teleop_keyboard
```
## How to Build
1. **Install Source Code**
```bash
  cd <ROS2_workspace>/src
  git clone https://github.com/KomkaninM/CardDealerRobot.git
  ros2 run turtlebot3_teleop teleop_keyboard
```
2. Build
```bash
  cd <ROS2_workspace>
  colcon build --symlink-install
  source install/setup.bash
```

## Github Pull & Update

1. **Create the Repo temporary Folder**
 ```bash
    cd ~/github_upload/CardDealerRobot
    git pull
 ```
2. **copy your code to Git Folder**
```bash
    cp -r /home/ubuntu-me-2/<your_workspace>/src/* src/
 ```
3. **Stage, Commit and Push**
```bash
    git add .
    git commit -m "<Your Commit Message>"
    git push origin main
 ```
 4. **When it asks for Username**
See the github access token by
```bash
    cat github_keys.txt
 ```
When it asks for your Username, enter GitHub username : KomkaninM. When it asks for your Password, paste the Personal Access Token in github_keys.txt file

