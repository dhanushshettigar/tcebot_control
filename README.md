# tcebot_control

TCEbot Control is a ROS 2 package designed to control a differential-drive robot using commands from `cmd_vel`. The package bridges communication between ROS 2 and an Arduino-based motor controller via serial communication.

## Installation

1. Clone the repository

```bash
git clone https://github.com/dhanushshettigar/tcebot_control.git
cd tcebot_control
```

2. Install Dependencies

```bash

sudo apt update && sudo apt install ros-jazzy-teleop-twist-keyboard

```

3. Build the Package

```bash
colcon build --packages-select tcebot_control
source install/setup.bash
```
## Running the System
1. Run Custom Teleop Keyboard
This script allows manual control of the robot with adjustable speed.
```sh
ros2 run tcebot_control teleop_keyboard
```

**Controls**

```bash
Control Your Robot:
---------------------------
Move:    
    W    
  A   D  
    S    

Stop:     X  
Speed Up: +  
Slow Down: -  
Exit:     Q  
```

2. Start the Motor Control Node
```sh
ros2 launch tcebot_control tcebot_control.launch.py
```

## Troubleshooting
- If SSH fails, connect a monitor and keyboard to troubleshoot.
- Ensure all dependencies are installed:
  ```bash
  sudo apt install python3-rpi.gpio
  ```
- Enable GPIO Access Without Root
  ```bash
  sudo groupadd gpio
  sudo usermod -aG gpio tcebot
  sudo chown root:gpio /dev/gpiomem
  sudo chmod g+rw /dev/gpiomem
  sudo reboot
  ```
- Try - `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
