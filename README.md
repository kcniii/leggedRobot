# Legged Robot

This file provides step-by-step instructions on setting up and running the Talos robot.

# Walking

1. Open the terminal in this src folder(e.g. ~/src). Install venv, pydrake and neccessay dependencies.

    ```bash
    source /opt/ros/noetic/setup.bash
    python3 -m venv env
    env/bin/pip install --upgrade pip
    env/bin/pip install drake
    source env/bin/activate
    pip install rospkg
    pip install pybullet
    sudo apt install robotpkg-py3*-ndcurves -y
    ```

2. Make sure you are in the venv and run walking.py
    ```bash
    ./env/bin/python walking.py
    ```
![](https://github.com/kcniii/leggedRobot/blob/main/GIF/walking.gif)

## Prerequisites

Before running the code, make sure you have the following dependencies installed:

1. [Pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html)

2. PyBullet

   ```shell
   pip install pybullet
   ```

3. TSID
    ```shell
    sudo apt install robotpkg-py3*-tsid
    ```

## build
To build the project, follow these steps:
 Navigate to the workspace.Run the following commands:
```shell
catkin_make
source ./devel/setup.bash
```

## launch
### Standing


```bash
rosrun bullet_sims t4_01_standing.py
```
![](https://github.com/kcniii/leggedRobot/blob/main/GIF/stand.gif)

### One Leg Stand

```bash
rosrun bullet_sims t4_02_one_leg_stand.py
```
![](https://github.com/kcniii/leggedRobot/blob/main/GIF/one-leg.gif)
### Squatting

```bash
rosrun bullet_sims t4_03_squating.py
```
![](https://github.com/kcniii/leggedRobot/blob/main/GIF/squat.gif)