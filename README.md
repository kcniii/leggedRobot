# Legged Robot

This file provides step-by-step instructions on setting up and running the exercise.

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