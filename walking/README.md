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
    ```

2. Make sure you are in the venv and run walking.py
    ```bash
    ./env/bin/python walking.py
    ```
