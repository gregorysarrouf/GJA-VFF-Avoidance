# VFF Avoidance Navigation Algorithm

Authors:
Gregory Sarrouf,
Jacques Dergham,
Alexa Fahel

## Running The Project
Add the project into the ros2_ws/src directory, and use colcon to
build the project (at the root of the ros2_ws directory):
```
colcon build --packages-select vff_avoidance
```

Open a new terminal and run the project:
```
ros2 launch vff_avoidance avoidance_vff.launch.py
```

This should open gazebo and automatically start the ros2 avoidance node
that will navigate the robot.
