# wall-a-maze
2016 KTH Robotics and autonomous systems winner

To run it:
```
catkin_make install
roslaunch wheels onboard.launch
roslaunch classifier classifier.launch
```
Then depending on contest stage
```
rosrun motherbrain mother.py explore
# or
rosrun motherbrain mother.py score
```
