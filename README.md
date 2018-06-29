# Path-Planning-Udacity
This is my solution to the Path Planning Project of the Udacity Self Driving Nanodegree.

The goal is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. This implementation succesfully keeps the vehicle inside its lane, avoids hitting other cars and changes lane if required using localization, sensor fusion and map data.

## About
At the end of the day the objective of a path planner is to bring you from point A to point B successfuly. Therefore, the planner needs to define which xy points are the most suitable at each timestamp.
We obviously don't want a path that goes in a straight line jumping from one point to the other making the trip the most unpleasent ever. Instead, it is much prefered a smooth transition that brings you to your goal as comfortable as possible, for this reason, a spline was used. 

Every 20ms the simulator reaches the program asking "to what point should I go next?", a total of 50 points are constantly updated according to the spline. THis spline has been predefined with some points from the previous path and some other future points spaced evenly 30m away, creating a total of 5 points to generate the spline.
The vehicle choses either to continue in the same lane or to change according to the feasibility of reaching its target speed of 50mph. When the vehicle detects that the front car is too close either slows down or checks whether changing lane is possible. The legality of changing road is determined according to the postion of the other vehicles in their respective lanes, when legal to change lane, the car create a new spline that allows to smootlhy change of lane(this is accomplished by spacing-out more the points).

## Improvements
Would be super cool to create a neural net where you feed the sensor fusion data and it outputs the position at each timestamp. Maybe a RNN would be more ideal...

## Result
The car is able to drive at least 4.32 miles without any incident. Driving according to the speed limit without braking the max acceleration and jerk limits.The car doesn't collide and it's able to succesfully change lanes when required.

## Build & Run Instructions
1. Clone repo
2. Make a build directory: ```mkdir build && cd build```
3. Compile: ```cmake .. && make```
4. Run it: ```./path_planning```




