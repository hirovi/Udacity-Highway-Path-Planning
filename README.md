# Path-Planning-Udacity
This is my solution to the Path Planning Project of the Udacity Self Driving Nanodegree.

The goal is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. This implementation succesfully keeps the vehicle inside its lane, avoids hitting other cars and changes lane if required using localization, sensor fusion and map data.

## About
There are many approaches that can be used to generate paths.
In order to succesfully generate a path, smoothness and effectiveness has to be taken into account.
From the current vehicle position 

At the end of the day the objective of a path planner is to bring you from point A to point B successfuly. Therefore, this path needs to define a smooth transition and decide which xy points are the most suitable at t+1. 
In this approach the program generates 50 points, which the car will individually follow every 20ms.
To do so a spline was used. The spline was created with 5 points, in which the first two points corresponded to the last two points of the previous path, and then 3 more points evenely spaced by 30 m when the car had to follow same lane. When a lane changing had to be made these last three points were further spaced to achieved higher smoothness in the path.
This implementation currently chooses to change lane when the vehicle can not achieve the set target speed. The vehicle using the sensor fusion output, checks which lane is the most effective to change so that the target speed is achieved.To do so, distances between the car and the front vehicles in the different lanes are compared. Aswell as the distances between the vehicle and rear cars.
