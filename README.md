# Path Planning

<a href="https://imgflip.com/gif/20lv1c"><img src="https://i.imgflip.com/20lv1c.gif" title="made at imgflip.com"/></a><a href="https://imgflip.com/gif/20lw16"><img src="https://i.imgflip.com/20lw16.gif" title="made at imgflip.com"/></a>
<a href="https://imgflip.com/gif/20ly7i"><img src="https://i.imgflip.com/20ly7i.gif" title="made at imgflip.com"/></a><a href="https://imgflip.com/gif/20lyie"><img src="https://i.imgflip.com/20lyie.gif" title="made at imgflip.com"/></a>

## Overview
The goal of this project is to build a path planner that creates smooth, safe trajectories for the car to follow. The simulated highway track has other vehicles driving +-10 MPH of the 50MPH speed limit. The car should try to go as close to the 50 MPH speed limit as possible, which means passing slower traffic when appropriate (other cars will try to change lanes too). The simulator transmits the car's localization and sensor fusion data to be used by the path planner. 

The path planner outputs a list of x and y global map coordinates. Each pair of x and y coordinates is a point, and all of the points togther form a trajectory. Every 20ms the car moves to the next point on the list. The car's new rotation becomes the line between the previous waypoint and the car's new location. The velocity of the car depends on the spacing of the points.

The challenge of this project is creating paths that can safely and smoothly navigate through highway traffic, changing lanes and passing cars when appropriate. For this project, a "smooth" ride is considered one in which the jerk does not exceed 10m/s^3 and total acceleration (including centripital acceleration from turning) does not exceed 10m/s^2. 

#### Highway Map

data/highmap.csv contains a list of waypoints that go all the way around the track. The track contains a total of 181 waypoints, with the last waypoint mapping back to the first. The waypoints are in the middle of the double-yellow dividing line in the center of the highway. The highway has 6 lanes total - 3 in each direction. Each lane is 4m wide.

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Path Planner

//Note: The simulator returns the previous trajectory point list but with processed points removed

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and that car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

#### Simulator Details

1. The simlutaed car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. 

2. There is some latency between the simulator running and the path planner returning a path (1-3 time steps). During this delay the simulator will continue using points that it was last given. previous_path_x, and previous_path_y are helpful for smoothing the transition


## Path Planner Model

The path planner can be broken down into 2 major steps:
1. Choosing the optimal lane using a cost function and safety restrictions 
2. Generating a trajectory based on the chosen lane and previous path points

### Choosing the lane
Calculate the cost of being in each lane by evaluating the sensor fusion data about other cars on the road. This model uses 4 different costs:
1. If the car is in another lane and is close ahead or close behind, register an overwhelming cost because it is not safe to go into this lane. 
2. If the car is ahead of us (in any lane), register a cost proportional to the difference between the car's speed and our target speed (49.5 mph). The slower the car is going relative to our target speed, the more cost it will accrue
3. Register a cost proportional to the distance the car is from our car. The closer the car is to us, the higher cost it will generate.
4. Register a small cost for any lane that is not our current lane. We want to stay in our lane unless one of the above costs compells us to change

The lane with the least cost becomes our "desired lane". If our desired lane is next to our current lane, we simply set our "chosen lane" to be the desired lane. If our desired lane is more than 1 lane away (i.e. current lane = 0 and desired lane = 2) then we check whether it is safe to move through the middle lane. If so, we set the "chosen lane" to the middle lane. If not, we stay in our lane. 

There is one more important step when evaluating the sensor fusion data. If there is a car close in-front of us in our lane, then we changed our target velocity to match the velocity of this car. This car then effectively acts as a "lead car" until we switch lanes. 

### Creating a trajectory
Start by adding the last 2 "previous path" points into an "anchor" list. These two points act as anchors for the start of the new trajectory. Then add 3 more "anchor points" that are spaced 40, 60 and 90 meters and which are centered in the "chosen lane". These 5 points make up the "anchor points" for the new trajectory. Then, using this [spline tool](http://kluge.in-chemnitz.de/opensource/spline/), fit a spline to these points, and because we are fitting a spline to points that are spaced out 40, 60, and 90 meters ahead, the spline generates a smooth trajectory even when this new trajectory is enacting a lane change. This spacing allows the car to meet jerk requirements. We then can then "fill" this new trajectory with points along the spline. These "fill points" are spaced to match our target velocity. While generating these fill points, the velocity is incremented/decremented to match target velocity that was set while evaluating the sensor fusion data. This newly created trajectory is appended onto the previous path trajectory that was not executed by the simulator. 



## Dependencies
* Udacity Simulator. Download the Term3 Simulator which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Running the code
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.









