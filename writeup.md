# Path Planning Project

[//]: # (Image References)

[imagetest1]: ./output_images/standard_1.jpg "Standard Driving"
[imagetest2]: ./output_images/standard_2.jpg "Standard Driving"
[imagetest3]: ./output_images/extreme_1.jpg "Extreme Driving"
[imagetest4]: ./output_images/extreme_2.jpg "Extreme Driving"


## Goals

The goals of this project are the following:

* The car is able to drive at least 4.32 miles without incident.
* The car drives according to the speed limit.
* Max Acceleration and Jerk are not Exceeded.
* Car does not have collisions.
* The car stays in its lane, except for the time between changing lanes.
* The car is able to change lanes.


## Trajectory Generation

The trajectory generation uses points from previous iterations of the path planner (where available), along with new points based on the target lane derived from any lane changing requests from the finite state machine as shown below.  This is then used with the [spline tool](http://kluge.in-chemnitz.de/opensource/spline/) as discussed in the Project Walkthrough to generate points for the vehicle to follow.


## Cost Function

The program includes the following items in the cost function:

| Item        | Description                                                                                                                |
| ----------- | -------------------------------------------------------------------------------------------------------------------------- |
| Distance    | The distance to the closest vehicle in front of the driven vehicle.                                                        |
| Speed       | The difference in speed between the closest vehicle in front of the driven vehicle and the driven vehicle.                 |
| Collision   | A true/false indicator for determining if a move into this lane will result in a collision.                                |
| Lane Change | A small cost associated with changing from the current lane to prevent changes with little benefit.                        |
| Centre Lane | A bias towards driving in the centre lane.  This is useful as there are two states from here rather than one on the sides. |


## Finite State Machine

The program includes a finite state machine with the following states:

| State | Description                                                       |
| ----- | ----------------------------------------------------------------- |
| "KL"  | *Keep Lane* where the cost is lowest to stay in the current lane. |
| "LCL" | *Lane Change Left* indicates the vehicle is changing lane.        |
| "LCR" | *Lane Change Right* indicates the vehicle is changing lane.       |


## Results

The solution meets the required goals, as shown in the Output section below.


#### Output

The final output includes projecting all of the information obtained onto a single image.

| Standard Driving        | Standard Driving        |
| ----------------------- | ----------------------- |
| ![alt text][imagetest1] | ![alt text][imagetest2] |

The following images show the value of `EXTREME` set during compilation.  This will result in the vehicle deliberately exceeding the speed and acceleration limits, however is useful in testing the overall system functionality.  The number of collisions with this setting is minimal.

| Extreme Driving         | Extreme Driving         |
| ----------------------- | ----------------------- |
| ![alt text][imagetest3] | ![alt text][imagetest4] |


## Discussion

The currrent solution could be improved by the following additions:

* Detect vehicles changing lanes into the target lane.  This type of side impact is a known issue with the currrent implementation.
* Currrently only the lane to the sides of the current lane are examined.  This is not optimal, as it may be worthwhile crossing from one side of the road to the other for best performance, however this is not currently performed.  The partial solution to this problem is the way the Cost function prefers the Centre Lane if all else is relatively equal.
