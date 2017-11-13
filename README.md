# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Basic Build Instructions

    1. Clone this repo.
    2. Make a build directory: `mkdir build && cd build`
    3. Compile: `cmake .. && make`
    4. Run it: `./path_planning`.


## Path Planner Implementation -

The high level implementation and the different state machines used is described in the flowchart below

![Alt text](flowchart.png?raw=true "Finite State Machine Representation")

There are 4 main states used in the behavior planner.
1. Stay in lane at constant velocity of ref_vel - This is the ideal case when the velocity is equal to ref_vel ( default = 47.5 ) and there is no car ahead in the same lane for atleast 40m.

2. Change lane to left lane - If there is a car ahead in the same lane, evaluate whether a lane change to the left lane is possible. If yes, then make the lane change.

3. Chane lane to right lane - If there is a car ahead in the same lane and a left lane is current not possible, then evaluate the feasibility of making a lane change to the right lane. If yes, then make the lane change.

4. Stay in Lane and Slow Down - It lane changes are not possible given that there is a car ahead, slow down with a deceleration based on the distance between the car ahead and start again at State 1.

To illustrate  the implementaton of the different states, the implementation code for the Change Right lane is shown below -

```c++
// Step 2 - If you can change to the left lane - Change to the right lane
      if(lane !=2 && !change_left && (current_timer - last_timer) > 100 ) {
      // Check if it is safe to change lanes
      bool valid_right = true;
      for(int i =0; i < sensor_fusion.size(); i++ ) {
      	double d = sensor_fusion[i][6];
      	if ( d < (2 + 4 * (lane + 1) + 2) && d > (2 + 4 * (lane + 1) -2) ) {
      		double vx = sensor_fusion[i][3];
      		double vy = sensor_fusion[i][4];
      		check_right_speed = sqrt(vx*vx+vy*vy);
      		double check_car_s = sensor_fusion[i][5];
      		double forward_dist = prev_size * 0.02 * check_right_speed;
      		check_car_s += forward_dist;
      		//Check the dist
      		if ( ((check_car_s > car_s) && ((check_car_s - car_s) < safety_gap))||
      			((check_car_s < car_s) && ((car_s - check_car_s) < 10))) {
      			valid_right = false ;
      			break;
      		}
      	}

      }
```
### Trajectory Planning and Generation

The main components of the trajectory planner is based on the method discussed during the walk through.

Following are the steps

      1. Include two points based on the previous path (General Case )
```c++

        ref_x = previous_path_x[prev_size - 1];
		ref_y = previous_path_y[prev_size - 1];

		double ref_x_prev = previous_path_x[prev_size -2];
		double ref_y_prev = previous_path_y[prev_size -2];

		// Update the reference angle based on these two points
		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

		// Push these two points onto the "anchor" as described in Walkthrough
		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);

		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
```
        2. Generate Frenet co-ordinates for the lane ( d value ) ( depending on which state it is in) and a s value of 30m, 60, 90m respectively.
```c++
        vector<double> next_wp0 = getXY(car_s + 30, 2 + 4.0 * lane, map_waypoints_s,
							map_waypoints_x, map_waypoints_y);
		vector<double> next_wp1 = getXY(car_s + 60, 2 + 4.0 * lane, map_waypoints_s,
							map_waypoints_x, map_waypoints_y);
		vector<double> next_wp2 = getXY(car_s + 90, 2 + 4.0 * lane, map_waypoints_s,
							map_waypoints_x, map_waypoints_y);
```
3. Get the equivalent (x,y) co-ordinates using the getXY function

4. Change from Global to Local ( ego car) co-ordinates

```c++
        double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;

		ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw) );
		ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw) );

```

3. Generate a spline using the spline function

```c++
    // Create a spline
    tk::spline s;
    s.set_points(ptsx, ptsy);// Takes vector of X and Y points
```

4. Generate the number of points based on the reference velocity and perform the conversion from local to global and add them to the waypoints.

```c++
    double N = target_dist / (0.02 * ref_vel/2.24); // 2.24 = 1.604 * 1000 / 3600
	double x_point = x_add_on + target_x/ N;
	double y_point = s(x_point);

	x_add_on = x_point;

	// Convert back based on regular frame of reference
	double x_ref = x_point;
	double y_ref = y_point;

	//Rotate back to normal
	x_point = ref_x + (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
	y_point = ref_y + (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

	next_x_vals.push_back(x_point);
	next_y_vals.push_back(y_point);
```

### Results Achieved

The planner is able to navigate the simulator withtout any incidents as shown in the video [here](https://youtu.be/Klz9Ec0shts)

![Alt text](result.png?raw=true "Result")

### Improvements in the Pipeline

1. Use more complicated cost functions (such as sigmoid / logistic).
2. Predict when other cars are going to change lanes (based on d value).

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

---

## Dependencies

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
