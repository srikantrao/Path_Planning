#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "helper.h"
#include "spline.h"
using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() {
	return M_PI;
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos) {
		return "";
	} else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

int main() {
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line)) {
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	int lane = 1;
	// Parametrize the safety gap that is needed
	double safety_gap = 40;
	// Have a reference velocity target
	double ref_vel = 0.0; // mph

	int prev_size = 0; // will be updated

	long last_timer = 0;

	long current_timer = 0;

	h.onMessage(
			[&last_timer, &current_timer, &lane, &ref_vel, &safety_gap, &prev_size, &map_waypoints_x,&map_waypoints_y,
			 &map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy]
			 (uWS::WebSocket<uWS::SERVER> ws,char *data, size_t length, uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);
		//cout << sdata << endl;

		// Increment the timer
		current_timer++ ;
		if (length && length > 2 && data[0] == '4' && data[1] == '2') {

			auto s = hasData(data);

			if (s != "") {
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry") {
					// j[1] is the data JSON object

					// Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];

					// Previous path data given to the Planner
					vector<double> previous_path_x = j[1]["previous_path_x"];
					vector<double> previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

					// Keep track of Previous Size based on project walkthrough
					prev_size = previous_path_x.size();

					// Using Sensor Fusion to avoid collision
					if(prev_size > 0) {
						car_s = end_path_s;
					}

					bool too_close = false;

					// Iterate over all the other cars
					// sensor_fusion : First Intex - Different Cars, Second Index - Look at README

					double check_lane_speed = 0;
					double check_left_speed = 0;
					double check_right_speed = 0;

					// There can be multiple cars in the same lane - check the smallest distance to set ref_vel
					double closest_car_distance = 10000;
					double closest_car_left_distance = 10000;
					double closest_car_right_distance = 10000;

					for (int i =0; i < sensor_fusion.size(); i++) {

						double d = sensor_fusion[i][6];
						// check if car is in the same lane
						if ( d < (2 + 4 * lane + 2) && d > (2 + 4 * lane -2) ) {

							// Check speed of car that is in lane
							double vx = sensor_fusion[i][3];
							double vy = sensor_fusion[i][4];
							check_lane_speed = sqrt(pow(vx,2) + pow(vy,2));// m/s
							double check_car_s = sensor_fusion[i][5];

							// check where the car will be at end of current planned path
							// Break it down to understand this better
							double forward_dist = prev_size * 0.02 * check_lane_speed;
							check_car_s += forward_dist;

							// check if the s value of the car in the lane
							if ( (check_car_s > car_s) && ((check_car_s - car_s) < safety_gap)) {
								// Try do one of 2 possible things - 1. Slow Down 2. Change lanes
								too_close = true;
								cout << "Car is too close right now" << endl ;
								if((check_car_s - car_s) < closest_car_distance) {
									closest_car_distance = check_car_s - car_s;
								}
							}
						}

					}

					/*
					 * Simple FSM for now - 1. Try to turn Left 2. Try to turn right 3. Stay in same lane
					 */
					bool change_left = false ;
					bool change_right= false ;
					if (too_close) {
						// Step 1. Try to change to the left lane
						//Check if you are already in the left most lane
						if(lane !=0  && (current_timer - last_timer) > 100) {

							// Check if it is safe to change lanes
							bool valid_left = true ;
							for(int i =0; i < sensor_fusion.size(); i++ ) {
								double d = sensor_fusion[i][6];
								if ( d < (2 + 4 * (lane -1) + 2) && d > (2 + 4 * (lane -1) -2) ) {
									double vx = sensor_fusion[i][3];
									double vy = sensor_fusion[i][4];
									check_left_speed = sqrt(vx*vx+vy*vy);
									double check_car_s = sensor_fusion[i][5];
									double forward_dist = prev_size * 0.02 * check_left_speed;
									check_car_s += forward_dist;
									// Check if car_dist
									if ( ((check_car_s > car_s) && ((check_car_s - car_s) < safety_gap))||
											((check_car_s < car_s) && ((car_s - check_car_s) < 10))) {
										valid_left = false ;
										break;
									}

								}
							}

							if(valid_left){
								// Can make a legal change to left lane
								lane -=1;
								// slow down to the speed of the car ahead
								if(ref_vel > check_left_speed * 2.24) ref_vel -= .224;
								cout << "Changing to the Left Lane -1 " << endl;
								change_left = true;
								last_timer = 0;
								current_timer = 0 ;
							}

						}

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
							if(valid_right){
								// Can make a legal change to left lane
								lane +=1;
								// slow down to the speed of the car ahead
								if(ref_vel > check_left_speed * 2.24) ref_vel -= .224;
								cout << "Changing to the Right Lane -1 " << endl;
								change_right = true;
								last_timer = 0;
								current_timer = 0 ;
							}

						}
						// Step 3 - Stay in the same lane
						if( !change_left && !change_right) {
							if(closest_car_distance < 10)
								ref_vel -=.4;
							if(closest_car_distance < 20 )
								ref_vel -= .338;
							else if (ref_vel > (check_lane_speed * 2.24)) ref_vel -= .224 ;
							cout << "Staying in lane - following car less in the lane at : " <<
									closest_car_distance << " ms" << endl ;
						}

					}

					else if(ref_vel < 47.5) {

						ref_vel += .224;
						cout << "No car ahead ..Accelerating:" << ref_vel << endl;
					}

					vector<double> ptsx;
					vector<double> ptsy;

					// reference point - will be either endpoint of previous path or current car (x,y) -- Explain ??
					double ref_x = car_x;
					double ref_y = car_y;
					double ref_yaw = deg2rad(car_yaw);

					// What was the previous path size that has not been taken
					if(prev_size < 2) {

						// Use the points that make the path tangent to the car -- Look into why this is tangent
						double prev_car_x = car_x - cos(ref_yaw);
						double prev_car_y = car_y - sin(ref_yaw);

						// Add the two points from the previous path
						ptsx.push_back(prev_car_x);
						ptsx.push_back(car_x);

						ptsy.push_back(prev_car_y);
						ptsy.push_back(car_y);

					}

					// If you have more remaining previous path points - Why dont they all get used up ?
					else {
						// Use only the last two points

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

					}

					json msgJson;

					// This is going to store the final points
					vector<double> next_x_vals;
					vector<double> next_y_vals;

					// Add some random waypoints in Frenet Co-ordinates - 30ms apart right now
					vector<double> next_wp0 = getXY(car_s + 30, 2 + 4.0 * lane, map_waypoints_s,
							map_waypoints_x, map_waypoints_y);
					vector<double> next_wp1 = getXY(car_s + 60, 2 + 4.0 * lane, map_waypoints_s,
							map_waypoints_x, map_waypoints_y);
					vector<double> next_wp2 = getXY(car_s + 90, 2 + 4.0 * lane, map_waypoints_s,
							map_waypoints_x, map_waypoints_y);

					// Now push these onto the "anchor" - We now have 5 anchor points
					ptsx.push_back(next_wp0[0]);
					ptsx.push_back(next_wp1[0]);
					ptsx.push_back(next_wp2[0]);

					ptsy.push_back(next_wp0[1]);
					ptsy.push_back(next_wp1[1]);
					ptsy.push_back(next_wp2[1]);

					// Make the change to these anchor points so that it is now in the car's reference frame
					for ( int i = 0; i < ptsx.size(); i++) {
						double shift_x = ptsx[i] - ref_x;
						double shift_y = ptsy[i] - ref_y;

						ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw) );
						ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw) );

					} // Lookout for why this is useful.

					// Create a spline
					tk::spline s;
					s.set_points(ptsx, ptsy);// Takes vector of X and Y points

					// Add all the points from the previous points that have not been added yet
					for (int i =0; i < previous_path_x.size(); i++) {
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					} // Avoid recreating the path every single time

					// Calculate how to break up spline points so that we travel at  our desired ref_vel
					// Action is taken every 0.02 seconds so calculate the number of points needed
					// Based on N * 0.02 * ref_vel = reference distance --> 30m
					// Why is it okay to linearize this calculation ?

					double target_x = 30.0;
					double target_y = s(target_x);
					double target_dist = sqrt(pow(target_x,2) + pow(target_y, 2));

					double x_add_on = 0;

					for(int i = 1; i <= 50 - previous_path_x.size(); i++ ) {
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

					}

					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\","+ msgJson.dump()+"]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

				}
			} else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
			size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		} else {
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
			char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
