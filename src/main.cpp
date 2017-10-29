#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

#define EXTREME

#ifdef EXTREME
  // Velocity and Acceleration values for testing high speed and acceleration.
  #define MAX_VELOCITY 99.5
  #define MAX_ACCEL 1.500
#else
  // Velocity and Acceleration values to meet requirements based on project walk through.
  #define MAX_VELOCITY 49.5
  #define MAX_ACCEL 0.224
#endif

// Constant MPH to MPS.
#define MPH_TO_MPS 2.24

// Vehicle buffer to avoid collisions (m).
#define VEHICLE_BUFFER 3.0

// Lane Width (m).
#define LANE_WIDTH 4.0

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

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

  // Initial Lane position.
  int curr_lane = 1;
  // Initial State ("KL" = Keep Lane, "LCL" = Lane Change Left, "LCR" = Lane Change Right).
  string curr_state="KL";

  // Current and target velocities (mph).
  double curr_v = 0.0;
  double target_v = MAX_VELOCITY;

  // Target lane for a Lane Change state.
  int target_lane = -1;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,
               &curr_lane,&target_lane,&curr_state,&curr_v,&target_v](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            int prev_size = previous_path_x.size();
            int sensor_fusion_size = sensor_fusion.size();

            // Collision avoidance.
            double lane_boundary_min = LANE_WIDTH/2 + LANE_WIDTH*curr_lane - LANE_WIDTH/2;
            double lane_boundary_max = LANE_WIDTH/2 + LANE_WIDTH*curr_lane + LANE_WIDTH/2;
            bool too_close = false;
            double end_car_s = (prev_size > 0)?end_path_s:car_s;

            for(unsigned int i = 0; i < sensor_fusion_size; ++i)
            {
              // Check if car is in this lane.
              float other_d = sensor_fusion[i][6];
              if(other_d<lane_boundary_max && other_d>lane_boundary_min)
              {
                double other_vx = sensor_fusion[i][3];
                double other_vy = sensor_fusion[i][4];
                double other_v = sqrt(other_vx*other_vx+other_vy*other_vy);
                double end_other_s = (double)(sensor_fusion[i][5]) + (double)(prev_size)*0.02*other_v;
                if(end_other_s>end_car_s && end_other_s-end_car_s < curr_v/2)
                {
                  // Change target velocity to match the vehicle ahead until a lane change can be performed.
                  target_v = other_v;
                  too_close = true;
                }
              }
            }

            if(!too_close)
              target_v = MAX_VELOCITY;

            if(target_v < curr_v)
            {
              curr_v -= MAX_ACCEL;
            }
            else if(target_v > curr_v)
            {
              curr_v += MAX_ACCEL;
            }

            // Finite State machine with Cost Function.
            if(curr_state == "LCL")
            {
              // Lane Change Left.
              if(target_lane==-1)
                target_lane = --curr_lane;
              else if(car_d<lane_boundary_max)
                curr_state = "KL";
            }
            else if(curr_state == "LCR")
            {
              // Lane Change Right.
              if(target_lane==-1)
                target_lane = ++curr_lane;
              else if(car_d>lane_boundary_min)
                curr_state = "KL";
            }
            else
            {
              // Currently in Keep Lane.
              target_lane = -1;

              map<string,int> options = {
                  std::make_pair ("LCR", 1),
                  std::make_pair ("KL", 0),
                  std::make_pair ("LCL", -1)
              };

              string next_state = "";
              double next_cost = -1.0;
              double act_d, act_s, act_speed, totcost;

              // Loop through possible state alternatives.
              for (auto const& option : options)
              {
                string action = option.first;
                int laneAction = curr_lane + option.second;

                if(laneAction>=0 && laneAction<=2)
                {
                  lane_boundary_min = LANE_WIDTH/2 + LANE_WIDTH*laneAction - LANE_WIDTH/2;
                  lane_boundary_max = LANE_WIDTH/2 + LANE_WIDTH*laneAction + LANE_WIDTH/2;

                  // Check for distance to vehicle in front.
                  act_s = 9999.0;
                  for(unsigned int i = 0; i < sensor_fusion_size; ++i)
                  {
                    // Check if car is in this lane.
                    float other_d = sensor_fusion[i][6];
                    if(other_d < lane_boundary_max && other_d > lane_boundary_min)
                    {
                      double other_vx = sensor_fusion[i][3];
                      double other_vy = sensor_fusion[i][4];
                      double other_v = sqrt(other_vx*other_vx+other_vy*other_vy);
                      double end_other_s = (double)(sensor_fusion[i][5]) + (double)(prev_size)*0.02*other_v;
                      if(end_other_s>end_car_s)
                      {
                        end_other_s -= end_car_s;
                        if(act_s > end_other_s)
                        {
                          act_s = end_other_s;
                          act_speed = other_v;
                        }
                      }
                    }
                  }
                  double distcost = 1.0/act_s * 1000.0;
                  // Remove 'noise' from cars in the far distance.
                  if(distcost<0.25)
                    distcost = 0.0;

                  double speedcost = (act_speed==0.0) ? 0.0 : 1.0/act_speed * 10.0;

                  // Check for collision avoidance.
                  double collcost = 0.0;
                  for(unsigned int i = 0; i < sensor_fusion_size; ++i)
                  {
                    // Check if car is in this lane.
                    float other_d = sensor_fusion[i][6];
                    if(other_d < lane_boundary_max && other_d > lane_boundary_min)
                    {
                      double other_vx = sensor_fusion[i][3];
                      double other_vy = sensor_fusion[i][4];
                      double other_v = sqrt(other_vx*other_vx+other_vy*other_vy);
                      double other_s = sensor_fusion[i][5];
                      // Check car is close now.
                      if(other_s > car_s-VEHICLE_BUFFER && other_s < car_s+VEHICLE_BUFFER)
                      {
                        collcost = 999.9;
                      }
                      else
                      {
                        // Check car is close in future projection.
                        double end_other_s = other_s + (double)prev_size*0.02*other_v;
                        if(end_other_s > end_car_s-VEHICLE_BUFFER && end_other_s < end_car_s+VEHICLE_BUFFER)
                        {
                          collcost = 999.9;
                        }
                      }
                    }
                  }

                  // Prevent lane changing for minimal gains.
                  double changecost = (double)(curr_lane!=laneAction)*9.0;

                  // Bias towards centre lane driving.
                  double centrecost = (double)(laneAction!=1)*10.0;

                  double totcost = distcost + speedcost + collcost + changecost + centrecost;
                  if(next_cost == -1.0 || totcost < next_cost)
                  {
                    next_state = action;
                    next_cost = totcost;
                  }
                }
              }
              curr_state = next_state;
            }

            // Extrapolate list of x/y coords ~30m apart.
            vector<double> ptsx;
            vector<double> ptsy;

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = car_yaw;

            if(prev_size<2)
            {
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
            }
            else
            {
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];

              double prev_ref_x = previous_path_x[prev_size-2];
              double prev_ref_y = previous_path_y[prev_size-2];

              ref_yaw = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);

              ptsx.push_back(prev_ref_x);
              ptsx.push_back(ref_x);

              ptsy.push_back(prev_ref_y);
              ptsy.push_back(ref_y);
            }

            vector<double> next_mp0 = getXY(end_car_s+30.0, 2.0+4.0*curr_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_mp1 = getXY(end_car_s+60.0, 2.0+4.0*curr_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_mp2 = getXY(end_car_s+90.0, 2.0+4.0*curr_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_mp0[0]);
            ptsy.push_back(next_mp0[1]);
            ptsx.push_back(next_mp1[0]);
            ptsy.push_back(next_mp1[1]);
            ptsx.push_back(next_mp2[0]);
            ptsy.push_back(next_mp2[1]);

            // Shift reference to car frame.
            for(unsigned int i=0;i<ptsx.size();++i)
            {
              double shift_x = ptsx[i]-ref_x;
              double shift_y = ptsy[i]-ref_y;

              ptsx[i] = shift_x*cos(-ref_yaw)-shift_y*sin(-ref_yaw);
              ptsy[i] = shift_x*sin(-ref_yaw)+shift_y*cos(-ref_yaw);
            }

            tk::spline s;

            s.set_points(ptsx, ptsy);

            // Note that previous_path_* only contains values that have not been already 'passed' by the simulator.
            for(unsigned int i=0;i<prev_size;++i)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // Break up spline points so the resulting speed is the target_speed.
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x+target_y*target_y);
            double x_add_on = 0.0;
            double N = target_dist/(0.02*curr_v/MPH_TO_MPS);

            for(unsigned int i=1; i<=50-prev_size; ++i)
            {
              double x_point = x_add_on+target_x/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
              y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

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
