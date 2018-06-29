//Author: Oscar Rovira (hirovi)

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

//Create class car that will correspond to each of the cars in the road
class Car {
  public:
    int id;
    double x;
    double y;
    double v;
    double s;
    double dist_s; //Distance s from host vehicle to this vehicle.
    double d;

};

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.

//////////////////////////////Udacity helpers///////////////////////////////////

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
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y)
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

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x,
                        const vector<double> &maps_y)
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
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x, const vector<double> &maps_y)
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

///////////////////////////////Custom helpers///////////////////////////////////

//Return what lane the car is at
int what_lane(double d){
  if (d > 0 && d < 4) return 0;
  else if (d > 4 && d < 8) return 1;
  else if (d > 8 && d < 12) return 2;
  else return -1;
}

//https://stackoverflow.com/questions/14081335/algorithm-vector-sort-with-objects
bool small_to_big(const Car &s1, const Car &s2){
  return s1.dist_s < s2.dist_s;
}

//Function that returns a vector with (can_I_change lane? & distance from host to front car)
vector <double> check_lane(vector<Car>cars, double car_s, double dist_to_front, int target_lane){
  vector<Car> target_vehicles;
  vector<double> result;
  double absolute_dist = 0.0;

  for(int i=0; i<cars.size(); i++){
    int car_lane = what_lane(cars[i].d);

    if(car_lane == target_lane){
      target_vehicles.emplace_back(cars[i]);
    }

  }
  //If road is empty, return "Yes, change lane"
  if(target_vehicles.size() == 0){
    result.push_back(1);
    result.push_back(1000);
    return result;
  }
  else{
    std::sort(target_vehicles.begin(),target_vehicles.end(), small_to_big);

    //Check what is the distance between the front and rear vehicle in target lane
    vector <double> negatives;
    vector <double> positives;
    for (int i=0; i<target_vehicles.size(); i++){
      if(target_vehicles[i].dist_s < 0){
        negatives.push_back(target_vehicles[i].dist_s);
      }
      else{
        positives.push_back(target_vehicles[i].dist_s);
      }
    }

    //When there are no cars in front but there is one behind
    if(positives.size()==0){
      double car_behind = abs(negatives.back());
      if(car_behind>5){
        result.push_back(1);
        result.push_back(1002);
        return result;
      }
      else{
        result.push_back(0);
        result.push_back(1000);
        return result;
      }
    }

    //When there are no cars behind but there is one in front
    else if(negatives.size()==0){
      double car_front = positives[0];
      if(car_front>dist_to_front){
        result.push_back(1);
        result.push_back(car_front);
        return result;
      }
      else{
        result.push_back(0);
        result.push_back(car_front);
        return result;
      }
    }
    else{
      //When there are cars in front and behind
      double car_behind = abs(negatives.back());
      double car_front = positives[0];
      if(car_front > dist_to_front && car_behind>5){
        //return a True, saying, yes change, and also state distance to the front car.
        result.push_back(1);
        result.push_back(car_front);
        return result;

      }
      else{
        result.push_back(0);
        result.push_back(car_front);
        return result;
      }
    }
  }

}
////////////////////////////////////////////////////////////////////////////////

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


  bool change_lane = false;
  int host_lane = 1;
  double ref_vel = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,
               &map_waypoints_dy, &host_lane, &ref_vel, &change_lane]
               (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////TASK//////////////////////////////////////////

//-----------------------------Debugging--------------------------------------//

            // std::cout<<"S: "<<car_s<<endl;
            // std::cout<<"D: "<<car_d<<endl;
            // std::cout<<"previous_path_x: "<<previous_path_x<<endl;
            // std::cout<<"previous_path_y: "<<previous_path_y<<endl;
            // std::cout<<"Last defined S: "<<end_path_s<<endl;
            // std::cout<<"Last defined D: "<<end_path_d<<endl;
            // std::cout<<"Speed(mph): "<<car_speed<<endl;

            // for(int i=0; i<sensor_fusion.size();i++){
            //   std::cout<<"id: "<<sensor_fusion[i][0]<<" s: "<<sensor_fusion[i][5]
            //            <<" d: "<<sensor_fusion[i][6]<<endl;
            // }

//-------------------------------Init-----------------------------------------//
//About: Set the gradual initial speed so that the car slowly reaches target speed

            int prev_size = previous_path_x.size();

            //Gradually increment speed without crossing acc./jerk limit
            if (ref_vel<=49 && change_lane==false){
              ref_vel +=0.9;
            }

//---------------------------Sensor Fusion------------------------------------//
//About: Store all the data from the sensor_fusion array into a car class.
            //[id, x, y, vx, vy, s, d]
            vector<Car> cars;
            for(int i=0; i<sensor_fusion.size(); i++){
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              Car car;
              car.id = sensor_fusion[i][0];
              car.x = sensor_fusion[i][1];
              car.y = sensor_fusion[i][2];
              car.v = sqrt((vx*vx)+(vy*vy))*2.24;
              car.s = sensor_fusion[i][5];
              car.dist_s = car.s - car_s;
              car.d = sensor_fusion[i][6];
              cars.emplace_back(car);
            }

//---------------------------Crash Avoider------------------------------------//
//About: Check distance from car to front car and reduce velocity accordingly.
//       Check safe distance and velocity diference
//       By doing velocity difference, the transition of slowing down is smoother.
//       Tried different methods. BUt the ideal would be a PID.

            vector <Car> front_cars;
            int safe_distance = 30;
            host_lane = what_lane(car_d);
            double dist_to_front = 0.0;

            for(int i=0; i<cars.size(); i++){
              int car_lane = what_lane(cars[i].d);

              //If front car is in same lane and its distance
              //is greater than mine(checking it's in front)
              if (car_lane == host_lane && cars[i].dist_s > 0){
                front_cars.emplace_back(cars[i]);
              }

            }
            if (front_cars.size()>0){
              std::sort(front_cars.begin(),front_cars.end(), small_to_big);
              dist_to_front = front_cars[0].dist_s;
              std::cout<<"Distance to front car: "<<dist_to_front<<" m"<<endl;
              if(dist_to_front < safe_distance){
                change_lane = true;
                if(ref_vel > front_cars[0].v){
                  ref_vel-=0.25;
                }
              }
              else{change_lane = false;
              }
            }
            //Don't change lane if there are no cars in front
            else{change_lane = false;
              std::cout<<"Distance to front car: "<<"too far"<<endl;
            }


//---------------------------Lane Changer-------------------------------------//
//About: Call function "check_lane" to flag when is legal to change lane.

            vector<double> change_left {0, 0};
            vector<double> change_right {0, 0};
            double diff_abs = 0.0;

            if(change_lane == true){

              if (host_lane == 0){
                change_right = check_lane(cars, car_s, dist_to_front, 1);
                if(change_right[0] == true){host_lane=1;}
              }
              else if (host_lane == 1){
                change_left = check_lane(cars, car_s, dist_to_front, 0);
                change_right = check_lane(cars, car_s, dist_to_front, 2);

                if(change_left[0] == true && change_right[0] == false){
                  host_lane=0;
                }

                else if(change_left[0] == false && change_right[0] == true){
                  host_lane=2;
                }

                else if (change_left[0] == true && change_right[0] == true){
                  //To avoid uncertainty & wabbling when changing lanes & Distance
                  //to front cars  are similar.
                  diff_abs = abs(change_left[1]-change_right[1]);
                  if(diff_abs>1.5){
                    if(change_left[1] >= change_right[1]){host_lane=0;}
                    else{host_lane=2;}
                  }
                  else{host_lane=1;}
                }
              }

              else if (host_lane==2){
                change_left = check_lane(cars, car_s, dist_to_front, 1);
                if(change_left[0] == true){host_lane=1;}
              }
            }

//--------------------------Spline Creation----------------------------------//
//About: Create 5 points that the Spline will use to generate the function that
//       will connect(See next block) such points in a smooth and sexy way.


            // Create a set of points (x,y)
		vector<double> ptsx;
		vector<double> ptsy;

		// Reference x, y, yaw states
		double ref_x = car_x;
		double ref_y = car_y;
		double ref_yaw = deg2rad(car_yaw);

		// If previous size is almost empty, use the car as starting reference
		if (prev_size < 2) {
			// Use two points that make the path tangent to the car
			double prev_car_x = car_x - cos(car_yaw);
			double prev_car_y = car_y - sin(car_yaw);

			ptsx.push_back(prev_car_x);
			ptsx.push_back(car_x);

			ptsy.push_back(prev_car_y);
			ptsy.push_back(car_y);
		} else {

			// Use last points from previous path
			ref_x = previous_path_x[prev_size-1];
			ref_y = previous_path_y[prev_size-1];

			// Use before the last point from previous path
			double prev_ref_x = previous_path_x[prev_size-2];
			double prev_ref_y = previous_path_y[prev_size-2];
			ref_yaw = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);

			ptsx.push_back(prev_ref_x);
			ptsx.push_back(ref_x);

			ptsy.push_back(prev_ref_y);
			ptsy.push_back(ref_y);
		}

	vector<double> next_wp0;
		vector<double> next_wp1;
		vector<double> next_wp2;

	if(change_lane!=true){
		// Using Frenet, add 30 m evenly spaced points ahead of the starting reference
		  next_wp0 = getXY(car_s+30, (2+4*host_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  next_wp1 = getXY(car_s+60, (2+4*host_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  next_wp2 = getXY(car_s+90, (2+4*host_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	}
	else{
		  next_wp0 = getXY(car_s+55, (2+4*host_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  next_wp1 = getXY(car_s+60, (2+4*host_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  next_wp2 = getXY(car_s+90, (2+4*host_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	}


		ptsx.push_back(next_wp0[0]);
		ptsx.push_back(next_wp1[0]);
		ptsx.push_back(next_wp2[0]);

		ptsy.push_back(next_wp0[1]);
		ptsy.push_back(next_wp1[1]);
		ptsy.push_back(next_wp2[1]);

        //Go from global to local coordinates so that x0=0, y0=0 and yaw=0
	for (int i = 0; i < ptsx.size(); i++) {
		// Shift car reference angle to 0 degrees
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;

		ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
		ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
	}
	// Create a spline called s
	tk::spline s;

	// Set (x,y) points to the spline
	s.set_points(ptsx, ptsy);


//----------------------------Waypoint Planner--------------------------------//
//About: Create each of the individual points in x that the car fill follow and
//       feed them into the spline function to look for the corresponding y

      			// Define the actual (x,y) points we will use for the planner
      			vector<double> next_x_vals;
      			vector<double> next_y_vals;

      			// Start with all the previous path points from last time
      			for (int i = 0; i < previous_path_x.size(); i++) {
      				next_x_vals.push_back(previous_path_x[i]);
      				next_y_vals.push_back(previous_path_y[i]);
      			}

      			// Compute how to break up spline points so we travel at our desired reference velocity
      			double target_x = 30.0;
      			double target_y = s(target_x);
      			double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
      			double x_add_on = 0;

      			// Fill up the rest of the path planner to always output 50 points
      			for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
      				double N = (target_dist/(.02*ref_vel/2.24));
      				double x_point = x_add_on + (target_x) / N;
              //Finally use the spline to look for the xy points
      				double y_point = s(x_point);

      				x_add_on = x_point;

      				double x_ref = x_point;
      				double y_ref = y_point;

      				// Go back to local coordinates
      				x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
      				y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

      				x_point += ref_x;
      				y_point += ref_y;

      				next_x_vals.push_back(x_point);
      				next_y_vals.push_back(y_point);
            }

///////////////////////////////END//////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
            json msgJson;

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
