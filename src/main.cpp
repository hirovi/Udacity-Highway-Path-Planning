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

//Other init
//bool once = true;

class Car {
  public:
    int id;
    double x;
    double y;
    double v;
    double s;
    double dist_s; //Distance s from host vehicle to other vehicle
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


vector <double> check_lane(vector<Car>cars, double car_s, int target_lane){
  vector<Car> target_vehicles;
  vector<double> result;
  double absolute_dist = 0.0;
  std::cout<<0<<endl;
  for(int i=0; i<cars.size(); i++){
    int car_lane = what_lane(cars[i].d);

    if(car_lane == target_lane){
      target_vehicles.emplace_back(cars[i]);
    }

  }
  //std::cout<<1<<endl;
  //If road is empty, return "Yes, change lane"
  if(target_vehicles.size() == 0){
    //std::cout<<1.5<<endl;
    result.push_back(1);
    result.push_back(1000);
    //std::cout<<2<<endl;
    return result;
  }
  else{
    //Check what is the distance between the fron and rear vehicle in target lane
    //std::cout<<"Size target vehicles vector: "<<target_vehicles.size()<<endl;
    std::sort(target_vehicles.begin(),target_vehicles.end(), small_to_big);
    //std::cout<<2.5<<endl;
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
    //std::cout<<3<<endl;

    //When there are no cars in front but there is one behind
    if(positives.size()==0){
      double car_behind = abs(negatives.back());
      std::cout<<4<<endl;
      if(car_behind>5){
        result.push_back(1);
        result.push_back(1000);
        return result;
      }
    }

    //When there are no cars behind but there is one in front
    else if(negatives.size()==0){
      double car_front = positives[0];
      std::cout<<5<<endl;
      if(car_front>32){
        result.push_back(1);
        result.push_back(car_front);
        return result;
      }
    }
    else{
      //When there are cars in front and behind
      double car_behind = abs(negatives.back());
      double car_front = positives[0];
      //std::cout<<"Car_front: "<<car_front<<endl;
      //std::cout<<"Car_behind: "<<car_behind<<endl;
      //double abs_dist = car_front + car_behind;
      std::cout<<6<<endl;
      if(car_front > 32 && car_behind>5){
        //return a True, saying, yes change, and also state distance to the front car.
        result.push_back(1);
        result.push_back(car_front);
        std::cout<<7<<endl;
        return result;
        //std::cout<<7.5<<endl;
      }
      else{
        result.push_back(0);
        result.push_back(car_front);
        std::cout<<8<<endl;
        return result;
      }
    }
  }

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


  bool change_lane = false;
  //Start lane
  int host_lane = 1;
  double ref_vel = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &host_lane, &ref_vel, &change_lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

            int prev_size = previous_path_x.size();

            //Gradually increment speed without crossing acc./jerk limit
            if (ref_vel<49.1 && change_lane==false){
              ref_vel +=0.9;
            }

//---------------------------Sensor Fusion------------------------------------//
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

            vector <Car> front_cars;
            int safe_distance = 30;
            host_lane = what_lane(car_d);
            std::cout<<host_lane<<endl;
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
              std::cout<<front_cars[0].dist_s<<endl;
              //Check safe distance and velocity diference
              //By doing velocity difference, the transition of slowing down is
              //smoother.
              //Tried different methods, even with a proportion to distance.
              //Ideally would be to build a PID to control the speed.

              if(front_cars[0].dist_s < safe_distance){
                change_lane = true;
                if(ref_vel > front_cars[0].v){
                  ref_vel-=0.25;
                }
              }
              else{change_lane = false;
              std::cout<<"I'm here"<<endl;}
            }
            //Don't change lane if there are no cars in front
            else{change_lane = false;
            std::cout<<"For some readon I'm here"<<endl;}
            //std::cout<<host_lane

//---------------------------Lane Changer-------------------------------------//
            //What lane should I change to?
            //If I'm on the left lane I can only turn right,
            //If I'm in the middle I can choose, and so on...
            //Parameters in consideration:
            //dist_s
            //speed difference
            //
            // bool change_left = false;
            // bool change_right = false;
            //
            // //std::cout<<host_lane<<endl;
            // if(change_lane == true){
            //
            //
            //   if (host_lane == 0){
            //     change_right = check_lane(cars, car_s, 1);
            //   }
            //   else if (host_lane == 1){
            //     change_left = check_lane(cars, car_s, 0);
            //     change_right = check_lane(cars, car_s, 2);
            //   }
            //   else if (host_lane == 2){
            //     change_left = check_lane(cars, car_s, 1);
            //   }
            //
            //
            //
            //   if (change_left == true && host_lane == 1){
            //     lane=0;
            //     change_left = false;
            //   }
            //   if (change_left == true && host_lane == 2){
            //     lane=1;
            //     change_left = false;
            //   }
            //   if (change_right == true && host_lane == 0){
            //     lane=1;
            //     change_right = false;
            //   }
            //   if (change_right == true && host_lane == 1){
            //     lane=2;
            //     change_right = false;
            //   }
            // }
            //Need to init otherwise it will crash after in the if conditions
            vector<double> change_left {0, 0};
            vector<double> change_right {0, 0};


            //std::cout<<host_lane<<endl;
            if(change_lane == true){

              if (host_lane == 0){
                change_right = check_lane(cars, car_s, 1);
                if(change_right[0] == true){host_lane=1;}
              }
              else if (host_lane == 1){
                change_left = check_lane(cars, car_s, 0);
                change_right = check_lane(cars, car_s, 2);

                if(change_left[0] == true && change_right[0] == false){
                  host_lane=0;
                }

                else if(change_left[0] == false && change_right[0] == true){
                  host_lane=2;
                }

                else if (change_left[0] == true && change_right[0] == true){
                  if(change_left[1] > change_right[1]){hos
