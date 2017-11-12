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
#include "spline.h"

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

// check for collison as well as buffer space behind and ahead of the ego vehicle
bool check_collision(double ego_s, double ego_lane, double ego_v, int path_size, const vector<vector<double> >& sensor_fusion) {
  //cout << "ego: s=" << ego_s << ", lane=" << ego_lane << ", v=" << ego_v << endl;

  double buffer = 15;

  // check for collisions at the remaining path points
  for (int i=0; i<50-path_size; i++) {
    // for simplicity, assume constant speed
    ego_s += ego_v * 0.02 * (i+1);

    for (auto sensor_data: sensor_fusion) {
      double d = sensor_data[6];
      int lane = int(d) / 4;
      if (lane != ego_lane) //ignore cars not in lane
        continue;
      double vx = sensor_data[3], vy = sensor_data[4];
      double v = sqrt(vx*vx + vy*vy);
      double s = sensor_data[5];
      s += v * 0.02 * (path_size+i+1);

      if (abs(s - ego_s) < buffer) // ego vehcile is too close
        return true;
    }
  }
  return false;
}

// A combined cost function for:
// 1. lane keep/chnage preferences
// 2. how far propsed speed from traget speed
double calc_cost(int proposed_lane, int actual_lane, double proposed_speed) {
    /*
    Cost becomes higher for trajectories with slower traffic. 
    */
    double target_speed = 50.0; // mph

    // lane change penalties: 0 for keep lane, 1 for change lane left, 2 for change lane right
    int lane_shift = proposed_lane - actual_lane;
    lane_shift = (lane_shift < 0)? 1 : (lane_shift > 0)? 2 : 0;

    // Using a higher factor (x1000) for slower car than for lane change penalty (x10)
    double cost = 1000*((target_speed - proposed_speed)/target_speed) + 10*lane_shift;

    return cost;
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

  double ref_vel = 0;
  int lane = 1;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&ref_vel,&lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          // if (car_speed > 50)
          //   cout << "Speed violation at " << car_speed << " mph" << endl;

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

          double pos_x;
          double pos_y;
          double angle;
          double pos_s;
          double pos_d;
          int path_size = previous_path_x.size();

          // First add the previous path points
          for(int i = 0; i < path_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double pos_x2, pos_y2;

          if(path_size == 0)
          {
            pos_x = car_x;
            pos_y = car_y;
            angle = deg2rad(car_yaw);

            pos_x2 = car_x - cos(angle);
            pos_y2 = car_y - sin(angle);

            pos_s = car_s;
            pos_d = car_d;
          }
          else
          {
            pos_x = previous_path_x[path_size-1];
            pos_y = previous_path_y[path_size-1];

            pos_x2 = previous_path_x[path_size-2];
            pos_y2 = previous_path_y[path_size-2];
            angle = atan2(pos_y-pos_y2,pos_x-pos_x2);

            pos_s = end_path_s;
            pos_d = end_path_d;       
          }

          double ref_x=pos_x, ref_y=pos_y;

          bool collision = true;
          int min_cost_lane = lane;
          double min_cost_speed = ref_vel;
          double min_cost = pow(10, 6);

          // try lane-1, lane, lane+1
          for (int l=lane-1; l<=lane+1; l++) {
            if (l<0 || l>2)
              continue;

            // try ref_vol-0.4, ref_vel, ref_vel+0.4
            for (int vi=-1; vi<=1; vi++) {
              double vel = ref_vel + vi*0.4;
              if (vel<=0 || vel>49.4)
                continue;

              // check collision and buffer
              if (check_collision(pos_s, l, vel, path_size, sensor_fusion))
                continue;

              collision = false;
              double new_cost = calc_cost(l, lane, vel);
              if (new_cost < min_cost) {
                min_cost = new_cost;
                min_cost_lane = l;
                min_cost_speed = vel;
              }
              //cout << "new_cost=" << new_cost << ", min_cost=" << min_cost << endl;
            }
          }

          // update lane and ref_cel associated with min_cost
          if (!collision) {
            lane = min_cost_lane;
            ref_vel = min_cost_speed;
          }
          else
            ref_vel -= 0.4;

          //cout << "lane=" << lane << ", ref_vel=" << ref_vel << ", min_cost=" << min_cost << endl;
          assert (lane>=0 && lane<=2 && ref_vel>0);

          // Take 5 points to pass to the spline
          vector<double> ptsx, ptsy;
          ptsx.push_back(pos_x2);
          ptsy.push_back(pos_y2);

          ptsx.push_back(pos_x);
          ptsy.push_back(pos_y);

          // space the points by 30m
          for (int step_s=30; step_s<100; step_s+=30) {
            double next_s = pos_s + step_s;
            vector<double> next_xy = getXY(next_s, 2+lane*4, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_xy[0]); // x
            ptsy.push_back(next_xy[1]); // y
          }

          //convert ptsx, ptsy to the local car coordinates
          for (int i=0; i<ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = shift_x*cos(-angle) - shift_y*sin(-angle);
            ptsy[i] = shift_x*sin(-angle) + shift_y*cos(-angle);
          }

          // Add the 5 points to the spline
          tk::spline s;
          s.set_points(ptsx, ptsy);

          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          double dist_inc = ((ref_vel*1.609*1000)/3600) * 0.02;
          double N = target_dist / dist_inc;

          vector<double> spline_ptsx, spline_ptsy;

          for(int i = 0; i < 50-path_size; i++)
          {
            double pt_x = (i+1) * target_x / N;
            double pt_y = s(pt_x);

            // convert back to the map coordinates
            double x_ref=pt_x, y_ref=pt_y;

            pt_x = x_ref*cos(angle) - y_ref*sin(angle);
            pt_y = x_ref*sin(angle) + y_ref*cos(angle);

            pt_x += ref_x;
            pt_y += ref_y;

            next_x_vals.push_back(pt_x);
            next_y_vals.push_back(pt_y);
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
