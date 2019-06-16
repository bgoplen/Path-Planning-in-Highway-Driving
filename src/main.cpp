#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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
  double ref_vel = 0.0;
  
  h.onMessage([&lane,&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

/* commenting out - switching to all metric
          const double MAX_VEL_INC=0.447; // 10m/sec^2 * 3600 sec/hr * 1 mile/1609m * 0.02 sec
          const double VEL_INC = 0.224;  // 0.447/2
          const double MAX_VEL = 49.5;
          const double TIME_INC = 0.02;
*/
          const double MAX_VEL_INC=0.2; // 10m/sec^2 * 0.02 sec
          const double VEL_INC = 0.1;  // 0.2/2 because it can go from +VEL_INC to -VEL_INC
          const double MAX_VEL = 22.1; // 49.5 mile/hr * 0.447 :convert to meters per sec
          const double TIME_INC = 0.02;          
          
          int prev_size = previous_path_x.size();
          if (prev_size > 0) {
            car_s = end_path_s;           // collisions avoidance
          }
          
          //sensor fusion
          double front_vel = 100; // dummy max value
          double front_dist = 100; // dummy max value
          double right_vel = 100; // dummy max value
          double right_dist = 100; // dummy max value
          double left_vel = 100; // dummy max value
          double left_dist = 100; // dummy max value
          if (lane < 1) {
            left_vel = 0;
            left_dist = 0;
          } else if (lane > 1) {
            right_vel = 0;
            right_dist = 0;
          }
          for (int i = 0;i < sensor_fusion.size();i++) {
            float d = sensor_fusion[i][6];
            int check_lane = (int) (d/4.0);
            if ((check_lane) >= 0 && (check_lane <= 2)) {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_vel = sqrt(vx*vx + vy * vy);
              double check_car_s = sensor_fusion[i][5]; 
              check_car_s += ((double)prev_size*TIME_INC*check_vel);
              double check_dist = check_car_s - car_s;
              if ((check_lane == lane) && (check_dist > 0)) {
                if (front_dist > check_dist) {
                  front_dist = check_dist;
                  front_vel = check_vel;
                }
              }
              else if (check_lane == (lane + 1)) {
//              if (abs(check_dist) < 30) {
                if ((check_dist>-20)&&(check_dist<30)) {
                  right_dist = 0;
                  right_vel = 0;
                }
                else if ((right_dist > check_dist) && (check_dist > 0)) {
                  right_dist = check_dist;
                  right_vel = check_vel;
                }
              }
              else if (check_lane == (lane - 1)) {
                if (abs(check_dist) < 30) {
                  left_dist = 0;
                  left_vel = 0;
                }
                else if ((left_dist > check_dist) && (check_dist > 0)) {
                  left_dist = check_dist;
                  left_vel = check_vel;
                }
              }
            }
          }

//          std::cout<<"left_vel="<<left_vel<<" front_vel="<<front_vel<<" right_vel="<<right_vel<<std::endl;
//          std::cout<<"left_dist="<<left_dist<<" front_dist="<<front_dist<<" right_dist="<<right_dist<<std::endl;
          
          //behavior - adjust lane and velocity
          if (front_dist < 30) {
            if ((left_dist > 30) && (right_dist > 30)) {
              if ((right_vel > left_vel) && (right_vel > front_vel)) {
                lane++;
              }
              else if((left_vel >= right_vel) && (left_vel > front_vel)) {
                lane--;
              }
            }
            else if (left_dist > 30) {
              lane--;
            }
            else if (right_dist > 30) {
              lane++;
            }
          }
          
//          std::cout<<"lane="<<lane<<std::endl;
          if (!((lane>-1)&&(lane<3))) {
            lane=1;
          }

          /* commented out cause put speed adjustment in trajectory section
          if ((front_dist >= 30) && (ref_vel < (MAX_VEL - VEL_INC))) {
            ref_vel += VEL_INC;
          }
          else if (ref_vel > VEL_INC) {
            if (front_dist < 25) {
              ref_vel -= VEL_INC;
            }
          	else if (front_dist < 15) {
              ref_vel -= MAX_VEL_INC;
            }
          }
          */

          //trajectory
          //create spline
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
//          std::cout<<"ref_x="<<ref_x<<" ref_y="<<ref_y<<" ref_yaw="<<ref_yaw<<std::endl;
          
          double ref_x_prev, ref_y_prev;
          if (prev_size < 2) {
            ref_x_prev = car_x - cos(car_yaw);
            ref_y_prev = car_y - sin(car_yaw);
          } else {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            ref_x_prev = previous_path_x[prev_size - 2];
            ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
          }
          vector<double> ptsx;
          vector<double> ptsy;
          ptsx.push_back(ref_x_prev);
          ptsy.push_back(ref_y_prev);
          ptsx.push_back(ref_x);
          ptsy.push_back(ref_y);

          vector<double> test_s=getFrenet(ref_x, ref_y, ref_yaw,  map_waypoints_x, map_waypoints_y);
//          std::cout<<"test_s[0]="<<test_s[0]<<" test_s[1]="<<test_s[1]<<std::endl;
//          std::cout<<"car_s="<<car_s<<std::endl;
          
          vector<double> next_wp0 = getXY(car_s + 30.0, 2.0 + 4.0 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60.0, 2.0 + 4.0 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90.0, 2.0 + 4.0 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ptsx.push_back(next_wp0[0]);
          ptsy.push_back(next_wp0[1]);
          ptsx.push_back(next_wp1[0]);
          ptsy.push_back(next_wp1[1]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp2[1]);

/*
          std::cout<<"next_wp0[0]="<<next_wp0[0]<<std::endl;
          std::cout<<"next_wp1[0]="<<next_wp1[0]<<std::endl;
          std::cout<<"next_wp2[0]="<<next_wp2[0]<<std::endl;
          std::cout<<"next_wp0[1]="<<next_wp0[1]<<std::endl;
          std::cout<<"next_wp1[1]="<<next_wp1[1]<<std::endl;
          std::cout<<"next_wp2[1]="<<next_wp2[1]<<std::endl;          
          
          vector<double> test_xy=getFrenet(next_wp0[0],next_wp0[1], ref_yaw,  map_waypoints_x, map_waypoints_y);
          std::cout<<"test_xy[0]="<<test_xy[0]<<std::endl;
          std::cout<<"test_xy[1]="<<test_xy[1]<<std::endl;
*/

          //convert to vehicle coordinates for spline stability
          for (int i = 0;i < ptsx.size();i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = shift_x*cos(-ref_yaw) - shift_y * sin(-ref_yaw);
            ptsy[i] = shift_x*sin(-ref_yaw) + shift_y * cos(-ref_yaw);
          }

          // use spline to create trajectory for 30m ahead
          tk::spline s;
          s.set_points(ptsx, ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for (int i = 0;i < prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

//          std::cout<<"prev_size="<<prev_size<<std::endl;

          double x_inc = 0;
          double target_x,target_y,target_dist,N,new_vel_inc;
          for (int i = 1;i <= 50 - prev_size;i++) {
            // adjust velocity to avoid collision
            //adjust velocity by difference of velocities between car and car-in-front in order to match velocities
            //also adjust velocity by difference of distance in front from 27.5 (ideal distance to car in front) 
            if (front_dist<7.5) {
              new_vel_inc=-MAX_VEL_INC;
            }
            else if (front_dist<15) {
               new_vel_inc=-VEL_INC;
            }
            else {
              new_vel_inc=1.0*((front_vel-ref_vel)+(front_dist-27.5)/TIME_INC);
            }

            //make sure max/min velocities and accelerations are not exceeded
            if ( new_vel_inc>VEL_INC) {
              new_vel_inc=VEL_INC;
            }
            else if (( new_vel_inc<-VEL_INC) && (front_dist>7.5)) {
              new_vel_inc=-VEL_INC;
            }
            ref_vel += new_vel_inc;
            if (ref_vel > MAX_VEL) {
              ref_vel=MAX_VEL;
            }
            else if (ref_vel < VEL_INC ) {
              ref_vel=VEL_INC;
            }
  
            target_x = 30.0;
            target_y = s(target_x);
            target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
            // hypotenuse to approximate the spline
            N = target_dist/(TIME_INC*ref_vel);
            double x_point = x_inc + target_x / N;
            // double x_point = x_inc+TIME_INC*ref_vel*target_x/target_dist;
            double y_point = s(x_point);
            x_inc = x_point;
            double x_ref = x_point;
            double y_ref = y_point;

            //convert from vehicle to map coordinates
            x_point = (x_ref*cos(ref_yaw) - y_ref * sin(ref_yaw)) + ref_x;
            y_point = (x_ref*sin(ref_yaw) + y_ref * cos(ref_yaw)) + ref_y;

//            std::cout<<"x_point="<<x_point<<" y_point="<<y_point<<std::endl;

            // adjust front_dist with prediction of car positions
            front_dist+=TIME_INC*front_vel-x_inc;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

          }

/*
          vector<double> next_x_vals1;
          vector<double> next_y_vals1;
          double dist_inc=0.5;
          for(int i=0;i<100;i++) {
            double next_s=car_s+(i+1)*dist_inc;
            double next_d=6;
            vector<double> xy1 = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            next_x_vals1.push_back(xy1[0]);
            next_y_vals1.push_back(xy1[1]);
            next_x_vals.push_back(xy1[0]);
            next_y_vals.push_back(xy1[1]);
          }

          std::cout<<" next_x_vals1[0]="<<next_x_vals1[0]<<" next_y_vals1[0]="<<next_y_vals1[0]<<std::endl;
          std::cout<<" next_x_vals1[99]="<<next_x_vals1[99]<<" next_y_vals1[99]="<<next_y_vals1[99]<<std::endl;
          
          for (int i = 0;i < next_x_vals1.size();i++) {
            double shift_x = next_x_vals1[i] - ref_x;
            double shift_y = next_y_vals1[i] - ref_y;
            next_x_vals1[i] = shift_x*cos(0.0-ref_yaw) - shift_y * sin(0.0-ref_yaw); //TBCL: delete 0's
            next_y_vals1[i] = shift_x*sin(0.0-ref_yaw) + shift_y * cos(0.0-ref_yaw);
          }

          std::cout<<"t next_x_vals1[0]="<<next_x_vals1[0]<<" next_y_vals1[0]="<<next_y_vals1[0]<<std::endl;
          std::cout<<"t next_x_vals1[99]="<<next_x_vals1[99]<<" next_y_vals1[99]="<<next_y_vals1[99]<<std::endl;
          
          tk::spline s1;
          s1.set_points(next_x_vals1,next_y_vals1);
          target_x = 50.0;
          target_y = s1(target_x);
          target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
          for (int i = 0;i <100;i++) {
            double x_point = i*target_x/100;
            double y_point = s1(x_point);
            double x_ref = x_point;
            double y_ref = y_point;
            if (i==0) {
              std::cout<<"s x_point[0]="<<x_point<<" y_point[0]="<<y_point<<std::endl;
            } else if(i==99) {
              std::cout<<"s x_point[99]="<<x_point<<" y_point[99]="<<y_point<<std::endl;
            }
            
            x_point = (x_ref*cos(ref_yaw) - y_ref * sin(ref_yaw)) + ref_x;
            y_point = (x_ref*sin(ref_yaw) + y_ref * cos(ref_yaw)) + ref_y;          
          
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          std::cout<<"f next_x_vals[0]="<<next_x_vals[0]<<" next_y_vals[0]="<<next_y_vals[0]<<std::endl;
          std::cout<<"f next_x_vals[99]="<<next_x_vals[99]<<" next_y_vals[99]="<<next_y_vals[99]<<std::endl;
          
          std::cout <<"next_x_val size="<<next_x_vals.size()<<std::endl;
          */          
          
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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