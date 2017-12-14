#include <fstream>
#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "CarDriver.h"
#include "WorldMap.h"
#include <thread>

using namespace std;

// for convenience
using json = nlohmann::json;

// global variables
uWS::Hub gh;
thread gh_thread;
uWS::WebSocket<uWS::SERVER> gh_socket;

void SendDebugValues(const CarDriver &driver);

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(const string &s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of('[');
    auto b2 = s.find_first_of('}');
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for (int i = 0; i < maps_x.size(); i++) {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x, y, map_x, map_y);
        if (dist < closestLen) {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {

    int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y - y), (map_x - x));

    double angle = fabs(theta - heading);
    angle = min(2 * pi() - angle, angle);

    if (angle > pi() / 4) {
        closestWaypoint++;
        if (closestWaypoint == maps_x.size()) {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta) {
    WorldMap *map = WorldMap::GetInstance();
    int next_wp = NextWaypoint(x, y, theta, map->map_waypoints_x, map->map_waypoints_y);

    unsigned long prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0) {
        prev_wp = map->map_waypoints_x.size() - 1;
    }

    double n_x = map->map_waypoints_x[next_wp] - map->map_waypoints_x[prev_wp];
    double n_y = map->map_waypoints_y[next_wp] - map->map_waypoints_y[prev_wp];
    double x_x = x - map->map_waypoints_x[prev_wp];
    double x_y = y - map->map_waypoints_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - map->map_waypoints_x[prev_wp];
    double center_y = 2000 - map->map_waypoints_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef) {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++) {
        frenet_s += distance(map->map_waypoints_x[i], map->map_waypoints_y[i], map->map_waypoints_x[i + 1], map->map_waypoints_y[i + 1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};
}

#ifdef _DEBUG_DATA

void StartGraphThread() {
    gh.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Graph Hub Connected!!!" << std::endl;
        WorldMap *map = WorldMap::GetInstance();

        json msgJson;

        msgJson["type"] = "worldmap";
        msgJson["worldmap_x"] = map->map_waypoints_x;
        msgJson["worldmap_y"] = map->map_waypoints_y;

        auto msg = msgJson.dump();
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        cout << "Worldmap sent" << endl;
        gh_socket = ws;
    });


    gh.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        std::cout << "Graph Hub Disconnected" << std::endl;
    });

    if (gh.listen(4568)) {
        cout << "Listening on port 4568" << endl;

        gh_thread = thread([](){
            gh.run();
        });
    } else {
        cerr << "Could not start graph listener" << endl;
    }
}

string GetLogFilename() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X") << ".log";
    return ss.str();
}

#endif

int main() {
    uWS::Hub h;

    // Waypoint map to read from
    string map_file = "../data/highway_map.csv";

    WorldMap *map = WorldMap::GetInstance();
    if (!map->ReadMap(map_file)) {
        cerr << "Could not load map file" << endl;
        return -1;
    }

#ifdef _DEBUG_DATA
    StartGraphThread();
#endif

    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    CarDriver driver;
    driver.set_ideal_speed(MAX_ALLOWED_SPEED_MPH_);
    driver.set_desired_lane(1);

    h.onMessage([&driver](
            uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
            uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (!s.empty()) {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {

                    driver.UpdateModel(j[1]);
                    auto path = driver.GetPath();

#ifdef _DEBUG_DATA
                    SendDebugValues(driver);
#endif
                    // response to the simulator
                    json msgJson;
                    msgJson["next_x"] = path[0];
                    msgJson["next_y"] = path[1];

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";
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


#ifdef _DEBUG_DATA
void SendDebugValues(const CarDriver &driver) {// debug values
    // save all debug packages in a file
    ofstream output("output.json");
    if (output) {
        //json msg = driver.debug_packets_;
        json msg = *driver.last_debug_;
        output << setw(4) << msg << endl;
        output.close();
    }

    if (gh_socket.getPollHandle()) {
        json debug = *driver.last_debug_;

        auto debug_str = debug.dump();
        gh_socket.send(debug_str.data(), debug_str.length(), uWS::TEXT);
    }
}
#endif