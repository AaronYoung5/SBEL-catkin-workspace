#include <algorithm>
#include <boost/asio.hpp>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <unistd.h>

#include "auto_custom_driver/DriverCodes.h"
#include "auto_custom_driver/DriverMessages.pb.h"
#include "auto_custom_driver/SegmentFollower.h"

#include "ros/ros.h"

// TODO: If we want to send different message types, we nee to add a byte at the
// beginning of the buffer to describe what type it is. I couldn't get to this
// but I can finish later

// data read from chrono
double lat = 0;
double lon = 0;
double alt = 0;
double yaw = 0;
std::vector<double> lidarData;
double prevTime = 0;
double currTime = 0;
double lightXPos = 0;
double lightYPos = 0;
int lightXDir = 0;
int lightYDir = 0;
bool shouldStop = false;
double lightOffset = 8.0;

// data to feed back to chrono
double steering;
double throttle;
double braking;

// translated data
double currX = 0;
double currY = 0;
double latRef = 43.070985;
double lonRef = -89.400285;
double altRef = 263.2339;
double currVelX = 0;
double currVelY = 0;
double currSpeed = 0;
double currXdir = 0;
double currYdir = 0;

// vehicle parameters
const double earthRadius = 6371000.0;
const double lidarMinAng = -1.5;
const double lidarMaxAng = 1.5;
const int lidarSamples = 100;
const double safeWidth = 2.0;
const double targetSpeed = 10.0; // m/s
const double stopDistance = 6.0;

// waypoints and pertaining information
std::vector<std::pair<double, double>> waypoints;
int currWayIndex = 0;
double lookAheadScale = 1.2; // look ahead multiplier for speed
double waypointDist =
    3.0; // distance to waypoint before switching to next waypoint
double pathP = -1.0;
double throttleP = 0;
double brakingP = 0;

const double radTodeg = 180.0 / 3.1415927;

// traffic light parameters
const double intersectionDist = 7.0;
const double intersectionDist2 = 15.0;
const double intersectionDist3 = 30.0;

// this function will check to see if the traffic light will allow the vehicle
// to go
void checkTrafficLight() {
  // vehicles that are stopping should stop some distance from the light

  double newbraking = 0;
  double newthrottle = 1;
  // check x zone yellow
  if (!shouldStop && lightXDir == 0 && currX > lightXPos + intersectionDist2 &&
      currX < lightXPos + intersectionDist3) {
    if (currY < lightYPos + 2 * intersectionDist2 &&
        currY > lightYPos - 2 * intersectionDist2) {
      newbraking = .75 * (1 - (currX - (lightXPos + intersectionDist)) /
                                  (intersectionDist3 - intersectionDist));
      newthrottle = 0;
      shouldStop = true;
    }
  }
  // check x zone red
  else if (lightXDir == -1 && currX > lightXPos + intersectionDist &&
           currX < lightXPos + intersectionDist3) {
    if (currY < lightYPos + 2 * intersectionDist2 &&
        currY > lightYPos - 2 * intersectionDist2) {
      newbraking = .75 * (1 - (currX - (lightXPos + intersectionDist)) /
                                  (intersectionDist3 - intersectionDist));
      newthrottle = 0;
    }
  }

  // check y zone yellow
  else if (!shouldStop && lightYDir == 0 &&
           currY < lightYPos - intersectionDist2 &&
           currY > lightYPos - intersectionDist3) {
    if (currX > lightXPos - 2 * intersectionDist2 &&
        currX < lightXPos + 2 * intersectionDist2) {
      newbraking = .75 * (1 - ((lightYPos - intersectionDist) - currY) /
                                  (intersectionDist3 - intersectionDist));
      newthrottle = 0;
    }
  }
  // check y zone red
  else if (lightYDir == -1 && currY < lightYPos - intersectionDist &&
           currY > lightYPos - intersectionDist3) {
    if (currX > lightXPos - 2 * intersectionDist2 &&
        currX < lightXPos + 2 * intersectionDist2) {
      newbraking = .75 * (1 - ((lightYPos - intersectionDist) - currY) /
                                  (intersectionDist3 - intersectionDist));
      newthrottle = 0;
    }
  }

  else {
    shouldStop = false;
  }

  throttle = std::min(throttle, newthrottle);
  braking = std::max(braking, newbraking);

  // std::cout<<"Light Throttle: "<<throttle<<", Light Braking:
  // "<<braking<<std::endl;
}

// This function will check that the vehicle has a safe path to drive
// and adjust throttle and braking accordingly
void checkCollisions() {
  double safeSpeed = targetSpeed;
  double minDistance = 50.0;
  double sideMinDistance = 50.0;

  // checks only distances in a rectangle in front of the vehicle
  // TODO: angle that rectangle based on wheel angle
  for (int i = 0; i < lidarData.size(); i++) {
    double angle =
        (i - lidarSamples / 2.0) * lidarMaxAng / (lidarSamples / 2.0);
    if (lidarData[i] * cos(angle) < minDistance &&
        abs(lidarData[i] * sin(angle)) < safeWidth / 2.0) {
      minDistance = lidarData[i] * cos(angle);
    }
  }

  safeSpeed = (-targetSpeed / (stopDistance - 50.0)) * (minDistance - 50.0) +
              targetSpeed;
  double error = (safeSpeed - currSpeed);
  if (error > 0) {
    throttle =
        error / 13.0 + 0.05; // slight offset to remove steady state error
    braking = 0;
  } else if (error < 0) {
    throttle = 0;
    braking = abs(error / 13.0) * 2.0;
  }

  if (minDistance < stopDistance)
    braking = .75;

  // std::cout<<"Collision Throttle: "<<throttle<<", Collision Braking:
  // "<<braking<<std::endl; std::cout<<"Min Distance: "<<minDistance<<std::endl;
}

// undo the cartesian to gps conversion and calculate velocity in x-y plane
void updateSpeedPos() {
  double dT = currTime - prevTime;
  double tempX =
      ((lon - lonRef) / radTodeg) * earthRadius * cos(lat / radTodeg);
  double tempY = ((lat - latRef) / radTodeg) * earthRadius;
  double dX = tempX - currX;
  double dY = tempY - currY;

  if (dT > 0.01) {
    currVelX = dX / dT;
    currVelY = dY / dT;

    currSpeed = sqrt(currVelX * currVelX + currVelY * currVelY);

    prevTime = currTime;
    currX = tempX;
    currY = tempY;
    if (currSpeed > 0.25) {
      currXdir = currVelX;
      currYdir = currVelY;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc,argv,"auto_custom_driver");
  // std::cout<<"HAS THIS BEEN RECOMPILED?"<<std::endl;

  // std::cout << "Dummy driver is running." << std::endl;
  if (argc != 3) {
    std::cout << "Usage: " << std::string(argv[0])
              << " <Port Number> <path to waypoint file>" << std::endl;
  }

  boost::asio::ip::udp::socket socket(
      *(new boost::asio::io_service),
      boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(),
                                     std::atoi(argv[1])));

  // create the steering controller
  auto follower = std::make_shared<SegmentFollower>(10.0, 0.12, 0.002,
  0.000);

  // Add gps waypoints from input file to a vector
  // assuming CSV file and columns are long, lat with nothing else
  /*std::string dataFile = "../../data/cave_data/Paths/path1.csv";*/
  std::string dataFile = argv[2];

  std::ifstream file(dataFile);
  std::string line;

  while (std::getline(file, line)) {
    std::vector<std::string> parts;
    std::stringstream linestring(line);
    std::string val;

    while (std::getline(linestring, val, ',')) {
      parts.push_back(val);
    }
    std::stringstream convertLon(parts[0]);
    std::stringstream convertLat(parts[1]);

    double tempLon;
    double tempLat;

    convertLon >> tempLon;
    convertLat >> tempLat;

    std::cout << std::setprecision(10) << "LonGPSRead: " << tempLon - lonRef
              << ", LonGPSRead: " << tempLat - latRef << std::endl;
    // convert gps to cartesian before pushing to vector
    tempLon =
        ((tempLon - lonRef) / radTodeg) * earthRadius * cos(tempLat /
        radTodeg);
    tempLat = ((tempLat - latRef) / radTodeg) * earthRadius;

    std::pair<double, double> temp(tempLon, tempLat);
    waypoints.push_back(temp);
  }
  std::cout << "-----Waypoints as they are in the waypoints list---------"
            << std::endl;
  for (int i = 0; i < waypoints.size(); i++) {
    std::cout << std::setprecision(10) << "Lon: " << waypoints[i].first
              << ", Lat: " << waypoints[i].second << std::endl;
  }
  /*return -1;*/
  // give the follower the x,y cartesian waypoints
  follower->Initialize(waypoints);

  while (true) {
    // This is a bit of a tricky process, but it lets us send messages of
    // whatever length we want without having to send the size with each
    // message.
    // Let the UDP stack fill up
    socket.receive(boost::asio::null_buffers(), 0);
    // Check the size of the waiting buffer
    int available = socket.available();
    // Allocate enough space
    std::vector<uint8_t> buffer(available);
    // Receive and record the actual size of the received packet
    boost::asio::ip::udp::endpoint simEndpoint;
    int received = socket.receive_from(
        boost::asio::buffer(buffer.data(), available), simEndpoint);
    // socket.receive_from(boost::asio::buffer(buffer.data(),
    // available),simEndpoint)
    std::cout << buffer.data()[0] << std::endl;
    if (buffer.data()[0] == LIDAR_MESSAGE) {
      std::cout << "LIDAR_MESSAGE RECIEVED" << std::endl;
      // Parse from what we ended up receiving
      DriverMessages::lidar message;
      message.ParseFromArray(buffer.data() + 1, received - 1);
      // message.PrintDebugString();
      lidarData = {};
      for (int i = 0; i < message.data_size(); i++) {
        lidarData.push_back(message.data(i));
      }

    } else if (buffer.data()[0] == GPS_MESSAGE) {
      std::cout << "GPS_MESSAGE RECIEVED" << std::endl;
      // Parse from what we ended up receiving
      DriverMessages::gps gpsMessage;
      gpsMessage.ParseFromArray(buffer.data() + 1, received - 1);
      // gpsMessage.PrintDebugString();
      lon = gpsMessage.longitude();
      lat = gpsMessage.latitude();
      alt = gpsMessage.altitude();
      // std::cout<<std::setprecision(10)<<"LONGITUDE: "<<lon<<"LATITUDE:
      // "<<lat<<std::endl; std::cout<<"AM I RECOMPILING?"<<std::endl;
    } else if (buffer.data()[0] == IMU_MESSAGE) {
      std::cout << "IMU_MESSAGE RECIEVED" << std::endl;
      // Parse from what we ended up receiving
      DriverMessages::imu imuMessage;
      imuMessage.ParseFromArray(buffer.data() + 1, received - 1);
      yaw = imuMessage.q2();
      // imuMessage.PrintDebugString();
      // std::cout<<"AM I RECOMPILING?"<<std::endl;
    } else if (buffer.data()[0] == TIME_MESSAGE) {
      std::cout << "TIME_MESSAGE RECIEVED" << std::endl;
      // Parse from what we ended up receiving
      DriverMessages::time timeMessage;
      timeMessage.ParseFromArray(buffer.data() + 1, received - 1);
      // timeMessage.PrintDebugString();
      currTime = timeMessage.t();
      // std::cout<<"TIME: "<<currTime<<std::endl;
      // std::cout<<"AM I RECOMPILING?"<<std::endl;
    } else if (buffer.data()[0] == LIGHT_MESSAGE) {
      std::cout << "LIGHT_MESSAGE RECIEVED" << std::endl;
      // Parse from what we ended up receiving
      DriverMessages::light lightMessage;
      lightMessage.ParseFromArray(buffer.data() + 1, received - 1);
      // lightMessage.PrintDebugString();

      lightXPos = ((lightMessage.xpos() - lonRef) / radTodeg) * earthRadius *
                      cos(lightMessage.xpos() / radTodeg) -
                  lightOffset;
      lightYPos = ((lightMessage.ypos() - latRef) / radTodeg) * earthRadius;

      lightXDir = lightMessage.xdir();
      lightYDir = lightMessage.ydir();

      // std::cout<<"TIME: "<<currTime<<std::endl;
      // std::cout<<"AM I RECOMPILING?"<<std::endl;
    }

    // update our known position, velocity, and time
    updateSpeedPos();
    // check that we are not about to hit anything
    checkCollisions();
    // check that we are staying on the path (road)
    // checkRoad();
    follower->Advance({currX, currY}, yaw, currTime);
    steering = follower->GetSteering();

    checkTrafficLight();

    if (currTime < 1.0) {
      steering = 0;
      throttle = 0;
      braking = 0;
    }

    // std::cout<<"Current GPS Loc: "<<lon<<", "<<lat<<std::endl;
    // std::cout<<"Throttle: "<<throttle<<std::endl;
    // std::cout<<"Steering: "<<steering<<std::endl;
    // std::cout<<"Braking: "<<braking<<std::endl;

    // package and send the control message
    DriverMessages::control controlMessage;
    controlMessage.set_throttle(throttle);
    controlMessage.set_steering(steering);
    controlMessage.set_braking(braking);

    int32_t sizeControl = controlMessage.ByteSize();
    std::vector<uint8_t> bufferControl(sizeControl + 1);
    bufferControl.data()[0] = CONTROL_MESSAGE;
    controlMessage.SerializeToArray(bufferControl.data() + 1, sizeControl);
    socket.send_to(boost::asio::buffer(bufferControl.data(), sizeControl +
    1),
                   simEndpoint);
  }
}
