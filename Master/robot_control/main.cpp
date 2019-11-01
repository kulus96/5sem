#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include <fl/Headers.h>

#include <iostream>
#include "lidar.h"
#include "camera.h"
#include "fuzzyControl.h"
#include "position.h"
#include "particelfilter.h"

#include <vector>

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}

int main(int _argc, char **_argv) {


  position robot;
  lidar laser;
  camera view;
  fuzzyControl controller;
  //particelFilter position("FloorPlan/Bigworld.png",laser);


  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo topics
  gazebo::transport::SubscriberPtr statSubscriber =
      node->Subscribe("~/world_stats", statCallback);

  gazebo::transport::SubscriberPtr poseSubscriber =
      node->Subscribe("~/pose/info", &position::poseCallback, &robot);

  gazebo::transport::SubscriberPtr cameraSubscriber =
      node->Subscribe("~/pioneer2dx/camera/link/camera/image", &camera::cameraCallback,&view);

  gazebo::transport::SubscriberPtr lidarSubscriber =
      node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", &lidar::lidarCallback,&laser);

  // Publish to the robot vel_cmd topic
  gazebo::transport::PublisherPtr movementPublisher =
      node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

  // Publish a reset of the world
  gazebo::transport::PublisherPtr worldPublisher =
      node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
  gazebo::msgs::WorldControl controlMessage;
  controlMessage.mutable_reset()->set_all(true);
  worldPublisher->WaitForConnection();
  worldPublisher->Publish(controlMessage);

  while(!robot.messageRecievedPos && !view.messageRecievedCamera && !laser.messageRecievedLidar)
  {

  }

<<<<<<< HEAD
  std::vector<double> control;
  control.push_back(0);
  control.push_back(0);
  
  int T = TimStamp;
  std::vector<float> currPos(3,0);
  // Loop
  while (true)
  {  
    gazebo::common::Time::MSleep(T);
    cv::waitKey(1);
=======
  // Loop
    controller.init("obsAvoidance.fll","dirControl.fll");
    std::vector<double> control;
    control.push_back(0);
    control.push_back(0);
    robot.setPath("FloorPlan/Bigworld.png");
    std::vector<float> pointOfInterest;

  // Loop
  while (true)
  {  
    gazebo::common::Time::MSleep(10);
>>>>>>> ee34274a9fa558da96ea8278523a88ede6fcd9e7

    pointOfInterest = robot.getPath();

    std::cout << "dis: " << robot.disNAngleToXY(pointOfInterest[0],pointOfInterest[1])[0] << " angle: " << robot.disNAngleToXY(pointOfInterest[0],pointOfInterest[1])[1] << std::endl;

<<<<<<< HEAD
    //control = obsAvoidance.fuzzyController(laser.getShortestDistance(),laser.getAngleShortestDistance());

    //std::cout << "marbel loc: "<< view.posMarbel()<< " Speed: " << control[1] << " Dir: " << control[0] << std::endl;
=======
    control = controller.fuzzyController(laser.getShortestDistance(),laser.getAngleShortestDistance(),robot.disNAngleToXY(pointOfInterest[0],pointOfInterest[1])[1]);

    std::cout << "Angle: "<< robot.disNAngleToXY(pointOfInterest[0],pointOfInterest[1])[1] << " Speed: " << control[1] << " Dir: " << control[0] << std::endl;

    std::cout << "posX: "<< robot.posX << " posY: " << robot.posY << " wanted: " <<pointOfInterest[0] << ","<< pointOfInterest[1] << std::endl;
>>>>>>> ee34274a9fa558da96ea8278523a88ede6fcd9e7

    // Generate a pose
    ignition::math::Pose3d pose(double(control[1]), 0, 0, 0, 0, double(control[0]));

<<<<<<< HEAD
    //robot.positionModel(control[1],control[0],currPos);
    //robot.showPosition(currPos);
  

=======
>>>>>>> ee34274a9fa558da96ea8278523a88ede6fcd9e7
    // Convert to a pose message
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);
    movementPublisher->Publish(msg);
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
