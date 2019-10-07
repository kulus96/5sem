#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include <fl/Headers.h>

#include <iostream>
#include "lidar.h"
#include "camera.h"
#include "fuzzyControl.h"
#include <vector>


lidar laser;
camera view;
fuzzyControl bug;

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}

void poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();

  for (int i = 0; i < _msg->pose_size(); i++) {
    if (_msg->pose(i).name() == "pioneer2dx") {
/*
      std::cout << std::setprecision(2) << std::fixed << std::setw(6)
                << _msg->pose(i).position().x() << std::setw(6)
                << _msg->pose(i).position().y() << std::setw(6)
                << _msg->pose(i).position().z() << std::setw(6)
                << _msg->pose(i).orientation().w() << std::setw(6)
                << _msg->pose(i).orientation().x() << std::setw(6)
                << _msg->pose(i).orientation().y() << std::setw(6)
                << _msg->pose(i).orientation().z() << std::endl;*/
    }
  }

}

void wrapperCameraCallback(ConstImageStampedPtr &msg)
{
  view.cameraCallback(msg,1);
}

void wrapperlidarCallback(ConstLaserScanStampedPtr &msg)
{
  laser.lidarCallback(msg,1);
}


int main(int _argc, char **_argv) {

  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo topics
  gazebo::transport::SubscriberPtr statSubscriber =
      node->Subscribe("~/world_stats", statCallback);

  gazebo::transport::SubscriberPtr poseSubscriber =
      node->Subscribe("~/pose/info", poseCallback);

  gazebo::transport::SubscriberPtr cameraSubscriber =
      node->Subscribe("~/pioneer2dx/camera/link/camera/image", wrapperCameraCallback);

  gazebo::transport::SubscriberPtr lidarSubscriber =
      node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", wrapperlidarCallback);

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

  //float speed = 0.;
  //float dir = 0.0;

  // Loop
    bug.init();

  std::vector<double> control;

  control.push_back(0);
  control.push_back(0);
  while (true)
  {  
    gazebo::common::Time::MSleep(10);
    cv::waitKey(1);

    std::cout << "Distance: " << laser.getShortestDistance() << " Angle: " << laser.getAngleShortestDistance() << std::endl;

    control = bug.fuzzyController(laser.getShortestDistance(),laser.getAngleShortestDistance());

    std::cout << "Speed: " << control[1] << " Dir: " << control[0] << std::endl;
    //control[1] = 0.2;
    // Generate a pose
    ignition::math::Pose3d pose(double(control[1]), 0, 0, 0, 0, double(control[0]));



    // Convert to a pose message
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);
    movementPublisher->Publish(msg);
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
