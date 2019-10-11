#include "position.h"

position::position()
{
    CP = coutpos;
}

void position::poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  if(CP)
  {

      for (int i = 0; i < _msg->pose_size(); i++)
      {
        if (_msg->pose(i).name() == "pioneer2dx")
        {

          std::cout << std::setprecision(2) << std::fixed << std::setw(6)
                    << _msg->pose(i).position().x() << std::setw(6)
                    << _msg->pose(i).position().y() << std::setw(6)
                    << _msg->pose(i).position().z() << std::setw(6)
                    << _msg->pose(i).orientation().w() << std::setw(6)
                    << _msg->pose(i).orientation().x() << std::setw(6)
                    << _msg->pose(i).orientation().y() << std::setw(6)
                    << _msg->pose(i).orientation().z() << std::endl;
        }
      }
  }

}

