#include "lidar.h"

LIDAR::LIDAR()
{

}


void LIDAR::lidarCallback(ConstLaserScanStampedPtr &msg)
{
  mutex.lock();

  std:: cout << " here" << std:: endl;
  float angle_min = float(msg->scan().angle_min());
  float angle_increment = float(msg->scan().angle_step());

  float range_min = float(msg->scan().range_min());
  float range_max = float(msg->scan().range_max());
         
  int nranges = msg->scan().ranges_size();
  int nintensities = msg->scan().intensities_size();
  
  
  assert(nranges == nintensities);
  
  float currentAngle = angle_min;
  int i = 0;

  for (auto r : nranges)
  {
      float distance = float(msg->scan().ranges(i++));
      if (distance > range_max)
      {
          distance = range_max;
      }
      r.distane = distance;
      r.angle = currentAngle;

      currentAngle += angle_increment;
  }
  
  mutex.unlock();
}

