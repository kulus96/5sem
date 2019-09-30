#include "lidar.h"

lidar::lidar()
{

}

void lidar::lidarCallback(ConstLaserScanStampedPtr &msg)
{
  lidarmutex.lock();
  int nranges = msg->scan().ranges_size();
    if(runs == 0)
    {
        runs = 1;
        laser test ;
        test.distance = 0;
        test.angle = 0;

         for(int k = 0; k < nranges; k++)
         {
            Laser.push_back(test);
         }
    }



  float angle_min = float(msg->scan().angle_min());
  float angle_increment = float(msg->scan().angle_step());

  float range_max = float(msg->scan().range_max());
         
  //int nintensities = msg->scan().intensities_size();
  
  
  //assert(nranges == nintensities);
  
  float currentAngle = angle_min;
  float tempShortestDist = 10;
  float tempAngleShortestDist = 0;
  int k = 0;

  for (int i = 0; i< nranges;i++)
  {
      float distance = float(msg->scan().ranges(k++));
      if (distance > range_max)
      {
          distance = range_max;
      }
      Laser[i].distance = distance;
      Laser[i].angle = currentAngle;

      if(distance<tempShortestDist)
      {
          tempShortestDist = distance;
          tempAngleShortestDist = currentAngle;
      }

      currentAngle += angle_increment;
  }
  shortestDistance = tempShortestDist;;
  angleShortestDistance = tempAngleShortestDist;
  
  lidarmutex.unlock();
}

void lidar::coutlidar()
{

    for(int i = 0; i<Laser.size();i++)
    {
        std::cout << "i: " << i << " Angle: " << Laser[i].angle << " Distance: "<< Laser[i].distance << std::endl;
    }
    runs = 2;

}

float lidar::getShortestDistance()
{
    return shortestDistance;
}

float lidar::getAngleShortestDistance()
{
    return angleShortestDistance;
}
