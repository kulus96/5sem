#include "lidar.h"

lidar::lidar()
{
    SL = showlidar;
}

void lidar::lidarCallback(ConstLaserScanStampedPtr &msg)
{

    if(runs == 0)
    {
        nranges = msg->scan().ranges_size();
        runs = 1;
        laser test ;
        test.distance = 0;
        test.angle = 0;

         for(int k = 0; k < nranges; k++)
         {
            Laser.push_back(test);
         }
         angle_min = float(msg->scan().angle_min());
         angle_increment = float(msg->scan().angle_step());

         range_max = float(msg->scan().range_max());
         range_min = float(msg->scan().range_min());

         nintensities = msg->scan().intensities_size();
         sec = msg->time().sec();
         nsec = msg->time().nsec();

    }

  assert(nranges == nintensities);
  
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

  if (SL)
  {
      showLidar();

  }

}

float lidar::getShortestDistance()
{
    return shortestDistance;
}

float lidar::getAngleShortestDistance()
{
    return angleShortestDistance;
}

void lidar::showLidar()
{
    assert(nranges == nintensities);
    float px_per_m = 200 / range_max;
    int width = 400;
    int height = 400;
    cv::Mat im(height, width, CV_8UC3);
    im.setTo(0);
    for (int i = 0; i < nranges; i++) {
      float angle = Laser[i].angle;
      float range = std::min(Laser[i].distance,range_max);

      cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                          200.5f - range_min * px_per_m * std::sin(angle));
      cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                        200.5f - range * px_per_m * std::sin(angle));
      cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
               16, 4);

    }
   cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
   cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
                cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
                cv::Scalar(255, 0, 0));
    mutex.lock();
        cv::imshow("lidar", im);
    mutex.unlock();
    cv::waitKey(1);
}


float lidar::objInFront(float range)
{
    float tempShortesDistance = 10;
    for(int i= 0; i < int(Laser.size()); i++)
    {
        if(abs(Laser[i].angle)< range)
        {
            if(Laser[i].distance <tempShortesDistance)
            {
                tempShortesDistance = Laser[i].distance;
            }
        }
    }
    return tempShortesDistance;
}
