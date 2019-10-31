#include "position.h"

position::position()
{
    CP = coutpos;
    scaleMapToTrack = MapToTrack;
    scaleTrackToMap = TrackToMap;
    size = mapsizepoints;
    intervall = robotprecision;
    counter = 0;
    messageRecievedPos = false;
}

void position::poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
    messageRecievedPos = true;
    for (int i = 0; i < _msg->pose_size(); i++)
    {
      if (_msg->pose(i).name() == "pioneer2dx")
      {
          posX = (_msg->pose(i).position().x());
          posY = (_msg->pose(i).position().y());
          posZ = (_msg->pose(i).position().z());
          oriW = _msg->pose(i).orientation().w();
          oriX = _msg->pose(i).orientation().x();
          oriY = _msg->pose(i).orientation().y();
          oriZ = _msg->pose(i).orientation().z();
          if(CP)
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
    ToEulerAngles();
}


void position::ToEulerAngles(void)
{
     // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (oriW * oriX + oriY * oriZ);
    double cosr_cosp = +1.0 - 2.0 * (oriX * oriX + oriY * oriY);
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (oriW * oriY - oriZ * oriX);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (oriW * oriZ + oriX * oriY);
    double cosy_cosp = +1.0 - 2.0 * (oriY * oriY + oriZ * oriZ);
    yaw = atan2(siny_cosp, cosy_cosp);

    //std::cout <<"roll: " << roll << " pitch: " << pitch << " yaw: " << yaw << std::endl;
}

std::vector<float> position::disNAngleToXY(float X,float Y)
{
    std::vector<float> output(2,0);

    float radius = 10;
    float xDir = posX+radius *  cos(yaw);
    float yDir = posY+radius *  sin(yaw);

    std::vector<float> currentDir = {xDir-posX,yDir-posY};
    std::vector<float> wantedDir = {X-posX,Y-posY};

    //find distance

    output[0] = lengthVector(wantedDir);

    //find angle
    output[1] = acos((currentDir[0]*wantedDir[0]+currentDir[1]*wantedDir[1])/(lengthVector(wantedDir)*lengthVector(currentDir)));

    if(wantedDir[1]<=0)
    {
        output[1] = output[1]*(-1);
    }
    return output;
}

float position::lengthVector(std::vector<float> vec)
{
    return sqrt(vec[0]*vec[0]+vec[1]*vec[1]);
}



void position::mouse_callback(int  event, int  x, int  y, int flags, void* userdata)
{
    position* Position = reinterpret_cast<position*>(userdata);

    Position->mouse_callback(event,x,y);
}

void position::mouse_callback(int  event, int  x, int  y)
{
    std::vector<float>temp(2,0);
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        // Store point coordinates
        temp[0]= x;
        temp[1] = y;


        std::cout <<"Number of points: "<< path.size()+1 <<" x: " << x << " y: " << y << std::endl;
        path.push_back(temp);
    }
}

void position::setPath(std::string pathToImage)
{
    cv::Point pt;
    cv::Mat img = cv::imread(pathToImage);
    cv::cvtColor(img,img,CV_BGR2GRAY);
    cv::Mat image(img.rows*size,img.cols*size, CV_8UC3, cv::Scalar(0,0, 100));

    image = enlargeImage(img,size);

    cv::namedWindow("Path");
    cv::imshow("Path", image);

    std::cout << "Enter 5 points: "<< std::endl;
    int numberOfPoints;

    numberOfPoints = 5;

    cv::setMouseCallback("Path",mouse_callback,this);

    while (int(path.size()) != numberOfPoints)
        {
            cv::waitKey(1);
        }

    cv::cvtColor(image,image,CV_GRAY2BGR);
    for (int i = 0; i < numberOfPoints; i++)
        {
        pt.x = path[i][0];
        pt.y = path[i][1];
        circle(image, pt,2,cv::Scalar(0,0,255),-1);
        circle(image, pt,10,cv::Scalar(255,0,0));
        }
    cv::imshow("Path", image);

    cv::waitKey(2);
    map = image;
}

std::vector<float> position::getPath()
{
    float pointXMap;
    float pointYMap;

        pointXMap =  (path[counter][0]-(map.cols)/2)*scaleMapToTrack/size;
        pointYMap = -(path[counter][1]-(map.rows)/2)*scaleMapToTrack/size;

   if(pointXMap+intervall>posX && pointXMap-intervall<posX && pointYMap+intervall>posY && pointYMap-intervall<posY)
   {
       counter++;
   }

   if(counter > 5)
   {
       pointXMap = 0;
       pointYMap = 0;
   }
   drawRobot();

   return {pointXMap,pointYMap};
}

cv::Mat position::enlargeImage(cv::Mat img,int size)
{

   cv::Mat image(img.rows*size,img.cols*size, CV_8UC3, cv::Scalar(0,0, 100));
   cv::cvtColor(image,image,CV_BGR2GRAY);
   uchar color = 0;

   for(int i = 0; i < img.rows;i++)
   {
       for(int k = 0; k < img.cols;k++)
       {
           color = img.at<uchar>(i,k);
           for(int j = 0; j < size; j++)
           {
               for(int l = 0; l < size; l++)
               {
                    image.at<uchar>(j+size*i,l+size*k) = color;
               }
           }
       }
   }
   return image;
}

void position::drawRobot()
{
    cv::Point pt;
    pt.x = posX*size*scaleTrackToMap +(map.cols)/2;
    pt.y = (-posY*size*scaleTrackToMap +(map.rows)/2);

    circle(map, pt,3,cv::Scalar(0,255,0),-1);
    cv::imshow("Path", map);
    cv::waitKey(2);
}
