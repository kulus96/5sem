#include "camera.h"

camera::camera()
{

}


void camera::cameraCallback(ConstImageStampedPtr &msg)
{
std::size_t width = msg->image().width();
std::size_t height = msg->image().height();
const char *data = msg->image().data().c_str();
cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

im = im.clone();
cv::cvtColor(im, im, CV_RGB2BGR);

cameramutex.lock();
cv::imshow("camera", im);
cameramutex.unlock();

}
