#include "camera.h"

camera::camera()
{
    SC = showcamera;
}


void camera::cameraCallback(ConstImageStampedPtr &msg)
{
    width = msg->image().width();
    height = msg->image().height();
    const char *data = msg->image().data().c_str();
    cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

    Mat imGray;

    im = im.clone();
    cv::cvtColor(im, imGray, CV_RGB2GRAY);

    cv::medianBlur(imGray,imGray,11);

    //Apply thresholding


    for(int i = 0; i < imGray.rows; i++)
    {
        for(int k = 0; k < imGray.cols; k++)
        {
            if(imGray.at<uchar>(i,k) >88&& imGray.at<uchar>(i,k) < 100)
            {
               imGray.at<uchar>(i,k) = 0;
            }
            else
            {
                imGray.at<uchar>(i,k) = 255;
            }
        }
    }
    cv::threshold(imGray, imGray, 100, 255, cv::THRESH_BINARY);

    cv::dilate(imGray,imGray,Mat());
    cv::erode(imGray,imGray,Mat());
    cv::erode(imGray,imGray,Mat());

      /// Apply the Hough Transform to find the circles
      HoughCircles( imGray, circles, CV_HOUGH_GRADIENT, 2,80, 100, 20, 0, 60 );
     if(SC)
     {
         /// Draw the circles detected
          for( size_t i = 0; i < circles.size(); i++ )
          {

              Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
              int radius = cvRound(circles[i][2]);
              // circle center
              circle( im, center, 3, Scalar(0,255,0), -1, 8, 0 );
              // circle outline
              circle( im, center, radius, Scalar(0,0,255), 3, 8, 0 );
           }
        //cv::imshow("gray", imGray);
        mutex.lock();
        cv::imshow("camera", im);
        //std::cout << "Hello" << std::endl;
        mutex.unlock();
     }
    cv::waitKey(1);
}

void camera::showHistogram(std::string const& name, cv::Mat1b const& image)
{

    std::string path = "Images/";
    path.append(name);
    path.append(".PNG");
    std::cout << path << std::endl;
    // Set histogram bins count
    int bins = 256;
    int histSize[] = {bins};
    // Set ranges for histogram bins
    float lranges[] = {0, 256};
    const float* ranges[] = {lranges};
    // create matrix for histogram
    cv::Mat hist;
    int channels[] = {0};

    // create matrix for histogram visualization
    int const hist_height = 256;
    cv::Mat3b hist_image = cv::Mat3b::zeros(hist_height, bins);

    cv::calcHist(&image, 1, channels, cv::Mat(), hist, 1, histSize, ranges, true, false);

    double max_val=0;
    minMaxLoc(hist, 0, &max_val);

    // visualize each bin
    for(int b = 0; b < bins; b++) {
        float const binVal = hist.at<float>(b);
        int   const height = cvRound(binVal*hist_height/max_val);
        cv::line
            ( hist_image
            , cv::Point(b, hist_height-height), cv::Point(b, hist_height)
            , cv::Scalar::all(255)
            );
    }
    cv::imwrite(path,hist_image);
    cv::imshow(name, hist_image);
    cv::waitKey(1);
}


float camera::posMarbel()
{
    if(circles.size())
    {
        int largestRadius = 0;
        int saveIndex = 0;

        for( size_t i = 0; i < circles.size(); i++ )
        {
            if(cvRound(circles[i][2])>largestRadius)
            {
                  largestRadius = cvRound(circles[i][2]);
                  saveIndex = i;
            }
        }
        float halfwidth = float(width)/2;

        return float (((cvRound(circles[saveIndex][0]))-halfwidth) *(1.046667/halfwidth) ) ;

    }
    return 0;
}

bool camera::marbelLocated()
{
    if(circles.size())
    {
        return 1;
    }
    else
    {
        return 0;

    }
}



