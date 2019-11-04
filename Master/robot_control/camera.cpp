#include "camera.h"

camera::camera()
{
    SC = showcamera;
    messageRecievedCamera = false;
}

namespace
{
    // windows and trackbars name
    const std::string windowName = "Hough Circle Detection Demo";
    const std::string cannyThresholdTrackbarName = "Lower threshold";
    const std::string accumulatorThresholdTrackbarName = "Upper threshold";

    // initial and max values of the parameters of interests.
    /*const int cannyThresholdInitialValue = 100;
    const int accumulatorThresholdInitialValue = 50;
    const int maxAccumulatorThreshold = 200;
    const int maxCannyThreshold = 255;
    */
    const int lowerinit = 0;
    const int upperinit = 200;
    const int maxlower = 255;
    const int maxupper = 255;

    void HoughDetection(const Mat& src_gray, const Mat& src_display,int lower, int upper)// int cannyThreshold, int accumulatorThreshold)
    {
        /*// will hold the results of the detection
        std::vector<Vec3f> circles;
        // runs the actual detection
        HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, cannyThreshold, accumulatorThreshold, 0, 0 );

        // clone the colour, input image for displaying purposes
        Mat display = src_display.clone();
        for( size_t i = 0; i < circles.size(); i++ )
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            circle( display, center, 3, Scalar(0,255,0), -1, 8, 0 );
            // circle outline
            circle( display, center, radius, Scalar(0,0,255), 3, 8, 0 );
        }*/
        Mat display = src_gray.clone();
    for(int i = 0; i < src_gray.rows; i++)
    {
        for(int k = 0; k < src_gray.cols; k++)
        {
            if(src_gray.at<uchar>(i,k) >lower& src_gray.at<uchar>(i,k) < upper)//88&& imGray.at<uchar>(i,k) < 100)
            {
               display.at<uchar>(i,k) = 255;
            }
            else
            {
                display.at<uchar>(i,k) = 0;
            }
        }
    }
    for(int i = 0; i<3;i++)
    {
        cv::dilate(display,display,Mat());
    }
    for(int i = 0; i<7;i++)
    {
        cv::erode(display,display,Mat());
    }
        // shows the results
        imshow( windowName, display);
    }
}
void camera::cameraCallback(ConstImageStampedPtr &msg)
{
    messageRecievedCamera = true;

    width = msg->image().width();
    height = msg->image().height();
    const char *data = msg->image().data().c_str();
    cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

    Mat imGray;
    Mat imBin = im.clone();

    im = im.clone();
    
    cv::cvtColor(im, imGray, CV_RGB2GRAY);

    cv::medianBlur(imGray,imGray,1);
    int lower = lowerinit;
    int upper = upperinit;
    //Apply thresholding
    for(int i = 0; i < imGray.rows; i++)
    {
        for(int k = 0; k < imGray.cols; k++)
        {
            if(imGray.at<uchar>(i,k) >98& imGray.at<uchar>(i,k) < 113)//88&& imGray.at<uchar>(i,k) < 100)
            {
               imGray.at<uchar>(i,k) = 255;
            }
            else
            {
                imGray.at<uchar>(i,k) = 0;
            }
        }
    }
    
    //cv::threshold(imGray, imGray, 100, 255, cv::THRESH_BINARY);
 

   
    /*namedWindow( windowName, WINDOW_AUTOSIZE );
    createTrackbar(cannyThresholdTrackbarName, windowName, &lower,maxlower);
    createTrackbar(accumulatorThresholdTrackbarName, windowName, &upper, maxupper);*/

    adaptiveThreshold(imGray,imGray,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,5,0);

    //char key = 0;
    /*while(key != 'q' && key != 'Q')
    {
        // those parameters cannot be =0
        // so we must check here
        lower = std::max(lower, 1);
        upper = std::max(upper, 1);

        //runs the detection, and update the display
        HoughDetection(imGray, imGray, lower, upper);
        //std::cout << cannyThreshold << std::endl;
        // get user key
        key = (char)waitKey(2);
    }*/
    for(int i = 0; i<3;i++)
    {
        cv::dilate(imGray,imGray,Mat());
    }
    for(int i = 0; i<4;i++)
    {
        cv::erode(imGray,imGray,Mat());
    }
    namedWindow("with yest",1);
    imshow("with yest",imGray);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;


    findContours(imGray,// source image
    contours, //Detected contours
    hierarchy, //*optional. Contains the image topology
    CV_RETR_TREE, // contour retrieval mode. Contours organized in two level hierarchy
    CV_CHAIN_APPROX_SIMPLE); //method of contour approximation. 

    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );

    for( int i = 0; i < contours.size(); i++ )
    { 
        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        //minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
    }

    for(int idx=0;idx < contours.size();idx++)
    {
        if(idx==0)
        {
            
            Scalar color(0, 0,0);
            drawContours(imGray,contours,idx,color,CV_FILLED,8,hierarchy);
            
        }
        else
        {
            Scalar color(255, 255, 255);
            
            drawContours(imGray,contours,idx,color,CV_FILLED,8,hierarchy);
            //rectangle( im, boundRect[idx].tl(), boundRect[idx].br(), Scalar(0,255,0), 2, 8, 0 );
        }
    }

    namedWindow("with yest2",1);
    imshow("with yest2",imGray);

    RotatedRect rectA;
    vector<Point2f> rect_points(4);

    for(int i = 0; i < contours.size();i++)
    {
        int rectArea = boundRect[i].width*boundRect[i].height;
        rectA = minAreaRect(contours[i]);
        int minArea = rectA.size.width * rectA.size.height;
        //int rotatedArea = rectA.points*rectA[1][1];
        std::cout << boundRect[i].height << " , " << boundRect[i].width << std::endl;

        if(contourArea(contours[i]) < minArea*0.70 && minArea > 100)//rectArea*0.8)
        {
            if(boundRect[i].height >= boundRect[i].width)
            {
                rectangle( im, boundRect[i].tl(), boundRect[i].br(), Scalar(0,255,0), 2, 8, 0 );
            }
        }
    }
      /// Apply the Hough Transform to find the circles
    
    /*HoughCircles( imGray, circles, CV_HOUGH_GRADIENT, 2,80, 23, 21, 0, 0 );

    /// Draw the circles detected
    
    for( size_t i = 0; i < circles.size(); i++ )
    {

        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        
        if(debouncePos(center))
        {
            int radius = cvRound(circles[i][2]);
            // circle center
            circle( im, center, 3, Scalar(0,255,0), -1, 8, 0 );
            // circle outline
            circle( im, center, radius, Scalar(0,0,255), 3, 8, 0 );
            
        }
        else
        {
            break;
        }
    
    }

    //cv::imshow("gray", imGray);*/
    
    
    mutex.lock();
    cv::imshow("camera", im);
    //std::cout << "Hello" << std::endl;
    mutex.unlock();
    waitKey(1);
    
}



void camera::CannyDetect(Mat &img, Mat &output)
{
    Mat src_gray;
    Mat detected_edges;
    int lowThreshold = 18;
    int ratio = 3;
    int kernel_size=3;
    cvtColor(img,src_gray,COLOR_BGR2GRAY);

    blur(src_gray,detected_edges,Size(3,3));
    Canny(detected_edges,detected_edges,lowThreshold,lowThreshold*ratio,kernel_size);
    output = Scalar::all(0);
    img.copyTo(output,detected_edges);
}

bool camera::debouncePos(Point q)
{   
    if(q.y==pos.y)
    {
        pos = q;
        debounceCounter++;
    }
    else
    {
        pos=q;
        debounceCounter = 0;
        return false;
    }
    
    if(debounceCounter>3)
    {

        debounceCounter = 0;
        
        return true;
    }
    

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

