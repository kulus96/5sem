#include "particelfilter.h"

particelFilter::particelFilter()
{

}

particelFilter::particelFilter(std::string path,lidar& laser)
{
    cv::Mat img = cv::imread(path,CV_LOAD_IMAGE_GRAYSCALE);
    if( !img.data )
    {
        std::cout << "Map was not loaded" << std::endl;
    }
    sizeofMapComparedtoOriginal = mapsize;
    map = enlargeImage(img,sizeofMapComparedtoOriginal);
    generateDistanceFromEachPosistion();

    *sensor = laser;

    NP = numberofParticels;
    STD = standartdeviation;
    lidarIncrement = sensor->getLidarIncrement();
    numberOfLaserReadings = sensor->getNumberofLidarReadings();
    indexIntervall = int((sensor->getNumberofLidarReadings()/numberOfGeneratedLaserReadings)+0.5);
    sensorRange = sensor->getLidarRangeMax();

    particelData temp;
    temp.posx = 0;
    temp.posy = 0;
    temp.ori = 0;
    temp.weight = 0.1;

    std::vector<particelData> M(NP,temp);

    std::vector<std::vector<particelData>> N(map.cols,std::vector<particelData>(map.rows,temp));
}


void particelFilter::findPos(float dir,float velo)
{
    if(sensor->lidarReady())
    {


        //Prediction
            //The model for the robot motion to estimate next state/pos
    int sumofWeights = 0;
    for(int i = 0; i < NP; i++)
    {
        //Update
            //Calculation likelihood of the state
        M[i].weight = likelihood(M[i].posx,M[i].posy,M[i].ori);
        sumofWeights += M[i].weight;
    }


            //Recalculation weights

    for(int i = 0; i < NP; i++)
    {
        M[i].weight = M[i].weight /sumofWeights;
        // Show map to and show where the robot is more likeli to be.
    }

        //Resample
            //Generate the map showing the possible possistion.

    }


}

float particelFilter::likelihood(int posx,int posy,float ori)
{
    int indexGeneratedDistance = 0;
    int indexMeasuredDistance = 0;
    float likelihood = 1;

    for(int i = 0; i < numberOfGeneratedLaserReadings; i++)
    {
        if((-pi+readingsIntervall*i-ori)<-(sensorRange))
        {
           indexGeneratedDistance = i;
           i = numberOfGeneratedLaserReadings;
        }
    }
    //std::cout<< "numberOfGeneratedLaserReadings: " << i << std::endl;
    for(int i = 0; i < numberOfLaserReadings; i++)
    {
       if(-sensorRange+lidarIncrement*i>-pi+readingsIntervall*indexGeneratedDistance-ori)
       {

           indexMeasuredDistance = i;
           i = numberOfLaserReadings;
       }
    }

   // std::cout<< "numberOfLaserReadings: " << i << std::endl;

    while(indexMeasuredDistance< numberOfLaserReadings)
    {
       if(indexGeneratedDistance<numberOfGeneratedLaserReadings)
       {
           likelihood = likelihood * normPDF(sensor->getLidarReading(indexMeasuredDistance),realMapDistance[posx][posy][indexGeneratedDistance],STD);
       }
       else
       {
           indexGeneratedDistance = indexGeneratedDistance-numberOfGeneratedLaserReadings;
           likelihood = likelihood * normPDF(sensor->getLidarReading(indexMeasuredDistance),realMapDistance[posx][posy][indexGeneratedDistance],STD);
       }
       indexGeneratedDistance++;
       indexMeasuredDistance += indexIntervall;
    }

    return likelihood;
}

cv::Mat particelFilter::enlargeImage(cv::Mat img,int size)
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
   //cv::imshow("whaaat", image);
   //1cv::waitKey(10);
   return image;
}

void particelFilter::generateDistanceFromEachPosistion()
{
    numberOfGeneratedLaserReadings = totallaserreadingsused;

    std::vector<std::vector<std::vector<float>>> tempvector(map.cols, std::vector<std::vector<float>>(map.rows, std::vector<float>(numberOfGeneratedLaserReadings,0.0)));

    realMapDistance = tempvector;

    float currentangle = -pi;
    readingsIntervall = 2*pi/numberOfGeneratedLaserReadings;

    for(int i = 0; i < map.rows-1;i++)
    {
        for(int k = 0; k < map.cols-1;k++)
        {
            //std::cout << "i: " << i << " k: "<< k << " "<< std::endl;
            if( map.at<uchar>(i,k) != 0)
            {
                currentangle = -pi;
                for(int j = 0; j< numberOfGeneratedLaserReadings; j++)
                {
                    //int i = map.cols/2;
                    //int k=  map.rows/2;
                    //std::cout << "i: " << i << " k: "<< k << " map.rows " << map.rows<< " map.cols "<< map.cols <<  std::endl;
                    int xCoordinates = k ;//map.cols/2;
                    int yCoordinates = i;//map.rows/2;
                    float radius = 0;
                    currentangle += readingsIntervall;
                    //std::cout << "currentangle: " << currentangle << " radius: "<< radius << std::endl;
                    if( map.at<uchar>(yCoordinates,xCoordinates) != 0)
                    {
                        while( map.at<uchar>(yCoordinates,xCoordinates) != 0 && radius < 10.1*sizeofMapComparedtoOriginal && xCoordinates < map.cols-1 && yCoordinates < map.rows-1 && xCoordinates > 0  && yCoordinates > 0 )
                        {
                            realMapDistance[k][i][j] = (radius)/sizeofMapComparedtoOriginal-2;
                            radius +=0.1;


                                 map.at<uchar>(yCoordinates,xCoordinates) = 50;


                            xCoordinates = int((k+radius *  cos(currentangle))+0.5);
                            yCoordinates = int((i+radius *  sin(currentangle))+0.5);



                            //std::cout << "xCoordinates " << xCoordinates << " yCoordinates " << yCoordinates <<" radius: "<< radius <<  std:: endl;
                            if(xCoordinates < 0 || yCoordinates < 0 || yCoordinates > map.rows || xCoordinates > map.cols)
                            {
                                //std:: cout << "here"<< std::endl;
                                xCoordinates = map.cols-1;
                                yCoordinates = map.rows-1;

                            }


                        }
                    }


                }

            }
        }

    }
    currentangle = -pi;

   /* for(int i = 0; i < numberOfGeneratedLaserReadings; i++)
    {

        int xCoordinates = map.cols/2;
        int yCoordinates = map.rows/2;

        currentangle += intervall;
        std::cout << "Currennt angle " << currentangle << std::endl;
        std::cout << realMapDistance[xCoordinates][yCoordinates][i] << std::endl;
        map.at<uchar>(yCoordinates,xCoordinates) = 255;

    }
    cv::imshow("whaaat", map);
    cv::waitKey(0);*/
}

float particelFilter::normPDF(int x,float mean,float std)
{
    return 1/(std*sqrt(2*pi))*exp(-((x-mean)*(x-mean))/(2*std*std));
}


float particelFilter::normalize(float x,float min,float max,float rangemin,float rangemax)
{
    return ((x-min)/(max-min))*(rangemax-rangemin)+rangemin;
}
