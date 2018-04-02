#include <iostream>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"

using namespace std;
using namespace cv;
Mat detectCentre(Mat& img);

int main()
{
    VideoCapture vid("frontcam.avi");
    Mat img;
    namedWindow("out",WINDOW_NORMAL);
    while(vid.read(img))
    {
        Mat img3(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));
        img3 = detectCentre(img);
        imshow("out",img3);
        waitKey(60);
    }
}

Mat detectCentre(Mat& img)
{
    long long x=0;
    long long y = 0;
    long long count = 0;
    int r= 53,R=256,b=0,B=85,g=0,G=90;
    for(int i=0;i<img.rows;i++){
        for(int j=0;j<img.cols;j++){
            if(img.at<Vec3b>(i,j)[0]>b && img.at<Vec3b>(i,j)[0]<B && img.at<Vec3b>(i,j)[1] >g && img.at<Vec3b>(i,j)[1]<G && img.at<Vec3b>(i,j)[2]>r && img.at<Vec3b>(i,j)[2]<R){
                x+=i;
                y+=j;
                count++;
            }
        }
    }
    if(count>100)
        circle(img,Point(y/count,x/count),3,Scalar(0,0,255)); 
    return img;  
}