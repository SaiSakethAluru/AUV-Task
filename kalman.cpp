#include <iostream>
#include <vector>
#include <sys/time.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

using namespace cv;
using namespace std;


cv::KalmanFilter KF;
cv::Mat_<float> measurement(2,1); 
Mat_<float> state(4, 1); // (x, y, Vx, Vy)

Point detectCentre(Mat& img);

void initKalman(float x, float y)
{
    // Instantate Kalman Filter with
    // 4 dynamic parameters and 2 measurement parameters,
    // where my measurement is: 2D location of object,
    // and dynamic is: 2D location and 2D velocity.
    KF.init(4, 2, 0);

    measurement = Mat_<float>::zeros(2,1);
    measurement.at<float>(0, 0) = x;
    measurement.at<float>(0, 0) = y;

    KF.statePre.setTo(0);
    KF.statePre.at<float>(0, 0) = x;
    KF.statePre.at<float>(1, 0) = y;

    KF.statePost.setTo(0);
    KF.statePost.at<float>(0, 0) = x;
    KF.statePost.at<float>(1, 0) = y; 

    KF.transitionMatrix = (Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1); 
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(.005)); //adjust this for faster convergence - but higher noise
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, Scalar::all(.1));
}

Point kalmanPredict() 
{
    Mat prediction = KF.predict();
    Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
    return predictPt;
}

Point kalmanCorrect(float x, float y)
{
    measurement(0) = x;
    measurement(1) = y;
    Mat estimated = KF.correct(measurement);
    Point statePt(estimated.at<float>(0),estimated.at<float>(1));
    return statePt;
}

//------------------------------------------------ main

int main (int argc, char * const argv[]) 
{
    Point correctedPoint, predictedPoint;

    initKalman(0, 0);
    VideoCapture vid("frontcam.avi");
    Mat img;
    while(vid.read(img)){
        predictedPoint = kalmanPredict();
        cout << "kalman prediction: " << predictedPoint.x << " " << predictedPoint.y << endl;
        Point center = detectCentre(img);
        if(center.x!=0 || center.y!=0){
            correctedPoint = kalmanCorrect(center.x,center.y);
        }
        cout << "kalman corrected state: " << correctedPoint.x << " " << correctedPoint.y << endl;
        
        // draw crosses at located points to display 

        #define drawCross( center, color, d )                                        \
            line( img, Point( center.x - d, center.y - d ),                          \
                 Point( center.x + d, center.y + d ), color, 1, LINE_AA, 0); \
            line( img, Point( center.x + d, center.y - d ),                          \
                 Point( center.x - d, center.y + d ), color, 1, LINE_AA, 0 )
       
        drawCross(center,Scalar(255,255,255),3);
        drawCross(correctedPoint,Scalar(0,0,255),3);
        drawCross(predictedPoint,Scalar(0,255,255),3);
        imshow("output",img);
        waitKey(60);

    }
    return 0;
}
// detect the object in the video

Point detectCentre(Mat& img)
{
    long long x=0;
    long long y = 0;
    long long count = 0;

    // threshold values of the bouy

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
    // Point p;
        // Threshold for minimum number of points to avoid noise detection
    if(count>100){ 
        Point p(y/count,x/count);
        return p;  
    }
    else{
        Point p(0,0);
        return p;
    }
}