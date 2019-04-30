#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ctime>
#include "std_msgs/Int16.h"
//#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include <iostream>

//CHANGE
#include <iostream>
#include <fstream>
#include <ctime>
#include <chrono>

#include <csignal>
#include <cstdio>
#include <math.h>


using namespace std;
using namespace cv;

//#define MEASURE_TIME 1
#define COLOR_TRACKING 1

#ifdef MEASURE_TIME
clock_t t_begin = 0;
#endif

static int counter = 0;
static int goal = 1000;
static double time_record[] = {};

int detected_count = 0;

void cv_process_img(const Mat& input_img, Mat& output_img)
{
    Mat gray_img;
    cvtColor(input_img, gray_img, CV_RGB2GRAY);
    
    double t1 = 20;
    double t2 = 50;
    int apertureSize = 3;
    
    Canny(gray_img, output_img, t1, t2, apertureSize);
}

void cv_publish_img(image_transport::Publisher &pub, Mat& pub_img)
{
    //sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_img).toImageMsg();
    sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", pub_img).toImageMsg();
    pub.publish(pub_msg);
}

void cv_color_tracking(const Mat& input_img, ros::Publisher &controlPub)
{
    int iLowH = 160;
    int iHighH = 179;
    
    int iLowS = 150;
    int iHighS = 255;
    
    int iLowV = 60;
    int iHighV = 255;
    
    Mat imgLines = Mat::zeros(input_img.size(), CV_8UC3);    
    Mat imgHSV;
    //convert input image from RGB to HSV
    cvtColor(input_img, imgHSV, CV_RGB2HSV);

    Mat1b mask1;
    Mat1b mask2;
    //*** supply a range of r,g,b to exclude
    cv::inRange(imgHSV, Scalar(115,127,65), Scalar(125, 255,255), mask1); // red
    cv::inRange(imgHSV, Scalar(115,127,65), Scalar(125, 255,255), mask2); // red

    //Mat1b mask = mask1;

    //    cv::inRange(imgHSV, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), mask1);    //Blue
    //    cv::inRange(imgHSV, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), mask2); // Blue
    //    cv::Mat1b mask = mask1 | mask2;
    cv::Mat1b mask = mask1 | mask2;

    erode(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    
    //morphological closing (removes small holes from the foreground)
    dilate( mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    
    //Find the contour, then indentify rectangle, center, radius using openCV build in functions
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
   

    double limit = 12000;
    int detected = false;
    std_msgs::Int16 msg;

    size_t num_contours = contours.size();
    for (size_t i = 0; i < num_contours; i++) {
        double area = contourArea(contours[i]);

    	if (area > limit) {
            cout << "Area: " << area << endl;
    	    detected = true;
    	    
            cout << boundingRect(contours[i]) << endl; // x,y - x is the width, y is the height from (x, y) starting from top left 
    	    
            Scalar sc(0,255,255); // values for the color yellow, used repeatedly in bounding box and drawing x.
            Rect br = boundingRect(contours[i]); // contour is a matrix of points, so the object to be detected
            double cx = br.x + br.width/2;
            double cy = br.y + br.height/2;
            Point center(cx, cy);
            cv::line(input_img, Point(cx - br.width/30, cy - br.height/30),
    	    Point(cx + br.width/30, cy + br.height/30), sc, 5, 8,0);
            cv::line(input_img, Point(cx - br.width/30, cy + br.height/30),
    	    Point(cx + br.width/30, cy - br.height/30), sc, 5, 8,0);

            // assuming width of the detectable region to be represented by 1080, we portion in thirds.
            // the reported position depends on which third the detected object is found
            if ( cx < 360 ){
                cout << "left" << endl;
    	    }
    	    else if ( cx > 720 ){
                cout << "right" << endl;
    	    }
    	    else{
                cout << "center" << endl;
            }
            cv::rectangle(input_img, br.tl(), br.br(), sc, 2);
    	}
    }
    if (detected){
        detected_count += 1;
    }else{
        detected_count = 0;
    }
    
    if (detected_count >= 2) {
        cout << "Detected" << std::endl;
        msg.data = 1;
        controlPub.publish(msg);
    }else{
        cout << "Haven't detected" << std::endl;
	msg.data = 0;
        controlPub.publish(msg);
    }
    cv::imshow("color_tracking_input_image", input_img);
    cv::imshow("blue_tracking", mask); 
    
    waitKey(1);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg, image_transport::Publisher &pub, ros::Publisher &controlPub)
{
    
    auto start = std::chrono::high_resolution_clock::now();
    cv_bridge::CvImageConstPtr cv_ori_img_ptr;
    try{
        Mat cv_ori_img = cv_bridge::toCvShare(msg, "bgr8")->image;
        Mat cv_output_img;
        
        cv_process_img(cv_ori_img, cv_output_img);
        
#ifdef COLOR_TRACKING
        cv_color_tracking(cv_ori_img, controlPub);
#endif
        
        cv_publish_img(pub, cv_output_img);
        
#ifdef MEASURE_TIME
        clock_t t_end = clock();
        double delta_time= double(t_end - t_begin) / CLOCKS_PER_SEC;
        //cout << "Delta_t = " << 1/delta_time << "\n";
        //t_begin = t_end;
#endif
        
        //imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        //imshow("view", cv_output_img);
        waitKey(30);
        
#ifdef MEASURE_TIME
        t_begin = clock();
#endif
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    // process time measurement
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    time_record[counter] = (double)elapsed.count();
    counter += 1;

    if(counter == goal){
        double sum = 0.0;
        for(int i = 0; i < goal; i ++){
            sum += time_record[i];
        }
        double mean = sum/goal;
        const char *path = "/home/nvidia/catkin_ws/time_test/color_tracking_recording.txt";
        ofstream file;
        file.open(path, ios::app);
        file << "The mean is: " << mean*1000 << "ms" << endl;
        time_t now = time(0);
        char* dt = ctime(&now);
        file << dt << '\n' << '\n';
        counter = 0;
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    
    ros::NodeHandle nh;
    
    //namedWindow("Vview");
    startWindowThread();
    image_transport::ImageTransport it(nh);
    
    ros::NodeHandle nh_pub;
    image_transport::ImageTransport itpub(nh_pub);
    image_transport::Publisher pub = itpub.advertise("sample/cannyimg", 1);
    
    ros::NodeHandle node;
    uint32_t queue_size = 500;
    ros::Publisher controlPub = node.advertise<std_msgs::Int16>("imagecontrol", queue_size);
    
    
#ifdef MEASURE_TIME
    t_begin = clock();
#endif
    
    image_transport::Subscriber sub = it.subscribe("rgb/image_rect_color", 1, boost::bind(imageCallback, _1, pub, controlPub));
    //ros::Subscriber controlSub = it.subscribe("rbg/image_rect_color")
    ros::spin();
    

    
    //destroyWindow("view");
    
}

