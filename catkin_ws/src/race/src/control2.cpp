#include <ros/ros.h>
#include <race/drive_param.h>
#include <race/pid_input.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/LaserScan.h>

#include <ctime>
#include <chrono>
#include <thread>

#include <string>
#include <vector>
#include <iostream>
#include <iterator>
#include <iomanip>
#include <sstream>
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"

#include <math.h>

/*TODO: 


 receives processed data from colortracking: area, position


follow red dot() speed based on area of object
obstacle avoidance() avoid obstacle by turning, keep memory of previous red dot position, restore after avoiding obstacle

  
pid
for speed vs area
obstacle dist vs turn/speed


*/


using namespace std;

static double kp = 250.0;
static double kd = 0.0;
static double servo_offset = 18.5;
static double prev_error = 0.0;
static double vel_input = -2.0;

static double vel_min = 6.0;
static double vel_max = 40.0;

static double start_time = 0.0;
static int choice = 0;// choose spin mode
static int angle_input = 0;
static bool distancePriority;
static int positionTarget;

static bool anyDetect = false;
static auto end_time = std::chrono::high_resolution_clock::now();
void autoCallback(const boost::shared_ptr<const std_msgs::String> &data, ros::Publisher &pub) {
	race::drive_param pubMsg;
	std::string subMsg = data->data.c_str();
	std::string delimiter = ",";
	int index = subMsg.find(delimiter);
	double area = stod(subMsg.substr(0,index));
	cout << "Area: " << area << endl;
	subMsg.erase(0, index + 1);
	index = subMsg.find(delimiter);
	double center = stod(subMsg.substr(0, index));
	cout << "Center: " << center << endl;
	subMsg.erase(0, index + 1);
	int detected = stoi(subMsg);
	cout << "Detected: " << detected << endl;
	double distance = 275 * pow(area, -0.5);
	double angle = 57.3 * atan(7/24.0 * (center - 540) / (sqrt(area / M_PI) * distance));
	cout << "Distance to circle = " << distance << endl;
	cout << "angle to move = " << angle << endl;
	if(detected == 1){
		anyDetect = true;
		//vel_input = pow(250 * vel_max, -0.5);
		vel_input = 16;
		// 200 center 440 on sides total 1080
	/*
		if( center <= 80 ){
		    angle_input = -(70 - 70/80 * center);
		}
		else if( center >= 1000){
		    double val = center - 1000;
		    angle_input = 70 - 70/100;
		}
		else{
		    angle_input = 0;
		}

		if(center <= 440){
		   angle_input = -(70 - 70/440 * center);
		}
		else if( center >= 640){
		   double val = center - 640;
		   angle_input = (70 - 70/440 * val);
		}
		else{
		   angle_input = 0;
		}
	*/
		angle_input = angle;
		if (center <= 360) {
			positionTarget = -1;
		} else if (center >= 720) {
			positionTarget = 1;
		} else {
			positionTarget = 0;
		}
		cout << "In detected if" << endl;
	} else {
		anyDetect = false;
		vel_input = -8.0;
		angle_input = 0;
	}
	pubMsg.velocity = vel_input;
	pubMsg.angle = angle_input;


	if (!distancePriority) {

		pub.publish(pubMsg);
	}
}

static double getRange(const sensor_msgs::LaserScan *data, int theta){
        double distance = data->ranges[theta];
/*
        if(!distance || distance > 1.5){
                distance = 50;
        }
*/
        if(!distance){
                cout << "distance";
        }
        return distance;
}

static double getRadians(double degree){
        const double PI = 3.1415926;
        return PI * degree / 180;
}


double p_distance;
static void distCallBack(const boost::shared_ptr<const sensor_msgs::LaserScan> &data, ros::Publisher &pub){
        double total = 0.0;
        double error = 0.0;
        double vel = -8.0;
        double c_distance;
	double max_way = 0.0;
	int max_way_index = 0;
	race::drive_param pubMsg;

        const sensor_msgs::LaserScan *raw_data = data.get();

        for(int i = 180; i < 540; i += 10){
                total += (getRange(raw_data,i)* cos(getRadians((i-180)/4))-.75) / getRange(raw_data,i) * sin(getRadians((i-180.0)/4));
        }
        error = total/36.0;

        c_distance = getRange(raw_data,540);

        p_distance = c_distance;
	bool obstaclePresent = false;
		for (int i = 480; i < 600; i+=10) {
			if(getRange(raw_data, i) < 0.75) {
				obstaclePresent = true;
			}
		}
	
	bool leftWall = false;
	bool rightWall = false;
	for (int i = 960; i > 900; i-=10) {
		if (getRange(raw_data, i) < 0.2) {
			leftWall = true;
		}
		if (getRange(raw_data, 1080 - i) < 0.2) {
			rightWall = true;
		}
	}
        if(obstaclePresent){
                distancePriority = true;
		cout << "***** distance has priority *****" << endl;
		if (positionTarget == 1) {
		
			for(int i = 360; i < 720; i+=10){
			   double dist_val = getRange(raw_data,i);
			   if(dist_val > max_way){
				max_way = dist_val;
				max_way_index = i;
		   		}
			}
        	} else {
			for (int i = 720; i > 360; i-=10) {
				double dist_val = getRange(raw_data, i);
				if (dist_val > max_way) {
					max_way = dist_val;
					max_way_index = i;
				}
			}
		}
		if( max_way >= 0.75 ){
		   pubMsg.angle = (540 - max_way_index) / 1.5;
		   vel = 30;
	           cout << ((540 - max_way_index) / 2) << endl;
	           cout << ((540 - max_way_index) / 6) << endl;

//		   cout << "lidar raw data " << i << ": " <<  getRange(raw_data,i) << endl;
		}
		pubMsg.velocity = vel;
		if (anyDetect) {
			pub.publish(pubMsg);
		}
		/*
		} else if (leftWall) {
			pubMsg.velocity = 16;
			pubMsg.angle = 20;
			// pubMsg.angle = 20;
			pub.publish(pubMsg);
		} else if (rightWall) {
			pubMsg.velocity = 16;
			pubMsg.angle = -20;
			// pubMsg.angle = 20;
			 pub.publish(pubMsg);
		*/		
	} else {
		distancePriority = false;
       	}

}


void callback(const boost::shared_ptr<const race::pid_input> &data, ros::Publisher &pub){

	// TEST: show time elapsed between callbacks
	// auto new_start = std::chrono::high_resolution_clock::now();
	// std::chrono::duration<double> elapsed = new_start - end_time;
	// cout << "Elapsed time between callback: " << (double)elapsed.count() << "s"<<endl;

	// double angle = data.get()->pid_error*kp;
	double angle = angle_input;
	race::drive_param msg;

	if (angle > 100){
		angle = 100;
	}
	if (angle < -100){
		angle = -100;
	}

	if (vel_input > 40) {
		vel_input = 40;
	}
	if (vel_input < 6) {
		vel_input = 0;
	}

	if(data.get()->pid_vel == 0){
		msg.velocity = -8;
	}
	else{
		msg.velocity = vel_input;
	}
	msg.velocity = 50;
	msg.angle = angle;
	cout << "In original" << endl;
	pub.publish(msg);

	// TEST:
	//end_time = std::chrono::high_resolution_clock::now();
}

int main(int argc, char **argv){
int pub_queue_size = 1;
int sub_queue_size = 1;
//int choice = 0;
int given_time = 100;
int hz = 10;
// string commandInput;
// string command = "s";
string command_s1;
string command_s2;

// Get user input
cout << "Listening to error for PID" << "\n";
cout << "hello" << endl;

// Initiate Node
ros::init(argc, argv, "pid_controller");
// Create Node Handeler for publisher
ros::NodeHandle p;
ros::Publisher pub = p.advertise<race::drive_param>("drive_parameters", pub_queue_size);
// Create Node Handeler for subscriber
//ros::NodeHandle s;
//ros::Subscriber sub = s.subscribe<race::pid_input>("error", sub_queue_size, boost::bind(callback, _1, pub));

// Create Node Handeler for another sub

ros::NodeHandle x;

  ros::Subscriber sub = x.subscribe<std_msgs::String>("control", 1000, boost::bind(autoCallback, _1, pub));

ros::NodeHandle s;

ros::Subscriber laserSub = s.subscribe<sensor_msgs::LaserScan>("scan", 1, boost::bind(distCallBack, _1, pub));	   

// Robust Spin
    if(choice == 0){
        ros::spin();
    }

    // Dynamic Spin
    else if(choice == 1){
        while(ros::ok()){
            // record the time in millisecond that used for the callback function
            auto start = std::chrono::high_resolution_clock::now();

            ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.01));

            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            int sleep_time = (int)(given_time - (double)elapsed.count()*1000);

            // Dynamically sleep
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
}
    }

    // Static Spin
    else if(choice == 2){
        ros::Rate r(hz);
        while (ros::ok()){
            ros::spinOnce();
            r.sleep();
        }
    } 
    return 0;
}
