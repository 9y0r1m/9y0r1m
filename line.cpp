#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;

string codec1 = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)640, height=(int)360, format=(string)NV12, framerate=(fraction)15/1 ! \
     nvvidconv flip-method=0 ! video/x-raw, width=(int)640, height=(int)360, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "line_tracing");
    ros::NodeHandle nh;
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Video on
    VideoCapture cap1(codec1, CAP_GSTREAMER);
    if (!cap1.isOpened()) {
        cout << "Video error" << endl;
        return -1;
    }

    Mat frame, frame1, dst;
    Point2d prevpt(320, 60);
    Point2d cpt;
    int minlb;
    int thres;
    double ptdistance;
    double threshdistance;
    vector<double> mindistance;
    double error;
    double myproms;

    while (ros::ok())
    {
        int64 t1 = getTickCount();

        cap1 >> frame;

        if (frame.empty()) {
            cout << "No frame captured" << endl;
            break;
        }

        cvtColor(frame, frame1, COLOR_BGR2GRAY);
        cout << "mean: " << mean(frame1)[0] << endl;
        frame1 = frame1 + 100 - mean(frame1)[0];
        cout << "mean2: " << mean(frame1)[0] << endl;
        thres = 160;
        threshold(frame1, frame1, thres, 255, THRESH_BINARY);

        dst = frame1(Rect(0, frame1.rows / 3 * 2, frame1.cols, frame1.rows / 3));

        Mat labels, stats, centroids;
        int cnt = connectedComponentsWithStats(dst, labels, stats, centroids);

        if (cnt > 1) {
            for (int i = 1; i < cnt; i++) {
                double* p = centroids.ptr<double>(i);
                ptdistance = abs(p[0] - prevpt.x);
                mindistance.push_back(ptdistance);
            }
            for (int i : mindistance) cout << i << " ";
            cout << endl;
            threshdistance = *min_element(mindistance.begin(), mindistance.end());
            minlb = min_element(mindistance.begin(), mindistance.end()) - mindistance.begin();
            cpt = Point2d(centroids.at<double>(minlb+1, 0), centroids.at<double>(minlb+1, 1));
            if (threshdistance > 100)
                cpt = prevpt;
            mindistance.clear();
        }
        else cpt = prevpt;

        prevpt = cpt;
        cvtColor(dst, dst, COLOR_GRAY2BGR);

        circle(frame, cpt, 2, Scalar(0, 0, 255), 2);

        error = dst.cols / 2 - cpt.x;
        cout <<"error: " << error << endl;

        int L_speed = 0.3 - error/500.0;  // Adjust for proper scaling
        int R_speed = 0.3 + error/500.0;

        // Create Twist message
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = (L_speed + R_speed) / 2.0;
        vel_msg.angular.z = (R_speed - L_speed) / 0.5;  // Adjust for turning

        // Publish velocity command
        velocity_pub.publish(vel_msg);

        imshow("dst", frame);
        waitKey(33);
        
        int64 t2 = getTickCount();
        myproms = (t2 - t1) * 1000 / getTickFrequency();
        cout << "time: " << myproms << endl;
    }

    return 0;
}