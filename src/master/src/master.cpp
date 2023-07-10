#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "boost/thread/mutex.hpp"
#include <opencv2/aruco.hpp>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <termios.h>
#include <sys/ioctl.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"

ros::Publisher pub_velocity;
ros::Subscriber sub_lidar;
ros::Subscriber sub_odometry;
ros::Subscriber sub_imu;

ros::Timer tim_;
image_transport::Publisher pub_vision_debugger;
image_transport::Subscriber sub_camera;

using namespace cv;

Mat frame_raw;
boost::mutex mutex_frame_raw;

float vel_robot[3];
float odometry_buffer[3];
float odometry_offset[3] = {0, 0, 90};
float odometry_final[3];

float lidar_depan;
float lidar_belakang;

Mat camera_matrix;
Mat distCoeffs;


std::vector<float> lidar_data;

void CallbackSubCamera(const sensor_msgs::ImageConstPtr &msg);
void CallbackTim50Hz(const ros::TimerEvent &event);
void CallbackSubOdom(const nav_msgs::OdometryConstPtr &msg);
void CallbackSubImu(const sensor_msgs::ImuConstPtr &msg);
void CallbackSubLidar(const sensor_msgs::LaserScanConstPtr &msg);

void motion_(float vel_x, float vel_y, float vel_th);

int kbhit();

uint16_t aruco_false_detect;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "master");
    ros::NodeHandle NH;
    image_transport::ImageTransport IT(NH);
    ros::MultiThreadedSpinner spinner(0);


    // //=====================================

    // resize(camera_matrix,camera_matrix,Size(3,3));
    // resize(distCoeffs,distCoeffs,Size(1,5));

    // camera_matrix.at<float>(Point(0,0)) = 245.312887;
    // camera_matrix.at<float>(Point(0,1)) = 0;
    // camera_matrix.at<float>(Point(0,2)) = 165.227376;
    // camera_matrix.at<float>(Point(1,0)) = 0;
    // camera_matrix.at<float>(Point(1,1)) = 245.201850;
    // camera_matrix.at<float>(Point(1,2)) = 123.723979;
    // camera_matrix.at<float>(Point(2,0)) = 0;
    // camera_matrix.at<float>(Point(2,1)) = 0;
    // camera_matrix.at<float>(Point(2,2)) = 1;

    // distCoeffs.at<float>(Point(0,0)) = 0.087617;
    // distCoeffs.at<float>(Point(0,1)) = -0.187180;
    // distCoeffs.at<float>(Point(0,2)) = 0.002487;
    // distCoeffs.at<float>(Point(0,3)) = 0.004910;
    // distCoeffs.at<float>(Point(0,4)) = 0;

    //=====================================

    sub_camera = IT.subscribe("/camera/image", 1, CallbackSubCamera);
    pub_vision_debugger = IT.advertise("vision_debugger", 2);

    pub_velocity = NH.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    sub_odometry = NH.subscribe("/odom", 1, CallbackSubOdom);
    sub_imu = NH.subscribe("/imu", 1, CallbackSubImu);
    sub_lidar = NH.subscribe("scan", 1, CallbackSubLidar);

    tim_ = NH.createTimer(ros::Duration(0.02), CallbackTim50Hz);
    spinner.spin();

    return 0;
}

void CallbackTim50Hz(const ros::TimerEvent &event)
{
    static Mat frame_raw_clone;


    mutex_frame_raw.lock();
    frame_raw_clone = frame_raw.clone();
    mutex_frame_raw.unlock();

    Mat frame_raw_gray;

    cvtColor(frame_raw_clone, frame_raw_gray, CV_RGB2GRAY);

    Mat frame_raw_bin = frame_raw_gray > 128;

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(frame_raw_bin, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);



    aruco_false_detect = rejectedCandidates.size();
    static uint32_t false_detect_cntr = 0;

    // ========================== KEYBOARD =================

    static int16_t interrupt_state;
    static int16_t robot_state = 0;

    // AGAR BISA TELEOP KEYBOARD
    if (kbhit() > 0)
    {
        char key = std::cin.get();

        switch (key)
        {
        case 'j':
            interrupt_state = 10;
            break;
        case ' ':
            interrupt_state = 30;
            robot_state = 0;
            break;
        case 'A':
            interrupt_state = 20;
            break;
        case 't':
            robot_state = 69;
            break;

        }
    }

    // ================================== FINITE STATE MACHINE ===================================

    printf("POSE: %.2f %.2f %.2f\n", odometry_buffer[0], odometry_buffer[1], odometry_buffer[2]);

    static float th_buff;
    static float jarak_aruco;
    static float pos_buffer[2];

    switch (robot_state)
    {
    case 69:
    {

        float error = 0;

        if(markerCorners.size()>0){
            if(markerCorners[0].size() > 0)
            {
                float pojok_kiri = markerCorners[0][0].x - 360;
                float pojok_kanan = markerCorners[0][1].x - 360;

                error = pojok_kiri + pojok_kanan;
            }
        }

        float v_out = error * 0.0005;

        printf("error: %.2f -> %.2f\n",error,v_out);

        if(v_out > 1)
        v_out =1;
        else if(v_out < -1)
        v_out = -1;


        if(fabs(error) < 10){
            motion_(0,0,0);
        robot_state = 20;

        }
        else {
            motion_(0,0,-v_out);

        }



    break;
    }
    case 0:
        // tunggu saat distart keyboard
        // printf("TUNGGU DI START, ARUCO %ld\n",markerCorners.size());
        if (interrupt_state == 20)
        {
            robot_state = 10;
        }
        motion_(0, 0, 0);
        false_detect_cntr = 0;
        break;
    case 10:
        printf("JALAN SAMPAI NEMU ARUCO %ld %ld %d\n", markerIds.size(), rejectedCandidates.size(), false_detect_cntr);

        motion_(0.1, 0, 0);
        if (aruco_false_detect > 6)
        {
            false_detect_cntr++;
        }

        if ((/*false_detect_cntr >= 50 || */ markerIds.size() > 0) && lidar_depan <= 60) 
        {
            robot_state = 69;
        }
        break;
    case 20:
        motion_(0, 0, 0);

        th_buff = odometry_buffer[2];

        while (th_buff > 180)
            th_buff -= 360;
        while (th_buff < -180)
            th_buff += 360;

        robot_state = 30;
        jarak_aruco = lidar_depan;

        break;
    case 30:
    {

        static const float max_th_vel_rad = 1.5;
        static const float kp = 1.5;
        float th_error = th_buff - 180 - odometry_buffer[2];

        while (th_error > 180)
            th_error -= 360;
        while (th_error < -180)
            th_error += 360;

        float out_buff = th_error * M_PI / 245 * kp;

        if (out_buff > max_th_vel_rad)
            out_buff = max_th_vel_rad;
        else if (out_buff < -max_th_vel_rad)
            out_buff = -max_th_vel_rad;

        motion_(0, 0, out_buff);

        if (fabs(th_error) < 2)
        {
            robot_state = 40;
            pos_buffer[0] = odometry_buffer[0];
            pos_buffer[1] = odometry_buffer[1];
        }

        printf("TH: %.2f %.2f\n",odometry_buffer[2], th_buff - 180);

        break;
    }
    case 40:
    {
        printf("MUNDUR\n");
        motion_(-0.2, 0, 0);

        float dx = odometry_buffer[0] - pos_buffer[0];
        float dy = odometry_buffer[1] - pos_buffer[1];

        if (sqrtf(dx * dx + dy * dy) > jarak_aruco - 40) // 90, adalah jarak mundur, semakin besar semakin jauh dari aruco
        {
            robot_state = 50;
        }
        break;
    }
    case 50:
        printf("SAMPAI\n");
        motion_(0, 0, 0);
        break;
    }

    // ============================= TRANSMITTER ======================================

    geometry_msgs::Twist msg_vel_motor;
    msg_vel_motor.linear.x = vel_robot[0];
    msg_vel_motor.linear.y = vel_robot[1];
    msg_vel_motor.angular.z = vel_robot[2];
    pub_velocity.publish(msg_vel_motor);

    sensor_msgs::ImagePtr msg_vision_debugger = cv_bridge::CvImage(std_msgs::Header(), "mono8", frame_raw_bin).toImageMsg();
    pub_vision_debugger.publish(msg_vision_debugger);
}

void CallbackSubCamera(const sensor_msgs::ImageConstPtr &msg)
{
    mutex_frame_raw.lock();
    frame_raw = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    mutex_frame_raw.unlock();
}

void motion_(float vel_x, float vel_y, float vel_th)
{
    vel_robot[0] = vel_x;
    vel_robot[1] = vel_y;
    vel_robot[2] = vel_th;
}

void CallbackSubOdom(const nav_msgs::OdometryConstPtr &msg)
{
    odometry_buffer[0] = msg->pose.pose.position.x * 100; // convert ke cm
    odometry_buffer[1] = msg->pose.pose.position.y * 100;
    // odometry_buffer[2] = msg->pose.pose.orientation.w * 180; // convert ke derajat

    // while (odometry_buffer[2] > 180)
    //     odometry_buffer[2] -= 360;
    // while (odometry_buffer[2] < -180)
    //     odometry_buffer[2] += 360;
}

void CallbackSubImu(const sensor_msgs::ImuConstPtr &msg)
{
    odometry_buffer[2] += msg->angular_velocity.z;

    while (odometry_buffer[2] > 180)
        odometry_buffer[2] -= 360;
    while (odometry_buffer[2] < -180)
        odometry_buffer[2] += 360;
}

void CallbackSubLidar(const sensor_msgs::LaserScanConstPtr &msg)
{
    lidar_data = msg->ranges;
    int it = 0;
    float total = 0;
    int16_t cntr = 0;
    for (float i = msg->angle_min; i < msg->angle_max; i += msg->angle_increment)
    {
        // printf("%.2f -> %.2f\n", i * 180 / M_PI, lidar_data[it] * 100);

        if (i * 180 / M_PI > 0 && i * 180 / M_PI < 10 && lidar_data[it] != 0)
        {
            total += lidar_data[it];
            cntr++;
        }
        it++;
    }

    lidar_depan = total / cntr * 100;
    lidar_belakang = total / cntr * 180;
    // printf("================================================ %.2f\n", lidar_depan);
}

int kbhit()
{
    static const int STDIN = 0;
    static bool initialized = false;

    if (!initialized)
    {
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;

    if (ioctl(STDIN, FIONREAD, &bytesWaiting) == -1)
        return 0;

    return bytesWaiting;
}
