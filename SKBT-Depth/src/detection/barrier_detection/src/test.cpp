#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "CSF.h"

cv::Mat depth_image;
cv::Mat depth_image_noinf;
bool barrier_flag;
C_S_FILTER csf(320, 240);
Eigen::Quaterniond Q;
Eigen::Vector3d P;


void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // 将ROS图像消息转换为OpenCV格式
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    depth_image = cv_ptr->image.clone();
   
    // 处理深度图像中的无穷大值
    cv::MatIterator_<float> it, end;
    for (it = cv_ptr->image.begin<float>(), end = cv_ptr->image.end<float>(); it != end; ++it)
    {
        if (std::isinf(*it))
        {
            *it = 100.0f;
        }
    }
    depth_image_noinf = cv_ptr->image;

    csf.Input_Depth(depth_image_noinf);
    csf.Reset_Cloth();
    csf.Shot();
    csf.Barrier_Check();
    csf.Find_Hole(Q, P);
    csf.Show_Result('H');
}

void odomCallback(const odom_fusion::OdomStamp& msg){
    P[0] = msg.pose.pose.pose.position.x;
    P[1] = msg.pose.pose.pose.position.y;
    P[2] = msg.pose.pose.pose.position.z;

    Q.coeffs()[0] = msg.pose.pose.pose.orientation.x;  // 设置虚部 x 
    Q.coeffs()[1] = msg.pose.pose.pose.orientation.y;  // 设置虚部 y
    Q.coeffs()[2] = msg.pose.pose.pose.orientation.z;  // 设置虚部 z
    Q.coeffs()[3] = msg.pose.pose.pose.orientation.w;  // 设置实部 w

}

void onMouse(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        // 输出点击位置处的像素值
        if (!depth_image.empty())
        {
            float depth_value = depth_image_noinf.at<float>(y, x);
            ROS_INFO("Depth value at (%d, %d): %f", y, x, depth_value);
        }
    }
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "test");

    // 创建节点句柄
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/xv_sdk/xv_dev/tof_camera/image", 1, depthImageCallback);

    ros::Subscriber odom_sub = nh.subscribe("/fusion/odom", 1, odomCallback);

    // 创建一个空白窗口
    cv::namedWindow("Depth Image");

    // 设置鼠标事件回调函数
    cv::setMouseCallback("Depth Image", onMouse, NULL);

     // 创建一个空白窗口
    cv::namedWindow("Marked Columns");


    // 循环等待回调函数
    ros::spin();

    return 0;
}