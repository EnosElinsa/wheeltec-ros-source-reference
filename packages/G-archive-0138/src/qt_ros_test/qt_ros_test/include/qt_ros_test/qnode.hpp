/**
 * @file /include/qt_ros_test/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qt_ros_test_QNODE_HPP_
#define qt_ros_test_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <cv_bridge/cv_bridge.h>

#include <QImage>



/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

    void init_pubsub();
    void init_param();
    void pub_cmd(bool car_model,int keybutton,double speed,double turn);
    void sub_camera(QString cameratopic);
    void image1_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void image2_callback(const sensor_msgs::msg::CompressedImage &msg);
    void batteryCallback(const std_msgs::msg::Float32::SharedPtr message);
    void speedCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    QImage Mat2QImage(cv::Mat const& src);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void image_val(QImage);
    void batteryState(double);
    void speedState(double,double,double,double,double,double);

public:
    rclcpp::Node::SharedPtr nh=nullptr;

private:
	int init_argc;
	char** init_argv;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr imagecomp_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Cmd_Vel_Sub;
    float control_speed = 0; //#前进后退实际控制速度
    float control_turn  = 0; //#转向实际控制速度
    float control_HorizonMove = 0; //#横向移动实际控制速度
    QString subcameratopic;
    //std::string rgbtopic,depthtopic;


    int oldkey;
};

#endif /* qt_ros_test_QNODE_HPP_ */
