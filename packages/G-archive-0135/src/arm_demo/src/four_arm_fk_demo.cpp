#include <ros/ros.h>
#include <ros/console.h>
#include <moveit/move_group_interface/move_group_interface.h>

bool success;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_fk_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("arm");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    std::vector<double> joint_group_positions(3);
    // 控制机械臂回到预设置的位置
    arm.setNamedTarget("arm_home");
    arm.move();
    sleep(1); 
    
    //直接输入三个关节的目标旋转角度
    joint_group_positions[0] =  1.0;
    joint_group_positions[1] = 1.56;
    joint_group_positions[2] =  1.2;

    arm.setJointValueTarget(joint_group_positions);

    success = ((arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS));
    ROS_INFO_NAMED("fk_demo_plan",success ? "plan_success" : "plan_False");

    if(success)  arm.execute(my_plan),sleep(1); //如果规划成功则执行

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("arm_home");
    arm.move();
    sleep(1);

    ros::shutdown(); 

    return 0; 
}