import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import Twist
from autoware_auto_vehicle_msgs.msg import TurnIndicatorsCommand, VelocityReport, SteeringReport, GearReport, HazardLightsReport, TurnIndicatorsReport, ControlModeReport
from autoware_auto_control_msgs.msg import AckermannControlCommand
from ackermann_msgs.msg import AckermannDriveStamped
from tier4_vehicle_msgs.msg import VehicleEmergencyStamped
from autoware_auto_vehicle_msgs.msg import GearCommand
from autoware_auto_vehicle_msgs.srv import ControlModeCommand
from nav_msgs.msg import Odometry


class Autoware_Wheeltec(Node):
    def __init__(self):
        super().__init__('autoware_wheeltec_interface')

        # Publishers
        # self.pub_ulc = self.create_publisher(UlcCmd,'/vehicle/ulc_cmd', 10)  #for longitudinal speed
        self.pub_cmdvel = self.create_publisher(Twist,'/cmd_vel', 10)  #for wheeltec car speed
        # Publishers
        self.pub_velocity = self.create_publisher(VelocityReport,'/vehicle/status/velocity_status', 10)
        self.pub_turn_signal = self.create_publisher(TurnIndicatorsReport,'/vehicle/status/turn_indicators_status', 10)
        self.pub_hazard_signal = self.create_publisher(HazardLightsReport, '/vehicle/status/hazard_lights_status', 10)
        self.pub_gear = self.create_publisher(GearReport,'/vehicle/status/gear_status', 10)
        self.pub_steering = self.create_publisher(SteeringReport,'/vehicle/status/steering_status', 10)
        self.pub_control_mode = self.create_publisher(ControlModeReport,'/vehicle/status/control_mode', 10)

        # Subscribers
        self.subscription_control = self.create_subscription(AckermannControlCommand, '/control/command/control_cmd', self.callback_control, 10)
        self.subscription_gearmode = self.create_subscription(GearCommand, '/control/command/gear_cmd', self.callback_gearmode, 10)
        self.subscription_controlmode = self.create_service(ControlModeCommand, '/control/control_mode_request', self.callback_controlmode)
        self.subscription_turn_signal = self.create_subscription(TurnIndicatorsCommand, '/control/command/turn_indicators_cmd', self.callback_turn_signal, 10)
        self.subscription_odom = self.create_subscription(Odometry, '/sensing/odom', self.callback_odom, 10)
        self.steering_ratio = 14.8
        self.current_velocity = 0
        self.wheel_base = 0.322
        self.wheel_tread = 0.322
        self.odomspeed = 0.0
        self.odomturn = 0.0
        self.gearmode = 0
        self.automode = 0

        # disable traffic light , because no use
        turn_msg = TurnIndicatorsReport()
        turn_msg.stamp = self.get_clock().now().to_msg()
        turn_msg.report = 1
        self.pub_turn_signal.publish(turn_msg)

        # disable hazard light , because no use
        hazard_msg = HazardLightsReport()
        hazard_msg.stamp = turn_msg.stamp
        hazard_msg.report = 1
        self.pub_hazard_signal.publish(hazard_msg)

        # keep driving statue    1kongdang 2drive 20daoche 22tingche
        gear_msg = GearReport()
        gear_msg.stamp = turn_msg.stamp
        gear_msg.report = 22
        self.pub_gear.publish(gear_msg)

        control_msg = ControlModeReport()
        control_msg.stamp = turn_msg.stamp
        control_msg.mode = 4
        self.pub_control_mode.publish(control_msg)

    def callback_control(self, data):
        msg = Twist()
        speed = data.longitudinal.speed
        turn_angel = data.lateral.steering_tire_angle
        
        #msg.header.stamp = data.stamp
        #msg.header.frame_id = 'base_link'
        #if turn_angel != 0:
           # print(math.tan(turn_angel)*speed/self.wheel_base)
        if self.gearmode == 2 and self.automode == 1:
            msg.linear.x = speed
            msg.angular.z = math.tan(turn_angel)*speed/self.wheel_base
            #if turn_angel != 0:
                #msg.drive.steering_angle = math.atan(self.wheel_base/(self.wheel_base/math.tan(turn_angel)-self.wheel_tread/2))
            #msg.drive.speed = speed
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            #msg.drive.steering_angle = 0.0
            #msg.drive.speed = 0.0
        # print("autoware_wheeltec_interface turn_angel %f" ,turn_angel)
        #print(speed,self.gearmode,self.automode)
        self.pub_cmdvel.publish(msg)

    def callback_gearmode(self, data):
        self.gearmode = data.command
        gear_msg = GearReport()
        gear_msg.stamp = self.get_clock().now().to_msg()
        gear_msg.report = data.command
        self.pub_gear.publish(gear_msg)

    def callback_controlmode(self, request,response):
        self.automode=request.mode
        if self.automode == 1:      #auto
            control_msg = ControlModeReport()
            control_msg.stamp = self.get_clock().now().to_msg()
            control_msg.mode = 1
            self.pub_control_mode.publish(control_msg)
        elif self.automode ==4:     #manual
            control_msg = ControlModeReport()
            control_msg.stamp = self.get_clock().now().to_msg()
            control_msg.mode = 4
            self.pub_control_mode.publish(control_msg)
        response.success = True
        return response


    def callback_turn_signal(self, data):   #left2 right3 no use
        #self.signalvalue = data.command
        turn_msg = TurnIndicatorsReport()
        turn_msg.stamp = self.get_clock().now().to_msg()
        turn_msg.report = data.command
        self.pub_turn_signal.publish(turn_msg)

    def callback_odom(self, data):
        msg = VelocityReport()
        self.odomspeed = data.twist.twist.linear.x
        self.odomturn = data.twist.twist.angular.z 
        msg.header = data.header
        msg.longitudinal_velocity = self.odomspeed
        msg.lateral_velocity = data.twist.twist.linear.y
        msg.heading_rate = self.odomturn
        self.pub_velocity.publish(msg)

        steering_msg = SteeringReport()
        steering_msg.stamp = self.get_clock().now().to_msg()
        if self.odomspeed!=0:
            steering_msg.steering_tire_angle = math.atan(self.odomturn*self.wheel_base/self.odomspeed)
        elif self.odomspeed==0:
            steering_msg.steering_tire_angle = 0.0
        # print("autoware_wheeltec_interface steering_msg %f" , steering_msg.steering_tire_angle)
        self.pub_steering.publish(steering_msg)

        control_msg = ControlModeReport()
        control_msg.stamp = self.get_clock().now().to_msg()
        control_msg.mode = self.automode
        self.pub_control_mode.publish(control_msg)


def main(args=None):
    rclpy.init(args=args)
    converter_node = Autoware_Wheeltec()
    converter_node.get_logger().info("autoware_wheeltec_interface Node starts")

    rclpy.spin(converter_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    converter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
