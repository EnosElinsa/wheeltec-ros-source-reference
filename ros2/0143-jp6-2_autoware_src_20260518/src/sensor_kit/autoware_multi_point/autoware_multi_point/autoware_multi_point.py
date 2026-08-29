import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Bool, Empty
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from autoware_adapi_v1_msgs.msg import RouteState,OperationModeState
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import time
import threading



class Autoware_Wheeltec(Node):
    def __init__(self):
        super().__init__('autoware')
        # Publishers
        self.client_auto = self.create_client(ChangeOperationMode,'/api/operation_mode/change_to_autonomous')
        self.client_enable = self.create_client(ChangeOperationMode,'/api/operation_mode/enable_autoware_control')
        self.client_disable = self.create_client(ChangeOperationMode,'/api/operation_mode/disable_autoware_control')
        self.sub1 = self.create_subscription(RouteState, '/api/routing/state', self.callback_routing, 10)
        self.sub2 = self.create_subscription(PoseStamped, '/planning/mission_planning/goal', self.callback_pose, 10)
        self.sub1 = self.create_subscription(OperationModeState, '/api/operation_mode/state', self.callback_operation_mode, 10)
        #创建位置标记话题发布者
        self.marker_pub   = self.create_publisher(MarkerArray,'/autoware_multi_point_marker',   10) 
        self.pose_pub = self.create_publisher(PoseStamped, '/planning/mission_planning/goal', 10)
        while not self.client_auto.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available,waiting')
        
        self.routing_state = 1        #UNKNOWN = 0  UNSET = 1  SET = 2  ARRIVED = 3  CHANGING = 4
        self.markerArray = MarkerArray()
        self.marker_pose  = Marker() #创建marker对象
        
        self.makesure = "0"   #input keyboard mode
        
        self.send_pose_id = 0   #the next pose id will be sended
        
        self.auto_state = False   #auto mode can be clicked
        
        self.auto_mode = 0    #current opertation mode
        
        rate = self.create_rate(1)

    
        thread = threading.Thread(target=self.spin_node, daemon=True)
        thread.start()
        threadkey = threading.Thread(target=self.key_node, daemon=True)
        threadkey.start()
        while rclpy.ok():
            if self.makesure == "1":
                self.markerArray.markers.append(self.marker_pose) #添加元素进数组
                idm = 0
                for m in self.markerArray.markers:
                    m.id = idm
                    idm += 1
                self.marker_pub.publish(self.markerArray)
                self.makesure = "0"
            elif self.makesure == "2":
                self.clear_rviz()
                self.makesure = "0"
            elif self.makesure == "3" or self.makesure == "4":
                self.auto_run()
            elif self.makesure == "5":
                self.set_stop()
                self.send_pose_id = self.send_pose_id - 1
                if self.send_pose_id<0:
                    self.send_pose_id = 0
                self.makesure = "0"
            time.sleep(0.5)
       
            

    def spin_node(self):
        while(1):
            rclpy.spin_once(self,timeout_sec=0.2)
            
    def key_node(self):
        while(1):
            sure = str(input("添加当前目标点:1  清楚所有目标点:2  循环一次:3  一直循环:4  暂停循环:5："))
            if sure == "3" or sure == "4":
                self.send_first_pos()
            self.makesure = sure
            time.sleep(2)

    def callback_routing(self,msg):
        self.routing_state = msg.state
        #print("routing_state")
        #print(self.routing_state)

    def callback_operation_mode(self,msg):
        self.auto_state = msg.is_autonomous_mode_available
        self.auto_mode = msg.mode    #automode is 2
        #print("self.auto_state")
        #print(self.auto_state)

    def callback_pose(self,msg):
        marker_shape  = Marker() #创建marker对象
        marker_shape.id = 0 #必须赋值id
        marker_shape.header.frame_id = 'map' #以哪一个TF坐标为原点
        marker_shape.type = Marker.ARROW #TEXT_VIEW_FACING #一直面向屏幕的字符格式
        marker_shape.action = Marker.ADD #添加marker
        marker_shape.scale.x = 0.7 #marker大小
        marker_shape.scale.y = 0.2 #marker大小
        marker_shape.scale.z = 0.05 #marker大小，对于字符只有z起作用
        marker_shape.pose.position.x = msg.pose.position.x#字符位置
        marker_shape.pose.position.y = msg.pose.position.y #字符位置
        marker_shape.pose.position.z = 0.1 #msg.position.z #字符位置
        marker_shape.pose.orientation.z = msg.pose.orientation.z #字符位置
        marker_shape.pose.orientation.w = msg.pose.orientation.w #字符位置
        marker_shape.color.r = 1.0 #字符颜色R(红色)通道
        marker_shape.color.g = 0.0 #字符颜色G(绿色)通道
        marker_shape.color.b = 0.0 #字符颜色B(蓝色)通道
        marker_shape.color.a = 1.0 #字符透明度
        self.marker_pose = marker_shape
        
    def clear_rviz(self):
        markers = MarkerArray()
        for m in self.markerArray.markers:
            m.action = Marker.DELETEALL
        self.marker_pub.publish(self.markerArray)
        self.markerArray = markers
    def send_first_pos(self):
        #self.send_pose_id = 0
        if self.send_pose_id <len(self.markerArray.markers):
            send_pose = PoseStamped()
            send_pose.header.frame_id = 'map'
            send_pose.header.stamp = self.get_clock().now().to_msg()
            send_pose.pose.position.x = self.markerArray.markers[self.send_pose_id].pose.position.x
            send_pose.pose.position.y = self.markerArray.markers[self.send_pose_id].pose.position.y
            send_pose.pose.orientation.z = self.markerArray.markers[self.send_pose_id].pose.orientation.z
            send_pose.pose.orientation.w = self.markerArray.markers[self.send_pose_id].pose.orientation.w
            self.pose_pub.publish(send_pose)
            self.send_pose_id += 1
            
    def auto_run(self):
        if self.routing_state == 2 and self.auto_mode != 2:
            #print("self.set_AUTO")
            self.set_auto()
        elif self.routing_state == 3:
            #print("self.send_pose_id")
            #print(self.send_pose_id)
            #print("len(self.markerArray.markers")
            #print(len(self.markerArray.markers))
            if self.send_pose_id <len(self.markerArray.markers):
                send_pose = PoseStamped()
                send_pose.header.frame_id = 'map'
                send_pose.header.stamp = self.get_clock().now().to_msg()
                send_pose.pose.position.x = self.markerArray.markers[self.send_pose_id].pose.position.x
                send_pose.pose.position.y = self.markerArray.markers[self.send_pose_id].pose.position.y
                send_pose.pose.orientation.z = self.markerArray.markers[self.send_pose_id].pose.orientation.z
                send_pose.pose.orientation.w = self.markerArray.markers[self.send_pose_id].pose.orientation.w
                self.pose_pub.publish(send_pose)
                time.sleep(5)
                self.send_pose_id += 1
            elif self.makesure == "4":
                self.send_pose_id = 0
    
    def set_auto(self):
        while not self.auto_state:
            time.sleep(1)
            print("AUTO mode is currently unavailable for clicking!")
        req = ChangeOperationMode.Request()
        future = self.client_auto.call_async(req)
        rclpy.spin_until_future_complete(self,future)
        
        req = ChangeOperationMode.Request()
        future = self.client_enable.call_async(req)
        rclpy.spin_until_future_complete(self,future)
        
    def set_stop(self):
        req = ChangeOperationMode.Request()
        future = self.client_disable.call_async(req)
        rclpy.spin_until_future_complete(self,future)

def main(args=None):
    rclpy.init(args=args)
    converter_node = Autoware_Wheeltec()
    converter_node.get_logger().info("autoware Node starts")

    rclpy.spin(converter_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    converter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
