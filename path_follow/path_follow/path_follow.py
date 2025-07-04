import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading
import  math , time
from sensor_msgs.msg import LaserScan

lookahead_distance = 0.25
speed = 0.15
robot_r = 0.2

def euler_from_quaternion(x,y,z,w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def find_nearest_point(x, y, path):
    min_dist = float('inf')
    nearest = path[-1]
    idx = len(path) - 1
    for i, pt in enumerate(path):
        dist = math.hypot(x - pt[0], y - pt[1])
        if dist < min_dist:
            min_dist = dist
            nearest = pt
            idx = i
    return nearest, idx

def pure_pursuit(current_x, current_y, current_heading, path, index):
    global lookahead_distance
    closest_point = None
    v = max(0.05, speed*0.8)
    _, index = find_nearest_point(current_x, current_y, path)
    # 경로 끝까지 보면서 lookahead 거리 이상 떨어진 점 찾기
    for i in range(index, len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y)
        if distance >= lookahead_distance:
            closest_point = (x, y)
            index = i
            break

    # 못 찾으면 마지막 점을 사용
    if closest_point is None:	
        closest_point, index = find_nearest_point(current_x, current_y, path)

    # 헤딩 계산
    target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
    desired_steering_angle = target_heading - current_heading

    # 헤딩 wrap-around 처리
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi

    # steering angle이 너무 크면 제자리 회전만 → 너무 멈추는 경우 방지
    max_steer = math.pi / 4  # 더 유연하게 설정 가능
    if abs(desired_steering_angle) > max_steer:
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = sign * max_steer
        v = 0.05  # 조금만 이동

    return v, desired_steering_angle, index

def localControl(scan):
    v = None
    w = None
    for i in range(60): #60
        if scan[i] < robot_r:
            v = 0.05
            w = -math.pi/4 
            break
    if v == None:
        for i in range(300,360): #300,360
            if scan[i] < robot_r:
                v = 0.05
                w = math.pi/4
                break
    return v,w

class pathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.subscription_path = self.create_subscription(Float32MultiArray,'/path',self.get_path,10)
        self.subscription_odom = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.publisher_visual_path = self.create_publisher(Path, '/visual_path', 10)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription_scan = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        print("Path follower node has been started")
        self.x = None
        self.y = None
        self.yaw = None
        self.path = []
        self.scan = None


    def get_path(self,msg):
        #Rota dinleme
        print("Path has been received")
        print(f"[{self.get_name()}] Path received, length: {len(msg.data)//2}")
        data_list = [msg.data[i] for i in range(len(msg.data))]
        if len(msg.data) % 2 != 0 or len(msg.data) == 0:
            self.get_logger().error("Received path data length is invalid.")
            return
        try:
            reshaped_data_list = [(msg.data[i], msg.data[i+1]) for i in range(0, len(msg.data), 2)]
            self.path = reshaped_data_list
            threading.Thread(target=self.follow_path).start()
        except Exception as e:
            self.get_logger().error(f"Path processing error: {str(e)}")


    
    def follow_path(self):
        twist = Twist()
        path_msg = Path()
        path_msg.header.frame_id = "merge_map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for pt in self.path:
            pose = PoseStamped()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            path_msg.poses.append(pose)    
        def normalize_angle(angle):
            return (angle + math.pi) % (2 * math.pi) - math.pi
        target_x, target_y = self.path[0]
        while True:
            if self.x is None or self.yaw is None or self.scan is None:
                time.sleep(0.05)
                continue
            v_loc, w_loc = localControl(self.scan)
            if v_loc is not None:
                twist.linear.x = v_loc
                twist.angular.z = w_loc
            else:
                dx = target_x - self.x
                dy = target_y - self.y
                desired_yaw = math.atan2(dy, dx)
                error = (desired_yaw - self.yaw + math.pi) % (2*math.pi) - math.pi
                if abs(error) < 0.05:
                    break
                twist.linear.x = 0.0
                twist.angular.z = 0.5 * math.copysign(1.0, error)
            self.publisher_cmd_vel.publish(twist)
            time.sleep(0.1)
        i = 0
        while True:
            if self.x is None or self.y is None or self.yaw is None:
                time.sleep(0.05)
                continue
            if hasattr(self, 'scan'):
                v_loc, w_loc = localControl(self.scan)
            else:
                v_loc, w_loc = None, None
            if v_loc is None:
                v, w, i = pure_pursuit(self.x, self.y, self.yaw, self.path, i)
            else:
                v, w = v_loc, w_loc
            if abs(self.x - self.path[-1][0]) < 0.15 and abs(self.y - self.path[-1][1]) < 0.15:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher_cmd_vel.publish(twist)
                print("Path has been followed")
                break
            twist.linear.x = v
            twist.angular.z = w
            self.publisher_visual_path.publish(path_msg)
            self.publisher_cmd_vel.publish(twist)
            time.sleep(0.1)

    def odom_callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        
    def scan_callback(self, msg: LaserScan):
        self.scan = msg.ranges


def main(args=None):
    rclpy.init(args=args)
    path_follower = pathFollower()
    rclpy.spin(path_follower)
    path_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
