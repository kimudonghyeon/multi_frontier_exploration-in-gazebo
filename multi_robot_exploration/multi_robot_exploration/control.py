import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid , Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np
import heapq , math , time , threading
import scipy.interpolate as si
import datetime
from tf2_ros import Buffer, TransformListener, LookupException
import random

lookahead_distance = 0.25
speed = 0.15
expansion_size = 3
target_error = 0.15

TB0_PATH = [(0,0)]
TB0_PATHF = 0
TB1_PATH = [(0,0)]
TB1_PATHF = 0
VISITED = []
MAX_DIST = 5.0
MIN_DIST = 1.0


def get_time_string():
    now = datetime.datetime.now()
    return now.strftime("%H:%M:%S")

def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def astar(array, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data = data + [start]
            data = data[::-1]
            return data
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    continue
            else:
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    if goal not in came_from:
        closest_node = None
        closest_dist = float('inf')
        for node in close_set:
            dist = heuristic(node, goal)
            if dist < closest_dist:
                closest_node = node
                closest_dist = dist
        if closest_node is not None:
            data = []
            while closest_node in came_from:
                data.append(closest_node)
                closest_node = came_from[closest_node]
            data = data + [start]
            data = data[::-1]
            return data
    return False

def bspline_planning(array, sn):
    try:
        array = np.array(array)
        x = array[:, 0]
        y = array[:, 1]
        N = 2
        t = range(len(x))
        x_tup = si.splrep(t, x, k=N)
        y_tup = si.splrep(t, y, k=N)
        x_list = list(x_tup)
        xl = x.tolist()
        x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]
        y_list = list(y_tup)
        yl = y.tolist()
        y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]
        ipl_t = np.linspace(0.0, len(x) - 1, sn)
        rx = si.splev(ipl_t, x_list)
        ry = si.splev(ipl_t, y_list)
        path = [(rx[i],ry[i]) for i in range(len(rx))]
    except:
        path = array
    return path

def frontierB(matrix):
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            if matrix[i][j] == 0.0:
                if i > 0 and matrix[i-1][j] < 0:
                    matrix[i][j] = 2
                elif i < len(matrix)-1 and matrix[i+1][j] < 0:
                    matrix[i][j] = 2
                elif j > 0 and matrix[i][j-1] < 0:
                    matrix[i][j] = 2
                elif j < len(matrix[i])-1 and matrix[i][j+1] < 0:
                    matrix[i][j] = 2
    return matrix

def assign_groups(matrix):
    group = 1
    groups = {}
    for i in range(len(matrix)):
        for j in range(len(matrix[0])):
            if matrix[i][j] == 2:
                group = dfs(matrix, i, j, group, groups)
    return matrix, groups

def dfs(matrix, i, j, group, groups):
    if i < 0 or i >= len(matrix) or j < 0 or j >= len(matrix[0]):
        return group
    if matrix[i][j] != 2:
        return group
    if group in groups:
        groups[group].append((i, j))
    else:
        groups[group] = [(i, j)]
    matrix[i][j] = 0
    dfs(matrix, i + 1, j, group, groups)
    dfs(matrix, i - 1, j, group, groups)
    dfs(matrix, i, j + 1, group, groups)
    dfs(matrix, i, j - 1, group, groups)
    dfs(matrix, i + 1, j + 1, group, groups)
    dfs(matrix, i - 1, j - 1, group, groups)
    dfs(matrix, i - 1, j + 1, group, groups) 
    dfs(matrix, i + 1, j - 1, group, groups) 
    return group + 1

def fGroups(groups):
    sorted_groups = sorted(groups.items(), key=lambda x: len(x[1]), reverse=True)
    top_five_groups = [g for g in sorted_groups[:5] if len(g[1]) > 2]    
    return top_five_groups

def calculate_centroid(x_coords, y_coords):
    n = len(x_coords)
    sum_x = sum(x_coords)
    sum_y = sum(y_coords)
    mean_x = sum_x / n
    mean_y = sum_y / n
    centroid = (int(mean_x), int(mean_y))
    return centroid

def visitedControl(targetP):
    global VISITED
    for i in range(len(VISITED)):
        k = VISITED[i]
        d = math.sqrt((k[0] - targetP[0])**2 + (k[1] - targetP[1])**2)
        if d < 0.3:
            return 1
    return 0

def findClosestGroup(matrix, groups, current, resolution, originX, originY, choice, self):
    global TB0_PATH, TB1_PATH, TB0_PATHF, TB1_PATHF
    targetP = None
    arrays = []
    other_robot_pos = (self.tb1_x, self.tb1_y) if choice == 0 else (self.tb0_x, self.tb0_y)
    for i in range(len(groups)):
        if len(groups[i][1]) < 3:
            continue 
        middle = calculate_centroid([p[0] for p in groups[i][1]], [p[1] for p in groups[i][1]]) 
        if current == middle:  # 같은 위치면 무시
            continue
        goal_world = (middle[1]*resolution + originX, middle[0]*resolution + originY)
        frontier_dist_to_other = math.hypot(goal_world[0] - other_robot_pos[0],goal_world[1] - other_robot_pos[1])
        if frontier_dist_to_other < MIN_DIST or frontier_dist_to_other > MAX_DIST:
            continue
        if visitedControl(goal_world):
            continue
        path = astar(matrix, current, middle)
        if not path or len(path) < 2:
            continue
        path = [(p[1]*resolution + originX, p[0]*resolution + originY) for p in path]
        dist = pathLength(path)
        if dist < 0.3:  # 너무 짧은 경로는 무시
            continue
        arrays.append((len(groups[i][1]), dist, path))
    if not arrays:
        return None
    arrays.sort(key=lambda x: x[1])  # 거리 기준 오름차순
    arrays_a = [a for a in arrays if a[1] > target_error * 2]  # 최소 이동거리 필터
    p1 = sorted([a for a in arrays_a if a[1] < 2.0], key=lambda x: x[0], reverse=True)
    if p1:
        return p1[0][2]
    p1 = sorted([a for a in arrays_a if a[1] < 4.0], key=lambda x: x[0], reverse=True)
    if p1:
        return p1[0][2]
    arrays.sort(key=lambda x: x[0], reverse=True)
    return arrays[0][2]  # 최후 수단: 가장 큰 그룹


def pathLength(path):
    for i in range(len(path)):
        path[i] = (path[i][0],path[i][1])
        points = np.array(path)
    differences = np.diff(points, axis=0)
    distances = np.hypot(differences[:,0], differences[:,1])
    total_distance = np.sum(distances)
    return total_distance

def costmap(data,width,height,resolution, other_robot_pos):
    data = np.array(data).reshape(height,width)
    wall = np.where(data == 100)
    for i in range(-expansion_size,expansion_size+1):
        for j in range(-expansion_size,expansion_size+1):
            if i  == 0 and j == 0:
                continue
            x = wall[0]+i
            y = wall[1]+j
            x = np.clip(x,0,height-1)
            y = np.clip(y,0,width-1)
            data[x,y] = 100
    data = data*resolution
    return data

def exploration(data, width, height, resolution, column, row, originX, originY, choice, self, fail_count): # 경로, 도착 지점 안전거리 50cm
    global TB0_PATH, TB1_PATH, TB0_PATHF, TB1_PATHF
    f = 1
    other_robot_pos = (self.tb1_x, self.tb1_y) if choice == 0 else (self.tb0_x, self.tb0_y)
    robot_name = "TB3_0" if choice == 0 else "TB3_1"
    data = costmap(data, width, height, resolution, other_robot_pos)
    binary = (data >= 0.5).astype(int)
    if row < 0 or row >= height or column < 0 or column >= width:
        print(f"[{get_time_string()}] [ERROR] Robot 위치 인덱스 초과: row={row}, column={column}, height={height}, width={width}")
        return 0
    data[row][column] = 0
    data[data > 5] = 1
    data = frontierB(data)
    data, groups = assign_groups(data)
    groups = fGroups(groups)
    path = None
    if len(groups) == 0:
        f = -1
    else:
        data[data < 0] = 1
        path = findClosestGroup(data, groups, (row, column), resolution, originX, originY, choice, self)
        if path is not None:
            path = bspline_planning(path, len(path) * 5)
            path_dist_to_other = min(math.hypot(p[0] - other_robot_pos[0], p[1] - other_robot_pos[1]) for p in path)
            if path_dist_to_other < 0.5:
                self.get_logger().warn(f"[{get_time_string()}] [{robot_name}] 경로가 정지 로봇과 너무 가까움 ({path_dist_to_other:.2f}m)")
                path = None
        else:
            f = -1
    if path is not None and len(path) > 1:
        target_pos = path[-1]
        goal_dist_to_other = math.hypot(target_pos[0] - other_robot_pos[0], target_pos[1] - other_robot_pos[1])
        if goal_dist_to_other < 0.5:
            self.get_logger().warn(f"[{get_time_string()}] [{robot_name}] 목표지점이 다른 로봇과 너무 가까움 ({goal_dist_to_other:.2f}m)")
            path = None
    failed = (f == -1) or (path is None) or (len(path) < 1)    
    if failed:
        fail_count += 1
        if f == -1:
            self.get_logger().warn(f"[{get_time_string()}] [{robot_name}] 경로 실패 {fail_count}회: No frontier")
        elif path is None:
            self.get_logger().warn(f"[{get_time_string()}] [{robot_name}] 경로 실패 {fail_count}회: Path is None")
        elif len(path) < 1:
            self.get_logger().warn(f"[{get_time_string()}] [{robot_name}] 경로 실패 {fail_count}회: len={len(path)} < 1")
        if fail_count >= 5:
            self.get_logger().warn(f"[{get_time_string()}] [{robot_name}] 5회 연속 실패 → Fallback 시도")
            curr_x = self.tb0_x if choice == 0 else self.tb1_x
            curr_y = self.tb0_y if choice == 0 else self.tb1_y
            col = self.c_tb0 if choice == 0 else self.c_tb1
            row = self.r_tb0 if choice == 0 else self.r_tb1
            fallback_path = find_fallback_point(curr_x, curr_y, binary, originX, originY, resolution, np.array(self.data).reshape(self.height, self.width))
            if fallback_path is not None:
                if choice == 0:
                    TB0_PATH = fallback_path
                    TB0_PATHF = 1
                    self.tb0_path_pub()
                    self.tb0_s = False
                else:
                    TB1_PATH = fallback_path
                    TB1_PATHF = 1
                    self.tb1_path_pub()
                    self.tb1_s = False
                print(f"[DEBUG] fallback_path length = {len(fallback_path)}")
                print(f"[DEBUG] fallback_path[0:3] = {fallback_path[:3]}")
                print(f"[DEBUG] fallback_path[-3:] = {fallback_path[-3:]}")
                print(f"[{get_time_string()}] [{robot_name}] fallback 경로 생성 완료")
                return -1
            else:
                return -2
        else:
            if choice == 0:
                TB0_PATHF = -1
            else:
                TB1_PATHF = -1
            return fail_count
    # 성공적인 경우
    if choice == 0:
        if TB0_PATHF == 0:
            TB0_PATH = path
            TB0_PATHF = 1
            self.tb0_path_pub()
    else:
        if TB1_PATHF == 0:
            TB1_PATH = path
            TB1_PATHF = 1
            self.tb1_path_pub()
    return 0    

def find_fallback_point(x, y, binary_map, originX, originY, resolution, explored_map): # 0.3m 이상, 1.0m 이하
    H, W = binary_map.shape
    free_idx = np.argwhere((explored_map == 0) & (binary_map == 0))
    print(f"[DEBUG] free_idx length = {len(free_idx)}")
    if free_idx.size == 0:
        print(f"[{get_time_string()}] [DEBUG] fallback 후보 없음: free 셀 자체가 없음")
        return None
    occ = binary_map.copy()
    occ[explored_map != 0] = 1
    np.random.shuffle(free_idx)
    for r, c in free_idx:
        wx = c * resolution + originX
        wy = r * resolution + originY
        d = math.hypot(wx - x, wy - y)
        if not (0.3 <= d <= 1.0):
            continue
        start_rc = (int((y - originY) / resolution), int((x - originX) / resolution))
        goal_rc  = (r, c)
        path_cells = astar(occ, start_rc, goal_rc)
        if not path_cells:
            continue
        path_world = [(col * resolution + originX, row * resolution + originY) for row, col in path_cells]
        try:
            smooth = bspline_planning(path_world, len(path_world) * 5)
        except:
            smooth = path_world
        clean = [(float(px), float(py)) for px, py in smooth]
        return clean
    print(f"[{get_time_string()}] [DEBUG] fallback 후보 없음: 0.3m 이상, 1.0m 이하인 곳 없음")
    return None

def calculate_distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


class HeadquartersControl(Node):
    def __init__(self):
        super().__init__('Exploration')
        self.subscription = self.create_subscription(OccupancyGrid,'merge_map',self.map_callback,10)
        self.publisher_tb3_0_path = self.create_publisher(Float32MultiArray,'tb3_0/path', 10)
        self.publisher_tb3_1_path = self.create_publisher(Float32MultiArray,'tb3_1/path', 10)
        self.subscription_tb3_0_odom = self.create_subscription(Odometry,'tb3_0/odom',self.tb0_odom_callback,10)
        self.subscription_tb3_1_odom = self.create_subscription(Odometry,'tb3_1/odom',self.tb1_odom_callback,10)
        self.subscription_tb3_0_cmd_vel = self.create_subscription(Twist,'tb3_0/cmd_vel',self.tb0_status_control,4)
        self.subscription_tb3_1_cmd_vel = self.create_subscription(Twist,'tb3_1/cmd_vel',self.tb1_status_control,4)
        print(f"[{get_time_string()}] 탐색 모드 활성화")
        self.kesif = True
        threading.Thread(target=lambda: self.start_exploration(0)).start()
        threading.Thread(target=lambda: self.start_exploration(1)).start()
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  
        self.turn = 0
        self.tb0_s = False
        self.tb1_s = False   
        self.cmd_pub_tb3_0 = self.create_publisher(Twist, 'tb3_0/cmd_vel', 10)
        self.cmd_pub_tb3_1 = self.create_publisher(Twist, 'tb3_1/cmd_vel', 10)
        

    def start_exploration(self, choice):
        global TB0_PATHF, TB1_PATHF, TB0_PATH, TB1_PATH
        tb_fail_count = 0
        other_choice = 1 - choice
        while True:
            if not hasattr(self, 'map_data') or not hasattr(self, 'tb0_x') or not hasattr(self, 'tb1_x'):
                time.sleep(0.1)
                continue
            if self.turn != choice:
                time.sleep(0.1)
                continue
            self.c_tb0 = np.clip(int((self.tb0_x - self.originX)/self.resolution), 0, self.width-1)
            self.r_tb0 = np.clip(int((self.tb0_y - self.originY)/self.resolution), 0, self.height-1)
            self.c_tb1 = np.clip(int((self.tb1_x - self.originX)/self.resolution), 0, self.width-1)
            self.r_tb1 = np.clip(int((self.tb1_y - self.originY)/self.resolution), 0, self.height-1)
            
            path_flag = TB0_PATHF if choice == 0 else TB1_PATHF
            stopped = self.tb0_s if choice == 0 else self.tb1_s
            robot = f"TB3_{choice}"
            if path_flag == 1 and not stopped:
                time.sleep(0.05)
                continue
            
            result = exploration(
                self.data, self.width, self.height, self.resolution,
                self.c_tb0 if choice == 0 else self.c_tb1,
                self.r_tb0 if choice == 0 else self.r_tb1,
                self.originX, self.originY, choice, self, tb_fail_count
            )
            # return is (-1 fallback success, 0 path creation success) / (-2 fallback fail) / (1, 2 cumulative fail)
            if result == 0:
                self.stop_robot(other_choice)
                start_wait = time.time()
                timeout = 5.0
                retreat_mode = False
                retreat_target = None
                start_x = self.tb0_x if choice == 0 else self.tb1_x
                start_y = self.tb0_y if choice == 0 else self.tb1_y
                path_history = [(start_x, start_y)]
                if choice == 0:
                    self.tb0_s = False
                else:
                    self.tb1_s = False
                while True:
                    finished = (choice == 0 and self.tb0_s) or (choice == 1 and self.tb1_s)
                    if retreat_mode:
                        target = retreat_target
                    else:
                        target = TB0_PATH[-1] if choice == 0 else TB1_PATH[-1]
                    curr_x = self.tb0_x if choice == 0 else self.tb1_x
                    curr_y = self.tb0_y if choice == 0 else self.tb1_y
                    last_x, last_y = path_history[-1]
                    if calculate_distance(curr_x, curr_y, last_x, last_y) > 0.05 and not retreat_mode:
                        path_history.append((curr_x, curr_y))
                    dist_to_target = calculate_distance(curr_x, curr_y, target[0], target[1])
                    DIST = calculate_distance(self.tb0_x, self.tb0_y, self.tb1_x, self.tb1_y)
                    if not retreat_mode and DIST >= MAX_DIST - 0.3 and len(path_history) > 1:
                        self.get_logger().warn(f"[{get_time_string()}] [{robot}] 거리 제한 초과({DIST:.2f}m) → 후퇴")
                        reverse_path = list(reversed(path_history))
                        half_idx = len(reverse_path) // 2
                        retreat_path = reverse_path[:half_idx+1]
                        # publish
                        if choice == 0:
                            TB0_PATH  = retreat_path
                            TB0_PATHF = 1
                            self.tb0_s = False
                            self.tb0_path_pub()
                        else:
                            TB1_PATH  = retreat_path
                            TB1_PATHF = 1
                            self.tb1_s = False
                            self.tb1_path_pub()
                        retreat_mode   = True
                        retreat_target = retreat_path[-1]
                        start_wait = time.time()
                    if not retreat_mode and finished and dist_to_target < 0.05:
                        break
                    if retreat_mode and dist_to_target < 0.05:
                        break
                    elapsed = time.time() - start_wait
                    if elapsed > timeout and finished:
                        break
                    if elapsed > 100:
                        self.stop_robot(choice)
                        break
                    time.sleep(0.1)
                self.stop_robot(choice)
                #self.stop_robot(other_choice)
                print(f"[{get_time_string()}] [{robot}] 탐색 종료")
                self.turn = other_choice
                TB0_PATHF = TB1_PATHF = 0
                tb_fail_count = 0
                time.sleep(1)
                continue
            elif result == -1:
                start_wait = time.time()
                timeout = 5.0
                while True:
                    finished = (choice == 0 and self.tb0_s) or (choice == 1 and self.tb1_s)
                    target = TB0_PATH[-1] if choice == 0 else TB1_PATH[-1]
                    curr_x = self.tb0_x if choice == 0 else self.tb1_x
                    curr_y = self.tb0_y if choice == 0 else self.tb1_y
                    dist_to_target = calculate_distance(curr_x, curr_y, target[0], target[1])
                    if dist_to_target < 0.01:
                        break
                    elapsed = time.time() - start_wait
                    if elapsed > timeout and finished:
                        break
                    if elapsed > 15:
                        self.stop_robot(choice)
                        break
                    time.sleep(0.1)
                print(f"[{get_time_string()}] [{robot}] 탐색 종료")
                self.turn = other_choice
                TB0_PATHF = TB1_PATHF = 0
                tb_fail_count = 0
                time.sleep(1)
                continue
            elif result == -2:
                self.get_logger().warn(f"[{get_time_string()}] [{robot}] fallback 실패 → 턴 전환")
                self.stop_robot(other_choice)
                self.turn = other_choice
                TB0_PATHF = TB1_PATHF = 0
                tb_fail_count = 0
                time.sleep(1)
                continue
            else:
                tb_fail_count = result

    def map_callback(self,msg):
        global TB0_PATHF, TB1_PATHF
        self.map_data = msg
        self.resolution = self.map_data.info.resolution
        self.originX = self.map_data.info.origin.position.x
        self.originY = self.map_data.info.origin.position.y
        self.width = self.map_data.info.width
        self.height = self.map_data.info.height
        self.data = self.map_data.data

    def tb0_odom_callback(self,msg):
        self.tb0_x = msg.pose.pose.position.x
        self.tb0_y = msg.pose.pose.position.y

    def tb1_odom_callback(self,msg):
        self.tb1_x = msg.pose.pose.position.x
        self.tb1_y = msg.pose.pose.position.y  

    def tb0_status_control(self,msg):
        if msg.linear.x == 0 and msg.angular.z == 0:
            if not self.tb0_s:
                self.tb0_s = True
        else:
            self.tb0_s = False 
    
    def tb1_status_control(self,msg):
        if msg.linear.x == 0 and msg.angular.z == 0:
            if not self.tb1_s:
                self.tb1_s = True
        else:
            self.tb1_s = False

    def tb0_path_pub(self):
        global TB0_PATH
        global VISITED
        VISITED.append(TB0_PATH[-1])
        message = Float32MultiArray()
        try:
            message.data = [float(elem) for tup in TB0_PATH if isinstance(tup, (list, tuple)) and len(tup) == 2 for elem in tup if isinstance(elem, (int, float)) or hasattr(elem, "item")]
            #print(f"[DEBUG] tb0_path_pub: message.data = {message.data}")
        except Exception as e:
            print(f"[ERROR] tb0_path_pub 예외: {e}")
            return
        self.publisher_tb3_0_path.publish(message)
        t = pathLength(TB0_PATH)/speed
        t = max(t - 0.2, 0)
        self.t0 = threading.Timer(t, lambda: None)
        self.t0.start()
    
    def tb1_path_pub(self):
        global TB1_PATH
        global VISITED
        VISITED.append(TB1_PATH[-1])
        message = Float32MultiArray()
        try:
            message.data = [float(elem) for tup in TB1_PATH if isinstance(tup, (list, tuple)) and len(tup) == 2 for elem in tup if isinstance(elem, (int, float)) or hasattr(elem, "item")]
            #print(f"[DEBUG] tb1_path_pub: message.data = {message.data}")
        except Exception as e:
            print(f"[ERROR] tb1_path_pub 예외: {e}")
            return
        self.publisher_tb3_1_path.publish(message)
        t = pathLength(TB1_PATH)/speed
        t = max(t - 0.2, 0)
        self.t1 = threading.Timer(t, lambda: None)
        self.t1.start()
    
    def stop_robot(self, choice):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        for i in range(10):
            if choice == 0:
                self.cmd_pub_tb3_0.publish(msg)
            else:
                self.cmd_pub_tb3_1.publish(msg)
            time.sleep(0.05)
    
def main(args=None):
    rclpy.init(args=args)
    headquarters_control = HeadquartersControl()
    rclpy.spin(headquarters_control)
    headquarters_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
