import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import time
import math
import cv2


def create_temporal_map(occupancy_map, last_update_time, current_time):
    temporal_map = np.zeros_like(occupancy_map, dtype=np.float32)
    lambda_decay = 5.0
    for x in range(occupancy_map.shape[0]):
        for y in range(occupancy_map.shape[1]):
            t_cell = last_update_time[x, y]
            delta_t = current_time - t_cell
            cT = math.exp(-delta_t / lambda_decay)
            temporal_map[x, y] = abs(occupancy_map[x, y] - 0.5) * cT + 0.5
    return temporal_map

def extract_dynamic_objects(occupancy_map):
    binary_map = (occupancy_map == 1).astype(np.uint8) * 255
    contours, _ = cv2.findContours(binary_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    dynamic_objects = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 20 <= area <= 300:
            dynamic_objects.append(cnt)
    return dynamic_objects

def filter_dynamic_objects(occupancy_map, temporal_map, dynamic_objects):
    prob_map = occupancy_map.copy()
    for obj in dynamic_objects:
        coords = obj.squeeze()
        if len(coords.shape) != 2:
            continue
        times = []
        for x, y in coords:
            x = np.clip(int(x), 0, occupancy_map.shape[0] - 1)
            y = np.clip(int(y), 0, occupancy_map.shape[1] - 1)
            times.append(temporal_map[x, y])
        if not times:
            continue
        avg_time = np.mean(times)
        p_d = 2 * abs(avg_time - 0.5)
        if p_d < 0.5:
            for x, y in coords:
                x = np.clip(int(x), 0, occupancy_map.shape[0] - 1)
                y = np.clip(int(y), 0, occupancy_map.shape[1] - 1)
                prob_map[x, y] = 0
    return prob_map

def merge_maps(map1, map2):
    resolution = map1.info.resolution
    if resolution != map2.info.resolution:
        raise ValueError("Map resolutions do not match!")

    # 맵 크기 및 데이터 추출
    w1, h1 = map1.info.width, map1.info.height
    w2, h2 = map2.info.width, map2.info.height
    data1 = np.array(map1.data, dtype=np.int8).reshape((h1, w1))
    data2 = np.array(map2.data, dtype=np.int8).reshape((h2, w2))

    # origin 좌표
    ox1, oy1 = map1.info.origin.position.x, map1.info.origin.position.y
    ox2, oy2 = map2.info.origin.position.x, map2.info.origin.position.y

    # 전체 병합 영역 계산
    min_x = min(ox1, ox2)
    min_y = min(oy1, oy2)
    max_x = max(ox1 + w1 * resolution, ox2 + w2 * resolution)
    max_y = max(oy1 + h1 * resolution, oy2 + h2 * resolution)

    new_w = int(np.ceil((max_x - min_x) / resolution))
    new_h = int(np.ceil((max_y - min_y) / resolution))
    merged_data = np.full((new_h, new_w), -1, dtype=np.int8)

    # 현재 시간 기준 더미 시간 업데이트 맵
    current_time = time.time()
    delta_t = 3.0  # 최근 업데이트 간격 가정
    lambda_decay = 5.0
    last_update1 = np.full((h1, w1), current_time - delta_t)
    last_update2 = np.full((h2, w2), current_time - delta_t)

    # 시간 신뢰도 계산
    cT1 = create_temporal_map(data1, last_update1, current_time)
    cT2 = create_temporal_map(data2, last_update2, current_time)

    # 동적 객체 필터링
    dyn_objs1 = extract_dynamic_objects(data1)
    dyn_objs2 = extract_dynamic_objects(data2)
    prob1 = filter_dynamic_objects(data1, cT1, dyn_objs1)
    prob2 = filter_dynamic_objects(data2, cT2, dyn_objs2)

    # 삽입 위치 계산
    x1, y1 = int((ox1 - min_x) / resolution), int((oy1 - min_y) / resolution)
    x2, y2 = int((ox2 - min_x) / resolution), int((oy2 - min_y) / resolution)

    for i in range(h1):
        for j in range(w1):
            val = prob1[i, j]
            if val == -1:
                continue
            merged_data[y1 + i, x1 + j] = val

    for i in range(h2):
        for j in range(w2):
            val = prob2[i, j]
            if val == -1:
                continue
            mx, my = x2 + j, y2 + i
            if merged_data[my, mx] == -1:
                merged_data[my, mx] = val
            else:
                # 논문식 병합: max(P1 * cT1, P2 * cT2)
                p1 = merged_data[my, mx]
                p2 = val
                ct1 = math.exp(-delta_t / lambda_decay)
                ct2 = math.exp(-delta_t / lambda_decay)
                merged_val = int(max(p1 * ct1, p2 * ct2))
                merged_data[my, mx] = merged_val

    # 메시지 생성
    merged_map = OccupancyGrid()
    merged_map.header.frame_id = 'merge_map'
    merged_map.info.resolution = resolution
    merged_map.info.width = new_w
    merged_map.info.height = new_h
    merged_map.info.origin.position.x = min_x
    merged_map.info.origin.position.y = min_y
    merged_map.data = merged_data.flatten().tolist()

    return merged_map


class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')
        self.publisher = self.create_publisher(OccupancyGrid, '/merge_map', 10)
        self.subscription1 = self.create_subscription(OccupancyGrid, '/tb3_0/map', self.map1_callback, 10)
        self.subscription2 = self.create_subscription(OccupancyGrid, '/tb3_1/map', self.map2_callback, 10)
        self.map1 = None
        self.map2 = None

    def map1_callback(self, msg):
        self.map1 = msg
        if self.map2 is not None:
            merged = merge_maps(self.map1, self.map2)
            self.publisher.publish(merged)

    def map2_callback(self, msg):
        self.map2 = msg
        if self.map1 is not None:
            merged = merge_maps(self.map1, self.map2)
            self.publisher.publish(merged)

def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MergeMapNode()
    rclpy.spin(merge_map_node)
    merge_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

