#! /usr/bin/env python

import rospy
import random

def planner(start_index, goal_index, width, height, costmap, resolution, origin, grid_viz)
    
    open_list = []

    closed_list = set()

    parents = dict()

current_index = start_index
open_list.append(current_index)
while open_list:
    random.shuffle(open_list)
    current_index = open_list.pop(0)

    closed_list.add(current_index)

    grid_viz.set_color(current_index, "pale yellow")

    # Kiểm tra xem đã đến đích chưa
    if current_index == goal_index:
        break

    # Xử lý các điểm láng giềng
    neighbors = find_neighbors(current_index, width, height, costmap, resolution)
    for neighbor_index, _ in neighbors:
        if neighbor_index in closed_list:
            continue
        parents[neighbor_index] = current_index

        open_list.append(neighbor_index)

        # Tính toán chi phí từ điểm bắt đầu đến láng giềng
        tentative_g_score = g_score[current_index] + distance(current_index, neighbor)

        

# Tạo đường đi từ điểm đích đến điểm bắt đầu
path = []
current_index = goal_index
path.append(goal_index)
while current_index != start_index:
    current_index = parents[current_index]
    path.append(current_index)
    
path.reverse()  # Đảo ngược đường đi để bắt đầu từ điểm bắt đầu

# Trả về đường đi
return path[::-1]
