import numpy as np
import time
import math
from shapely.geometry import Polygon
import random

def calculate_path_length(points):
    total_length = 0.0
    
    
    for i in range(len(points) - 1):
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        
        
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        
        
        total_length += distance
    
    return total_length

def perpendicular_distance_from_line(x1, y1, x2, y2, x0, y0):
    
    A = y2 - y1
    B = x1 - x2
    C = x2 * y1 - x1 * y2
    
    
    numerator = abs(A * x0 + B * y0 + C)
    denominator = math.sqrt(A**2 + B**2)
    
    distance = numerator / denominator
    return distance

import numpy as np

def calculate_circle_center_and_radius(p1, p2, p3):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    
    
    mid12 = ((x1 + x2) / 2, (y1 + y2) / 2)
    mid23 = ((x2 + x3) / 2, (y2 + y3) / 2)
    
    
    if y2 - y1 != 0:
        slope_perp_12 = -(x2 - x1) / (y2 - y1)
    else:
        slope_perp_12 = float('inf')  
    
    if y3 - y2 != 0:
        slope_perp_23 = -(x3 - x2) / (y3 - y2)
    else:
        slope_perp_23 = float('inf')  
    
    if slope_perp_12 != float('inf'):
        c12 = mid12[1] - slope_perp_12 * mid12[0]
    else:
        cx = mid12[0]  
    
    if slope_perp_23 != float('inf'):
        c23 = mid23[1] - slope_perp_23 * mid23[0]
    else:
        cx = mid23[0]  
    
    
    if slope_perp_12 != float('inf') and slope_perp_23 != float('inf'):
        cx = (c23 - c12) / (slope_perp_12 - slope_perp_23)
        cy = slope_perp_12 * cx + c12
    elif slope_perp_12 == float('inf'):
        cy = slope_perp_23 * cx + c23
    elif slope_perp_23 == float('inf'):
        cy = slope_perp_12 * cx + c12
    
    # Calculate the radius
    radius = np.sqrt((cx - x1)**2 + (cy - y1)**2)
    
    return (cx, cy), radius

def points_on_same_circle(points):
    """
    Check if all points lie on the same circle.
    """
    if len(points) < 3:
        return False, None, None
    
    
    point_list = [
         [points[len(points)//4], points[len(points)//2], points[3*(len(points)//4)]],
         [points[0], points[-10], points[-1]],
         [points[0], points[10], points[-1]]
    ]
    distance_points = calculate_path_length(points)
    # print(f"Distance of points: {distance_points}")
    for chosen_item in point_list:
        # print("new starts here ---------------------------------->")
        center, radius = calculate_circle_center_and_radius(chosen_item[0], chosen_item[1], chosen_item[2])
        # print(center, radius)
        # Check if all other points lie on this circle
        
        if radius > 2*distance_points:
            return "Line", None, None
        else:
            for p in points:
                dist = np.sqrt((p[0] - center[0])**2 + (p[1] - center[1])**2)
                # print("***************************************")
                if abs(dist-radius) > 0.2*radius:
                    # print(dist - radius)
            #     # time.sleep(1)
            #     if abs(dist - radius) > 5:
                    return "Random Shape", None, None
    
    return "Arc", center, radius

def read_csv(csv_path):
    np_path_XYs = np.genfromtxt(csv_path, delimiter=',')
    path_XYs = []
    for i in np.unique(np_path_XYs[:, 0]):
        npXYs = np_path_XYs[np_path_XYs[:, 0] == i][:, 1:]
        XYs = []
        for j in np.unique(npXYs[:, 0]):
            XY = npXYs[npXYs[:, 0] == j][:, 1:]
            XYs.append(XY)
        path_XYs.append(XYs)
    return path_XYs

def angle_between_lines_from_points(p1, p2, p3, p4):
    v1 = np.array(p2) - np.array(p1)
    v2 = np.array(p4) - np.array(p3)

    dot_product = np.dot(v1, v2)

    magnitude_v1 = np.linalg.norm(v1)
    magnitude_v2 = np.linalg.norm(v2)

    cos_theta = dot_product / (magnitude_v1 * magnitude_v2)
    cos_theta = np.clip(cos_theta, -1.0, 1.0)

    theta_radians = np.arccos(cos_theta)
    theta_degrees = np.degrees(theta_radians)

    return theta_degrees

def distance_2d(point1, point2):
    p1 = np.array(point1)
    p2 = np.array(point2)

    distance = np.linalg.norm(p2 - p1)
    
    return distance

def calculate_slope_intercept(x1, y1, x2, y2):
    if x1 == x2:
        return None, None  
    m = (y2 - y1) / (x2 - x1)
    b = y1 - m * x1
    return m, b

def find_intersection_from_points(p1, p2, p3, p4):
    x1, y1, x2, y2, x3, y3, x4, y4 = p1[0], p1[1], p2[0], p2[1], p3[0], p3[1], p4[0], p4[1]
    m1, b1 = calculate_slope_intercept(x1, y1, x2, y2)
    m2, b2 = calculate_slope_intercept(x3, y3, x4, y4)
    
    if m1 is None:
        x = x1
        y = m2 * x + b2
    elif m2 is None:
        x = x3
        y = m1 * x + b1
    elif m1 == m2:
        return None if b1 != b2 else "Infinite intersections (coincident lines)"
    else:
        x = (b2 - b1) / (m1 - m2)
        y = m1 * x + b1
    
    return (x, y)

def generate_circle_points(cx, cy, r):
    points = []
    num_points=int(2 * math.pi * r)
    for i in range(num_points):
        theta = 2 * math.pi * i / num_points
        x = cx + r * math.cos(theta)
        y = cy + r * math.sin(theta)
        points.append((x, y))
    return points

input_path = read_csv('problems\\problems\\frag1.csv')

print(len(input_path))

fragments = []
count_points = 0
while count_points < len(input_path):
    target_input = input_path[count_points][0]
    # print(len(target_input))
    count = 0
    list_change = []
    points_list = []
    while count < len(target_input) - 2:
        angle = angle_between_lines_from_points(target_input[count], target_input[count + 1], target_input[count + 1], target_input[count + 2])
        # print(angle)
        if angle > 5:
            list_change.append(angle)
            points_list.append(target_input[count + 1])
        count += 1
    angle = angle_between_lines_from_points(target_input[-2], target_input[-1], target_input[-1], target_input[0])
    if angle > 5:
        list_change.append(angle)
        points_list.append(target_input[-1])

    # print(list_change)
    # print(points_list)
    point_count = 0
    dist_list = []
    while point_count < len(points_list) - 1:
        dist_list.append(distance_2d(points_list[point_count], points_list[point_count + 1]))
        point_count += 1

    dist_list.append(distance_2d(points_list[0], points_list[-1]))
    # print(list_change)
    # print(dist_list)
    # print(points_list)

    count_dist = 0
    flag = 0
    polygon_point = []
    while count_dist < len(dist_list):
        if dist_list[count_dist] > 10:
            polygon_point.append(points_list[count_dist])

        count_dist += 1

    if len(polygon_point) != 0:
        # polygon_point.append(polygon_point[0])
        point_count = 0
        index_val_list = []
        list_target = []
        for items in target_input:
            list_target.append(list(items))
        # print(target_input)
        import time 
        # time.sleep(20)
        while point_count < len(polygon_point):
            # print(polygon_point[point_count])
            # import time 
            # time.sleep(20)
            index_val = list_target.index(list(polygon_point[point_count]))
            # print(target_input[index_val])
            index_val_list.append(index_val)
            point_count += 1
        start = 0
        # print(index_val_list)
        for items in sorted(index_val_list):
            fragments.append(target_input[start:items])
            start = items

        # print(f"Yaayyy: {count_points}")

        # print(polygon_point)
    else:
        fragments.append(target_input)
    count_points += 1

print(len(fragments))

import matplotlib.pyplot as plt

def plot(paths_XYs):
    fig, ax = plt.subplots(tight_layout=True, figsize=(8, 8))
    for i, XYs in enumerate(paths_XYs):
        c = 'r'
        for XY in XYs:
            # print(XY)
            ax.plot(XY[:, 0], XY[:, 1], c=c, linewidth=2)
    ax.set_aspect('equal')
    plt.show()

arc = []
line = []
random_shape = []

for fragment in fragments:
    comment, centre, radius = points_on_same_circle(fragment)
    if comment == "Arc":
        arc.append([fragment, centre, radius])
    elif comment == "Random Shape":
        random_shape.append(fragment)
    else: 
        line.append(fragment)
        
    # time.sleep(50)
    print(comment, centre, radius)
    # plot([fragment[0]])
    fragment = np.array(fragment)
    plot([[fragment]])
    # print("##################################################################")

'''
arc_figures = []
arc_count = 0
record = []

print(arc)
print(len(arc))
while arc_count < len(arc):
    if arc_count not in record:
        arc_figures.append([arc[arc_count][0]])
        print(arc_figures)
        tracker_arc = arc_count + 1
        while tracker_arc < len(arc):
            if tracker_arc not in record:
                if abs(distance_2d(arc[arc_count][1], arc[tracker_arc][1])) < 0.15*arc[arc_count][2] and abs(arc[arc_count][2] - arc[tracker_arc][2]) < 0.15*arc[arc_count][2] and distance_2d(arc[arc_count][0][-1], arc[tracker_arc][0][0]) < 0.15*calculate_path_length(arc[arc_count][0]):
                    record.append(1)
                    arc_figures[len(arc_figures) - 1] = [arc[tracker_arc]] + arc_figures[len(arc_figures) - 1]
                else: 
                    arc_figures.append(arc[tracker_arc])
            tracker_arc += 1
        arc_count += 1
    else:
        arc_count += 1

print(len(arc_figures))
'''
