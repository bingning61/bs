import math
import numpy as np
from plot_path import plot_path, plot_paths


def path_planning_multiple_obstacles(A, B, obstacles):
    """路径规划主函数，处理多个障碍物"""

    # 遍历所有障碍物判断是否A与B在障碍物内
    inObstacle = False
    for obstacle in obstacles:
        # 判断A是否在圆内
        A_tangents = calculate_tangent_points(A, obstacle)
        if A_tangents is None:
            inObstacle = True

        # 判断B是否在圆内
        B_tangents = calculate_tangent_points(B, obstacle)
        if B_tangents is None:
            inObstacle = True

    if inObstacle:
        return None
    else:
        # 获取所有切圆路径
        paths = get_all_paths(A, B, obstacles)
        print(len(paths))

        new_paths = []
        for path in paths:
            last_index = -1
            for i, item in enumerate(path):
                if np.array_equal(item, A):
                    last_index = i
            new_paths.append(path[last_index:])

        # 计算各路径长度
        path_lengths = [sum(np.linalg.norm(np.diff(path, axis=0), axis=1)) for path in new_paths]

        # 选择最短路径
        shortest_idx = np.argmin(path_lengths)
        return new_paths[shortest_idx]


def calculate_tangent_points(A, obstacle):
    """计算从点A到圆O的切线点"""

    O, R = obstacle
    OA = O - A
    dist_OA = np.linalg.norm(OA)
    # 处理代码计算后导致精度丢失问题
    if abs(dist_OA - R) < 1e-8:
        dist_OA = R
    if dist_OA < R:
        return None  # 点在圆内无切线

    alpha = np.arctan2(OA[1], OA[0])
    beta = np.arcsin(R / dist_OA)

    # 计算两个切点
    theta1 = alpha - beta
    theta2 = alpha + beta

    T1 = O - R * np.array([np.cos(theta1 + np.pi / 2), np.sin(theta1 + np.pi / 2)])
    T2 = O - R * np.array([np.cos(theta2 - np.pi / 2), np.sin(theta2 - np.pi / 2)])

    # 使用round保留小数位数
    precision = 8
    return [round(T1[0], precision), round(T1[1], precision)], [round(T2[0], precision), round(T2[1], precision)]


def get_closest_obstacle(A, B, obstacles, last_obstacle=None):
    """
    获取经过 AB 直线上的障碍物中，距离 A 最近的障碍物。

    参数:
    A (np.array): 起点坐标，形状为 (2,)
    B (np.array): 终点坐标，形状为 (2,)
    obstacles (list): 障碍物列表，每个元素是一个元组 (O, r)

    返回:
    tuple: 最近的障碍物 (O, r) 或 None
    """
    closest_obstacle = None
    min_distance = float('inf')

    if last_obstacle is not None:
        # 如果上一次找到的障碍物与当前障碍物相同，则跳过该障碍物
        O1, R1 = last_obstacle
        obstacles = [ob for ob in obstacles if not np.array_equal(ob[0], O1) or ob[1] != R1]

    for obstacle in obstacles:
        if is_line_passing_through_circle(A, B, obstacle):
            O, r = obstacle
            distance = np.linalg.norm(O - A)  # 计算障碍物中心到 A 的距离
            if distance < min_distance:
                min_distance = distance
                closest_obstacle = obstacle
    return closest_obstacle


def is_line_intersecting_circle(A, B, obstacle):
    """判断线段AB是否与圆O相交"""
    O, r = obstacle
    # 向量AB
    AB = B - A
    # 向量AO
    AO = O - A

    # 计算AB方向上的单位向量
    length_AB = np.linalg.norm(AB)
    if length_AB == 0:
        return False  # A和B重合，无法构成线段
    dir_AB = AB / length_AB

    # 投影长度：AO 在 AB 方向上的投影长度
    proj_length = np.dot(AO, dir_AB)

    # 最近点在线段上
    if proj_length < 0:
        closest_point = A
    elif proj_length > length_AB:
        closest_point = B
    else:
        closest_point = A + dir_AB * proj_length

    # 判断最近点到圆心的距离是否小于等于半径
    distance_to_circle = np.linalg.norm(closest_point - O)
    return distance_to_circle <= r


def is_line_passing_through_circle(A, B, obstacle):
    """判断线段AB是否完全穿过圆O内部，并忽略A/B在圆上的误差"""
    O, r = obstacle

    # 向量AB
    AB = B - A
    length_AB = np.linalg.norm(AB)
    if length_AB == 0:
        return False  # A和B重合，无法构成线段

    # 单位方向向量
    dir_AB = AB / length_AB

    # 圆心O到线段AB的投影点
    AO = O - A
    proj_length = np.dot(AO, dir_AB)

    # 投影点在线段外的情况
    if proj_length < 0 or proj_length > length_AB:
        return False  # 投影点不在线段上，说明线段与圆没有交点或切于端点

    # 计算最近点（即投影点）
    closest_point = A + dir_AB * proj_length

    # 判断距离是否小于半径
    distance_to_line = np.linalg.norm(O - closest_point)
    if distance_to_line >= r:
        return False  # 不与圆相交

    # 检查A和B是否都在圆外（允许误差）
    dist_A_to_O = np.linalg.norm(A - O)
    dist_B_to_O = np.linalg.norm(B - O)

    # 忽略A/B在圆上的情况（使用一个微小误差 epsilon）
    epsilon = 1e-8
    return abs(r - dist_A_to_O) > epsilon and abs(r - dist_B_to_O) > epsilon


def get_point_tangents(A, B, obstacle):
    """获取起点与圆的切点"""
    # 计算A到圆的切线
    A_tangents = calculate_tangent_points(A, obstacle)
    # 计算B到圆的切线
    B_tangents = calculate_tangent_points(B, obstacle)
    return {
        "A": A_tangents,
        "B": B_tangents
    }


def get_all_paths(A, B, obstacles, closest_two_line=None):
    """获取所有路径"""

    paths = []
    # AB线段是否穿过障碍物
    obstacle = get_closest_obstacle(A, B, obstacles)
    if obstacle is not None:
        # 获取A、B的切线点
        tangents = get_point_tangents(A, B, obstacle)
        # 找距离公切线closest_two_line最近的切点
        if closest_two_line is not None:
            if is_point_on_circle(closest_two_line[0], obstacle):
                closest_point = find_closest_point_to_segment(tangents["B"], closest_two_line)
                tangents["B"] = [closest_point]
        for tangentB in tangents["B"]:
            obstacle2 = get_closest_obstacle(np.array(tangentB), B, obstacles, obstacle)
            # 要tangentB到B穿过障碍圆的
            if obstacle2 is not None:
                new_paths = get_tangent_lines_from_o1_o2(tangentB, obstacle, obstacle2, obstacles, A, B, closest_two_line)
                paths = paths + new_paths
            else:
                # 递归遍历开启起点到best_tangentA最优路径
                min_distance = float('inf')
                best_tangentA = None
                # 找出距离 tangentB 最近的 tangentA
                for tangentA in tangents["A"]:
                    distance = np.linalg.norm(np.array(tangentA) - np.array(tangentB))
                    if distance < min_distance:
                        min_distance = distance
                        best_tangentA = tangentA
                obstacle2 = get_closest_obstacle(A, best_tangentA, obstacles, obstacle)
                if obstacle2 is not None:
                    new_paths = get_tangent_lines_from_o1_o2(best_tangentA, obstacle2, obstacle, obstacles, A, B, closest_two_line)
                    paths = paths + new_paths
                else:
                    arc = generate_arc_points(best_tangentA, tangentB, obstacle)
                    paths.append([A.tolist()] + arc + [B.tolist()])
    else:
        paths.append([A.tolist()] + [B.tolist()])
    return paths

def is_point_on_circle(point, obstacle):
    """
    判断点 A 是否在以 O 为圆心、r 为半径的圆上。

    参数:
    A (np.array or list): 点坐标 [x, y]
    O (np.array or list): 圆心坐标 [x, y]
    r (float): 圆的半径
    epsilon (float): 浮点数精度误差容忍值，默认 1e-8

    返回:
    bool: True 表示点在圆上
    """
    O, r = obstacle
    epsilon = 1e-8
    distance = np.linalg.norm(np.array(point) - np.array(O))
    return abs(distance - r) < epsilon

def find_closest_point_to_segment(points, closest_two_line):
    """
    找出 points 中距离线段 AB 最近的点

    参数:
    - points: 点列表 [(x1, y1), ...]
    - A: 线段起点
    - B: 线段终点

    返回:
    - 距离最近的点
    """
    A = np.array(closest_two_line[0])
    B = np.array(closest_two_line[1])
    min_dist = float('inf')
    closest_point = None

    for point in points:

        dist = np.linalg.norm(closest_point_on_segment(point, A, B) - point)
        if dist < min_dist:
            min_dist = dist
            closest_point = point

    return closest_point


def closest_point_on_segment(P, A, B):
    """
    计算点 P 到线段 AB 的最近点。

    参数:
    - P: 目标点 (np.array)
    - A: 线段起点 (np.array)
    - B: 线段终点 (np.array)

    返回:
    - 最近点 (np.array)
    """
    AP = P - A
    AB = B - A
    length_AB = np.linalg.norm(AB)

    if length_AB == 0:
        return A  # 线段退化为一个点

    t = np.dot(AP, AB) / (length_AB ** 2)
    t = np.clip(t, 0, 1)  # 投影超出线段时限制在 [0, 1]
    proj = A + t * AB  # 最近点
    return proj


def get_tangent_lines_from_o1_o2(tangentB, obstacle, obstacle2, obstacles, A, B, before_closest_two_line = None):
    """
    从A到tangentB的路径再到B的路径
    """
    paths = []
    outer_lines = outer_tangent_lines(obstacle, obstacle2)
    inner_lines = inner_tangent_lines(obstacle, obstacle2)
    closest_two_lines = find_closest_two_lines(tangentB, outer_lines, inner_lines)

    if before_closest_two_line is not None:
        if is_point_on_circle(before_closest_two_line[0], obstacle2):
            closest_two_lines = find_closest_two_lines(np.array(before_closest_two_line[0]), outer_lines, inner_lines)

    for closest_two_line in closest_two_lines:
        losest_tangents = get_losest_tangents(A, B, obstacle2, np.array(closest_two_line[1]))

        # 向前查找所有路径
        new_paths = []
        to_B_path = []
        if not np.allclose(np.array(losest_tangents[1]), B):
            obstacle3 = get_closest_obstacle(np.array(losest_tangents[1]), B, obstacles)
            if obstacle3 is not None:
                new_paths = get_tangent_lines_from_o1_o2(np.array(losest_tangents[1]), obstacle2, obstacle3, obstacles,
                                                         A, B)
            else:
                to_B_path = [losest_tangents[1]] + [B.tolist()]
                # new_paths.append([losest_tangents[1]] + [B.tolist()])

        # 向后逆差短路径
        shortest_path = []
        # 判断A到losest_tangents[0]是否经过圆
        obstacle_cb = get_closest_obstacle(A, np.array(losest_tangents[0]), obstacles)
        if obstacle_cb is not None:
            # todo:优化A到losest_tangents[0]最优路径
            shortest_path = recompute_shortest_path_from_A_to_Tangent(A, np.array(closest_two_line[1]), obstacles,
                                                                      closest_two_line)
        else:
            shortest_path = [A.tolist(), losest_tangents[0]]

        if len(to_B_path):
            arc = generate_arc_points(np.array(shortest_path[-1]), np.array(to_B_path[0]), obstacle2)
            paths.append(shortest_path + arc + to_B_path)
        elif len(new_paths):
            for new_path in new_paths:
                arc = generate_arc_points(np.array(shortest_path[-1]), np.array(new_path[0]), obstacle2)
                paths.append(shortest_path + arc + new_path)
        else:
            paths.append(shortest_path)
    return paths


def recompute_shortest_path_from_A_to_Tangent(A, tangent_point, obstacles, closest_two_line):
    """
    从起点 A 到指定切点的最短路径，避开所有障碍物。
    """
    # 使用现有的 get_all_paths 方法获取路径
    paths = get_all_paths(A, tangent_point, obstacles, closest_two_line)

    # 计算各路径长度并选择最短路径
    path_lengths = [sum(np.linalg.norm(np.diff(path, axis=0), axis=1)) for path in paths]
    shortest_idx = np.argmin(path_lengths)
    return paths[shortest_idx]


def outer_tangent_lines(c1, c2):
    """获取外公切线"""
    o1, r1 = c1
    o2, r2 = c2
    x1, y1 = o1
    x2, y2 = o2

    dx = x2 - x1
    dy = y2 - y1
    d = math.hypot(dx, dy)

    if d <= abs(r1 - r2):  # 内含或内切，无外切线
        return []

    # factor = 10 ** 8
    # 使用round保留小数位数
    precision = 8

    # 计算基础角度
    theta = math.atan2(dy, dx)
    alpha = math.asin((r2 - r1) / d)
    lines = []
    for sign in [1, -1]:
        angle = theta + sign * alpha
        t1 = [
            round(x1 + r1 * math.cos(angle + math.pi / 2) * sign, precision),
            round(y1 + r1 * math.sin(angle + math.pi / 2) * sign, precision),
            # int((x1 + r1 * math.cos(angle + math.pi / 2) * sign) * factor) / factor,
            # int((y1 + r1 * math.sin(angle + math.pi / 2) * sign) * factor) / factor,
        ]
        t2 = [
            round(x2 + r2 * math.cos(angle + math.pi / 2) * sign, precision),
            round(y2 + r2 * math.sin(angle + math.pi / 2) * sign, precision),
            # int((x2 + r2 * math.cos(angle + math.pi / 2) * sign) * factor) / factor,
            # int((y2 + r2 * math.sin(angle + math.pi / 2) * sign) * factor) / factor,
        ]

        lines.append([t1, t2])
    return lines


def inner_tangent_lines(c1, c2):
    """获取内公切线"""
    o1, r1 = c1
    o2, r2 = c2
    x1, y1 = o1
    x2, y2 = o2

    dx = x2 - x1
    dy = y2 - y1
    d = np.hypot(dx, dy)

    if d == 0:  # 同心圆无切线
        return []

    # 计算基础角度
    theta = math.atan2(dy, dx)

    lines = []
    # 内切线计算
    if d > (r1 + r2):
        beta = math.asin((r1 + r2) / d)
        for sign in [1, -1]:
            angle = theta + sign * beta
            t1 = [
                x1 + r1 * math.cos(angle - sign * math.pi / 2),
                y1 + r1 * math.sin(angle - sign * math.pi / 2)
            ]
            t2 = [
                x2 - r2 * math.cos(angle - sign * math.pi / 2),
                y2 - r2 * math.sin(angle - sign * math.pi / 2)
            ]
            lines.append([t1, t2])
    return lines


def find_closest_two_lines(tangent, outer_lines, inner_lines):
    """找最近的外切点与内切点"""
    all_lines = []
    for lines in [outer_lines, inner_lines]:
        if lines:
            all_lines.extend(lines)

    if not all_lines:
        return []

    distances = []
    for line in all_lines:
        A, B = np.array(line[0]), np.array(line[1])
        dist = distance_point_to_segment(tangent, A, B)
        distances.append((dist, line))

    # 按距离从小到大排序
    distances.sort(key=lambda x: x[0])

    # 取最近的两条线段
    closest_lines = [line for dist, line in distances[:2]]
    return closest_lines


def distance_point_to_segment(P, A, B):
    """计算点 P 到线段 AB 的最短距离"""
    AP = P - A
    AB = B - A
    proj_len = np.dot(AP, AB)
    if proj_len <= 0:
        return np.linalg.norm(AP)
    ab_len_sq = np.dot(AB, AB)
    if proj_len >= ab_len_sq:
        return np.linalg.norm(P - B)
    # 投影在线段上
    proj = A + (proj_len / ab_len_sq) * AB
    return np.linalg.norm(P - proj)


def generate_arc_points(T1, T2, obstacle):
    """
    生成从 T1 到 T2 的最短圆弧上的点
    自动判断使用顺时针还是逆时针方向，并根据弧长自动调整采样点数量（6~62）
    """

    n_min = 6
    n_max = 62
    O, r = obstacle

    # 计算起始和结束角度
    alpha1 = np.arctan2(T1[1] - O[1], T1[0] - O[0])
    alpha2 = np.arctan2(T2[1] - O[1], T2[0] - O[0])

    # 计算两个方向的角度差
    delta_angle_cw = (alpha1 - alpha2) % (2 * np.pi)
    delta_angle_ccw = (alpha2 - alpha1) % (2 * np.pi)

    # 选择较小的方向
    clockwise = delta_angle_cw < delta_angle_ccw
    angle_diff = delta_angle_cw if clockwise else delta_angle_ccw

    # 根据弧长计算采样点数量（线性映射：0 ~ 2π => 6 ~ 62）
    n = int(n_min + (angle_diff / (2 * np.pi)) * (n_max - n_min))
    n = max(n, n_min)  # 确保最少为6个点

    if clockwise:
        if alpha1 < alpha2:
            alpha1 += 2 * np.pi
        angles = np.linspace(alpha1, alpha2, n)
    else:
        if alpha2 < alpha1:
            alpha2 += 2 * np.pi
        angles = np.linspace(alpha1, alpha2, n)

    arc_points = [O + r * np.array([np.cos(theta), np.sin(theta)]) for theta in angles]
    return arc_points


def get_losest_tangents(A, B, obstacle, C):
    """获取距离C最近的两个切点【A与obstacle的切点、B与obstacle的切点，各取一个】"""
    tangents_A = calculate_tangent_points(A, obstacle)
    tangents_B = calculate_tangent_points(B, obstacle)

    if tangents_B is not None:
        T1_B, T2_B = tangents_B
        dist_T1_C = np.linalg.norm(T1_B - C)
        dist_T2_C = np.linalg.norm(T2_B - C)

        best_tangent_B = T1_B if dist_T1_C < dist_T2_C else T2_B
    else:
        best_tangent_B = B.tolist()

    T1_A, T2_A = tangents_A
    dist_T1_C = np.linalg.norm(T1_A - C)
    dist_T2_C = np.linalg.norm(T2_A - C)

    best_tangent_A = T1_A if dist_T1_C < dist_T2_C else T2_A

    return [best_tangent_A, best_tangent_B]


# 示例使用
if __name__ == "__main__":
    params = {
        "a": [
            10,
            50
        ],
        "b": [
            90,
            50
        ],
        "obstacles": [
            {
                "o": [
                    28,
                    53
                ],
                "radius": 4
            },
            {
                "o": [
                    57,
                    47
                ],
                "radius": 5
            },
            {
                "o": [
                    80,
                    53
                ],
                "radius": 6
            }
        ]
    }
    A = np.array(params["a"])  # 起点
    B = np.array(params["b"])  # 终点
    obstacles = []
    for ob in params["obstacles"]:
        obstacles.append((np.array(ob["o"]), ob["radius"]))
    path = path_planning_multiple_obstacles(A, B, obstacles)
    if path is not None:
        plot_path(A, B, obstacles, path)
    else:
        print("起点或终点在障碍物内，无法规划路径")
