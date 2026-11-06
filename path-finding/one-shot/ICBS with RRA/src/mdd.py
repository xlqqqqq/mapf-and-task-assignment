from collections import deque
from astar import VertexConstraint
from astar import EdgeConstraint


# 定义网格环境类
class GridEnvironment:
    def __init__(self, obstacles, edge_constraints, vertex_constraints, dimension):
        self.obstacles = obstacles
        self.edge_constraints = edge_constraints
        self.vertex_constraints = vertex_constraints
        self.dimension = dimension

    def in_map(self, expand_point):
        if 0 <= expand_point[0] < self.dimension[0] and 0 <= expand_point[1] < self.dimension[1]:
            return True
        else:
            return False

    def no_obstacles(self, expand_point):
        if (expand_point[0], expand_point[1]) not in self.obstacles:
            return True
        else:
            return False

    def vertex_conflict_judge(self, expand_state, time):
        if VertexConstraint(time, [expand_state[0], expand_state[1]]) not in self.vertex_constraints:
            return True
        else:
            return False

    def edge_conflict_Judge(self, current_state, expand_state, time):
        if EdgeConstraint(time, [current_state[0], current_state[1]],
                          [expand_state[0], expand_state[1]]) not in self.edge_constraints:
            return True
        else:
            return False

    def get_neighbors(self, current, time):
        time = time + 1
        neighbors = []
        candidate_points = [
            [current[0], current[1]],
            [current[0], current[1] + 1],
            [current[0], current[1] - 1],
            [current[0] - 1, current[1]],
            [current[0] + 1, current[1]],
        ]
        for point in candidate_points:
            if self.in_map(point) and self.no_obstacles(point) and self.vertex_conflict_judge(point, time) and \
                    self.edge_conflict_Judge(current, point, time):
                neighbors.append(tuple(point))

        return neighbors


# 启发式函数（曼哈顿距离）
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# 使用BFS构建MDD
def build_mdd(env, start, goal, cost):
    # 初始化MDD
    mdd = [set() for _ in range(cost + 1)]
    mdd[0].add(start)

    # 初始化队列用于 BFS，队列元素为 (位置, 时间步)
    queue = deque([(start, 0)])

    while queue:
        current_pos, current_time = queue.popleft()

        # 如果当前时间步达到成本上限，跳过
        if current_time >= cost:
            continue

        # 遍历当前位置的邻居
        for neighbor in env.get_neighbors(current_pos, current_time):
            next_time = current_time + 1

            # 剪枝：如果剩余时间不足以到达目标，跳过该节点
            if heuristic(neighbor, goal) > cost - next_time:
                continue

            # 如果邻居节点不在当前时间步的MDD中，添加到MDD并加入队列
            if neighbor not in mdd[next_time]:
                mdd[next_time].add(neighbor)
                queue.append((neighbor, next_time))

    # 从目标位置反向过滤，只保留能够在给定成本内到达目标的路径
    reachable_goal = [goal]
    for t in range(cost - 1, -1, -1):
        new_reachable = set()
        for pos in mdd[t]:
            for neighbor in env.get_neighbors(pos, t):
                if neighbor in reachable_goal:
                    new_reachable.add(pos)
        reachable_goal = new_reachable
        mdd[t] = reachable_goal

    return mdd


def compute_mdd(obstacles, edge_constraints, vertex_constraints, cost, start, goal, dimension):
    env = GridEnvironment(obstacles, edge_constraints, vertex_constraints, dimension)
    start = tuple(start)
    goal = tuple(goal)
    cost = cost - 1
    # 构建MDD
    mdd = build_mdd(env, start, goal, cost)

    return mdd
