"""从目标点反向搜索，为低层A*提供真实距离启发值。"""


class GridEnvironment:
    """负责判断位置是否在地图内、是否碰到障碍物。"""

    def __init__(self, obstacles, dimensions):
        self.obstacles = obstacles
        self.dimensions = dimensions

    def is_in_map(self, point):
        return 0 <= point[0] < self.dimensions[0] and 0 <= point[1] < self.dimensions[1]

    def is_free(self, point):
        return tuple(point) not in self.obstacles

    def get_neighbors(self, current):
        """返回上、下、左、右四个可通行位置。"""

        candidates = [
            [current.x, current.y + 1],
            [current.x, current.y - 1],
            [current.x - 1, current.y],
            [current.x + 1, current.y],
        ]

        return [point for point in candidates if self.is_in_map(point) and self.is_free(point)]


class StateTwoDimension:
    """RRA使用的二维状态，不包含时间。"""

    def __init__(self, point):
        self.x = point[0]
        self.y = point[1]

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __str__(self):
        return str((self.x, self.y))


class ASTAR_TwoDimension:

    def __init__(self, start_point, goal_point, obstacles, dimensions):
        self.start_state = StateTwoDimension(start_point)
        self.goal_state = StateTwoDimension(goal_point)
        self.environment = GridEnvironment(obstacles, dimensions)

    def search(self):

        # 标准的Dijkstra搜索，直到搜索到目标点，返回h值距离表、open集、closed集。
        distance = {self.start_state: 0}
        open_set = {self.start_state}
        closed_set = set()

        while open_set:
            current = min(open_set, key=lambda state: distance[state])
            open_set.remove(current)
            closed_set.add(current)

            for point in self.environment.get_neighbors(current):
                neighbor = StateTwoDimension(point)
                if neighbor in closed_set:
                    continue

                new_distance = distance[current] + 1
                old_distance = distance.get(neighbor, float("inf"))

                if neighbor not in open_set:
                    open_set.add(neighbor)
                elif new_distance >= old_distance:
                    continue

                distance[neighbor] = new_distance

            if current == self.goal_state:
                return distance, open_set, closed_set

        return distance, open_set, closed_set
