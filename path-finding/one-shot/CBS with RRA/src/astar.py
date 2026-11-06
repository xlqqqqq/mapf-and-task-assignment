"""CBS使用的低层时空A*。"""

from RRA import StateTwoDimension
from RRA_continue import ASTAR_two_dimension_two


class ConflictInformation:
    # 保存当前状态与其他路径之间的顶点冲突或边冲突信息。

    VERTEX = 1
    EDGE = 2

    def __init__(self, time, conflict_type, agent1, agent2, location1, location2, ):
        self.time = time
        self.type = conflict_type
        self.agent1 = agent1
        self.agent2 = agent2
        self.location1 = tuple(location1)
        self.location2 = tuple(location2)


class VertexConstraint:
    # 禁止智能体在指定时刻位于指定位置。

    def __init__(self, time, location):
        self.time = time
        self.location = tuple(location)

    def __eq__(self, other):
        return self.time == other.time and self.location == other.location

    def __hash__(self):
        return hash((self.time, self.location))

    def __str__(self):
        return str((self.time, self.location))


class EdgeConstraint:
    # 禁止智能体在指定时刻沿一条有向边移动。

    def __init__(self, time, from_location, to_location):
        self.time = time
        self.from_location = tuple(from_location)
        self.to_location = tuple(to_location)

    def __eq__(self, other):
        return (self.time == other.time and self.from_location == other.from_location and self.to_location ==
                other.to_location)

    def __hash__(self):
        return hash((self.time, self.from_location, self.to_location))

    def __str__(self):
        return str((self.time, self.from_location, self.to_location))


class State:
    # 低层A*中的位置和时间状态。

    def __init__(self, point, time=0):
        self.x = point[0]
        self.y = point[1]
        self.time = time
        self.number = 0  # 节点进入顺序，消除随机性，确保每次搜索结果一致。
        self.cat = 0  # conflict avoidance table 用来存储当前状态与其他路径之间的冲突数。
        self.h = 0
        self.f = 0
        self.g = 0

    @property
    def position(self):
        return self.x, self.y

    @property
    def key(self):
        # 时空状态的身份；f、h、cat在搜索过程中会改变，所以只能使用不变的坐标和时间。
        return self.time, self.x, self.y

    def __eq__(self, other):
        return self.key == other.key

    def __hash__(self):
        return hash(self.key)

    def __str__(self):
        return str((self.time, self.x, self.y))

    def __lt__(self, other):
        # 排序规则保持为 f->CAT->h->进入顺序。
        return (self.f, self.cat, self.h, self.number) < (other.f, other.cat, other.h, other.number)


def get_path_position(path, time, stay_at_goal):
    # 取得路径在指定时刻的位置

    if time < 0 or not path:
        return None
    if time < len(path):
        return path[time].position
    if stay_at_goal:
        return path[-1].position
    return None


class GridEnvironment:

    def __init__(self, obstacles, dimensions, vertex_constraints, edge_constraints, ):
        self.obstacles = obstacles
        self.dimensions = dimensions
        self.vertex_constraints = vertex_constraints
        self.edge_constraints = edge_constraints

    def is_in_map(self, point):
        return 0 <= point[0] < self.dimensions[0] and 0 <= point[1] < self.dimensions[1]

    def is_free(self, point):
        return tuple(point) not in self.obstacles

    def satisfies_vertex_constraint(self, state):
        constraint = VertexConstraint(state.time, state.position)
        return constraint not in self.vertex_constraints

    def satisfies_edge_constraint(self, current, neighbor):
        constraint = EdgeConstraint(neighbor.time, current.position, neighbor.position)
        return constraint not in self.edge_constraints

    def get_neighbors(self, current):
        # 依次尝试等待、上、下、左、右五种动作。

        candidate_points = [
            [current.x, current.y],
            [current.x, current.y + 1],
            [current.x, current.y - 1],
            [current.x - 1, current.y],
            [current.x + 1, current.y],
        ]
        neighbors = []

        for point in candidate_points:
            neighbor = State(point, current.time + 1)
            if not self.is_in_map(point) or not self.is_free(point):
                continue
            if not self.satisfies_vertex_constraint(neighbor):
                continue
            # 等待也是(x, y)->(x, y)的边，因此同样检查边约束。
            if not self.satisfies_edge_constraint(current, neighbor):
                continue
            neighbors.append(neighbor)

        return neighbors


class ASTAR:

    def __init__(self, name, start, goal, start_time, obstacles, dimensions, vertex_constraints, edge_constraints,
                 solution, stay_at_goal):
        self.name = name
        self.start_state = State(start, start_time)
        self.goal = tuple(goal)
        self.obstacles = obstacles
        self.dimensions = dimensions
        self.vertex_constraints = vertex_constraints
        self.edge_constraints = edge_constraints
        self.solution = solution
        self.stay_at_goal = stay_at_goal

    def is_goal_state(self, state):
        if state.position != self.goal:
            return False
        if not self.stay_at_goal:
            return True

        # 停留模式下，到达终点后还要满足未来终点上的约束。
        for constraint in self.vertex_constraints:
            if constraint.time > state.time and constraint.location == self.goal:
                return False

        # for constraint in self.edge_constraints:
        #     if (constraint.time > state.time and constraint.from_location == self.goal and constraint.to_location
        #             == self.goal):
        #         return False

        return True

    @staticmethod
    def reconstruct_path(parent, current):
        path = [current]
        while current in parent:
            current = parent[current]
            path.append(current)
        return path[::-1]

    def count_cat_conflicts(self, state, previous_state):
        # 计算当前一步移动与其他智能体当前路径产生的CAT冲突数。

        conflict_count = 0

        for agent_name, agent_path in self.solution.items():

            if agent_name == self.name:
                continue

            other_current = get_path_position(agent_path, state.time, self.stay_at_goal)

            if other_current is None:
                continue

            # 检查是否点冲突
            if other_current == state.position:
                conflict_count += 1
                continue

            other_previous = get_path_position(agent_path, state.time - 1, self.stay_at_goal)

            # 检查是否边冲突
            if previous_state.position == other_current and state.position == other_previous:
                conflict_count += 1

        return conflict_count

    def find_path_conflicts(self, path):

        path_conflicts = []

        for other_agent, other_path in self.solution.items():
            if other_agent == self.name:
                continue

            max_path_length = max(len(path), len(other_path))

            # 已知所有智能体的起点互不相同，因此从 t=1 开始检查。
            for time in range(1, max_path_length):
                current1 = get_path_position(path, time, self.stay_at_goal)
                current2 = get_path_position(other_path, time, self.stay_at_goal)

                # 终点消失模式下，路径结束后的位置为None，不再参与冲突判断。
                if current1 is None or current2 is None:
                    continue

                # 检查是否点冲突，如果是，生成对应的Conflict信息。
                if current1 == current2:
                    path_conflicts.append(ConflictInformation(time, ConflictInformation.VERTEX, self.name, other_agent,
                                                              current1,
                                                              current2))
                    continue

                previous1 = get_path_position(path, time - 1, self.stay_at_goal)
                previous2 = get_path_position(other_path, time - 1, self.stay_at_goal)

                # 检查是否边冲突，如果是，生成对应的Conflict信息。
                if previous1 == current2 and previous2 == current1:
                    path_conflicts.append(ConflictInformation(time, ConflictInformation.EDGE, self.name, other_agent,
                                                              previous1,
                                                              current1))

        return path_conflicts

    def search(self, h_values, h_open, h_closed):

        # 转换为二维状态，对应h_values的键，从而能直接读取h_values中的值。
        start_rra_state = StateTwoDimension(self.start_state.position)

        # if start_rra_state not in h_values:
        #     return [], [], h_values, h_open, h_closed, 0

        open_set = {self.start_state}
        # 与open_set同步的索引，键为 (time, x, y)，用于O(1)判断后继是否已在open中。
        open_index = {self.start_state.key: self.start_state}
        closed_set = set()
        parent = {}
        g_score = {self.start_state: 0}

        # cat_score记录由起点到当前状态的路径和其他路径之间的冲突数。
        cat_score = {self.start_state: 0}

        # 节点进入顺序
        enter_number = 0

        self.start_state.g = 0
        self.start_state.h = h_values[start_rra_state]
        self.start_state.f = self.start_state.h

        environment = GridEnvironment(self.obstacles, self.dimensions, self.vertex_constraints, self.edge_constraints)

        while open_set:

            current = min(open_set)
            if self.is_goal_state(current):
                path = self.reconstruct_path(parent, current)  # 生成从起点到终点的路径
                path_conflicts = self.find_path_conflicts(path)  # 查找最终的路径与其他智能体路径之间的冲突
                return path, path_conflicts, h_values, h_open, h_closed, current.f + 1

            open_set.remove(current)
            del open_index[current.key]
            closed_set.add(current)

            for neighbor in environment.get_neighbors(current): # 遍历当前状态的所有邻居，寻找可行的扩展节点。
                if neighbor in closed_set:
                    continue

                new_g = g_score[current] + 1

                # 计算扩展节点的冲突数，由当前状态的冲突数加上扩展节点的冲突数。
                new_cat = cat_score[current] + self.count_cat_conflicts(neighbor, current)

                # 如果open中已有相同的 (x, y, time)，取出原对象比较代价。
                old_neighbor = open_index.get(neighbor.key)

                if old_neighbor is not None:
                    # 如果扩展节点的代价大于等于已存在的节点或者代价相等时冲突数大于等于已存在的节点，跳过。

                    if new_g > g_score[old_neighbor]:
                        continue
                    if new_g == g_score[old_neighbor] and new_cat >= cat_score[old_neighbor]:
                        continue

                    neighbor.number = old_neighbor.number
                    open_set.remove(old_neighbor)
                else:
                    enter_number += 1
                    neighbor.number = enter_number

                # 转换为二维状态，查看h_values中是否存在该状态。
                neighbor_rra_state = StateTwoDimension(neighbor.position)

                # 如果不在closed_set中，则要继续RRA搜索。
                if neighbor_rra_state not in h_closed:
                    continue_search = ASTAR_two_dimension_two(neighbor_rra_state, self.obstacles, self.dimensions)
                    h_values, h_open, h_closed = continue_search.search(h_values, h_open, h_closed)

                # 更新扩展节点信息。
                neighbor.g = new_g
                neighbor.h = h_values[neighbor_rra_state]
                neighbor.f = neighbor.g + neighbor.h
                neighbor.cat = new_cat

                parent[neighbor] = current
                g_score[neighbor] = new_g
                cat_score[neighbor] = new_cat
                open_index[neighbor.key] = neighbor
                open_set.add(neighbor)

        return [], [], h_values, h_open, h_closed, 0
