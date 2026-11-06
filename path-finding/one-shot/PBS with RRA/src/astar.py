"""PBS使用的低层时空A*。"""

from RRA import StateTwoDimension
from RRA_continue import ASTAR_two_dimension_two


class ConflictInformation:
    # 保存两条路径之间的顶点冲突或边冲突信息。

    VERTEX = 1
    EDGE = 2

    def __init__(self, time, conflict_type, agent1, agent2, location1, location2):
        self.time = time
        self.type = conflict_type
        self.agent1 = agent1
        self.agent2 = agent2
        self.location1 = tuple(location1)
        self.location2 = tuple(location2)

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
    # 取得路径在指定时刻的位置；消失后返回None。

    if time < 0 or not path:
        return None
    if time < len(path):
        return path[time].position
    if stay_at_goal:
        return path[-1].position
    return None


def find_pair_conflicts(agent1, path1, agent2, path2, stay_at_goal):
    # 计算两条路径之间的全部顶点冲突和边冲突。

    conflicts = []
    max_path_length = max(len(path1), len(path2))

    # 已知所有智能体的起点互不相同，因此从 t=1 开始检查。
    for time in range(1, max_path_length):
        current1 = get_path_position(path1, time, stay_at_goal)
        current2 = get_path_position(path2, time, stay_at_goal)

        # 终点消失模式下，路径结束后的位置为None，不再参与冲突判断。
        if current1 is None or current2 is None:
            continue

        # 检查是否点冲突，如果是，生成对应的Conflict信息。
        if current1 == current2:
            conflicts.append(ConflictInformation(time, ConflictInformation.VERTEX, agent1, agent2,
                                                 current1, current2))
            continue

        previous1 = get_path_position(path1, time - 1, stay_at_goal)
        previous2 = get_path_position(path2, time - 1, stay_at_goal)

        # 检查是否边冲突，如果是，生成对应的Conflict信息。
        if previous1 == current2 and previous2 == current1:
            conflicts.append(ConflictInformation(time, ConflictInformation.EDGE, agent1, agent2,
                                                 previous1, current1))

    return conflicts


class GridEnvironment:

    def __init__(self, obstacles, dimensions, solution, higher_agents, stay_at_goal):
        self.obstacles = obstacles
        self.dimensions = dimensions
        self.solution = solution
        self.higher_agents = higher_agents
        self.stay_at_goal = stay_at_goal

    def is_in_map(self, point):
        return 0 <= point[0] < self.dimensions[0] and 0 <= point[1] < self.dimensions[1]

    def is_free(self, point):
        return tuple(point) not in self.obstacles

    def satisfies_higher_agents_vertex(self, point, time, solution, higher_agents):
        # 检查当前状态是否满足所有高优先级智能体的顶点避让。
        point = tuple(point)

        for agent in higher_agents:
            # 高优先级路径可能短于当前时刻，停留模式下按终点位置处理，消失模式下视为不冲突。
            if get_path_position(solution[agent], time, self.stay_at_goal) == point:
                return False

        return True

    def satisfies_higher_agents_edge(self, last_point, point, time, solution, higher_agents):
        # 检查当前状态是否满足所有高优先级智能体的边避让。
        point = tuple(point)
        last_point = tuple(last_point)

        for agent in higher_agents:
            other_current = get_path_position(solution[agent], time, self.stay_at_goal)
            other_previous = get_path_position(solution[agent], time - 1, self.stay_at_goal)

            # 高优先级智能体与本智能体在同一时间交换位置。
            if other_current == last_point and other_previous == point:
                return False

        return True

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
            time = current.time + 1
            if not self.is_in_map(point) or not self.is_free(point):
                continue
            if not self.satisfies_higher_agents_vertex(point, time, self.solution, self.higher_agents):
                continue
            if not self.satisfies_higher_agents_edge(current.position, point, time, self.solution, self.higher_agents):
                continue
            
            neighbors.append(State(point, time))
        
        return neighbors


class ASTAR:

    def __init__(self, name, start, goal, start_time, obstacles, dimensions, solution, stay_at_goal):
        self.name = name
        self.start_state = State(start, start_time)
        self.goal = tuple(goal)
        self.obstacles = obstacles
        self.dimensions = dimensions
        self.solution = solution
        self.stay_at_goal = stay_at_goal
       
        # 终点格从哪一时刻起可以安全停留，在search()中根据高优先级智能体路径计算。
        self.goal_free_time = 0

    def get_goal_free_time(self, higher_agents):
        # 计算终点格从哪一时刻起可以安全停留（stay模式下最后一次被高优先级智能体
        # 占用的时刻+1）
       
        # disappear模式下智能体到达终点后即消失，之后不再参与冲突，返回0。


        if not self.stay_at_goal:
            return 0

        free_time = 0

        for agent in higher_agents:
            path = self.solution.get(agent)
            if not path:
                continue

            if tuple(path[-1].position) == self.goal:
                return None

            for time, state in enumerate(path):
                if tuple(state.position) == self.goal:
                    free_time = max(free_time, time + 1)

        return free_time

    def get_goal_eccentricity(self, blocked):
        # 在"高优先级智能体都已停下"的静态地图（障碍物∪永久占位）上，从终点反向BFS，
        # 返回终点到其连通区域内最远格子的距离。终点被永久占用时返回-1。

        if self.goal in blocked:
            return -1

        distance = {self.goal: 0}
        frontier = [self.goal]

        while frontier:
            next_frontier = []
            for x, y in frontier:
                for point in ((x, y + 1), (x, y - 1), (x - 1, y), (x + 1, y)):
                    if point in distance or point in blocked or point in self.obstacles:
                        continue
                    if not (0 <= point[0] < self.dimensions[0] and 0 <= point[1] < self.dimensions[1]):
                        continue
                    distance[point] = distance[(x, y)] + 1
                    next_frontier.append(point)
            frontier = next_frontier

        return max(distance.values())

    def get_search_horizon(self, higher_agents):
        # 低层搜索的时间上界，保证无解分支能终止
        # T_h为高优先级路径最晚结束时刻，从该时刻起环境静止（stay：停留；disappear：已消失）。
        # 若某解存在，取该解在T_h时刻的位置p，则p到终点在静态图中存在长度不超过终点偏心度E
        # 的简单路径，因此解可在T_h+E内到达终点。剪掉超过该时刻的状态不会漏解。

        last_time = 0
        permanent_blocks = set()

        for agent in higher_agents:
            path = self.solution.get(agent)
            if not path:
                continue
            if len(path) - 1 > last_time:
                last_time = len(path) - 1
            if self.stay_at_goal:
                permanent_blocks.add(path[-1].position)

        eccentricity = self.get_goal_eccentricity(permanent_blocks)
        if eccentricity < 0:
            return -1

        return last_time + eccentricity + 1

    def is_goal_state(self, state):
        # 到达终点还不够：必须从这个时刻起终点不再被高优先级智能体占用，
        # 否则到达后停留在终点会产生冲突，需要继续搜索。
        return state.position == self.goal and state.time >= self.goal_free_time

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

    def search(self, h_values, h_open, h_closed, higher_agents):

        # 转换为二维状态，对应h_values的键，从而能直接读取h_values中的值。
        start_rra_state = StateTwoDimension(self.start_state.position)

        # if start_rra_state not in h_values:
        #     return [], h_values, h_open, h_closed, 0

        # 计算终点从哪一时刻起不会再被高优先级智能体占用；None表示被永久占据，必然无解。
        goal_free_time = self.get_goal_free_time(higher_agents)
        if goal_free_time is None:
            return [], h_values, h_open, h_closed, 0
        self.goal_free_time = goal_free_time

        # 低层搜索的时间上界；-1表示终点被高优先级智能体永久占据，必然无解。
        max_time = self.get_search_horizon(higher_agents)
        if max_time < 0:
            return [], h_values, h_open, h_closed, 0

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

        environment = GridEnvironment(self.obstacles, self.dimensions, self.solution, higher_agents,
                                      self.stay_at_goal)

        while open_set:

            current = min(open_set)
            if self.is_goal_state(current):
                path = self.reconstruct_path(parent, current)  # 生成从起点到终点的路径
                return path, h_values, h_open, h_closed, current.f + 1

            open_set.remove(current)
            del open_index[current.key]
            closed_set.add(current)

            for neighbor in environment.get_neighbors(current): # 遍历当前状态的所有邻居，寻找可行的扩展节点。
                # 超过时间上界仍未到达终点，说明该优先级顺序下无解。
                if neighbor.time > max_time:
                    continue
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

        return [], h_values, h_open, h_closed, 0
