"""CBS高层约束数的搜索。"""

import datetime
import sys
from copy import deepcopy
import os
import yaml

from astar import ASTAR, ConflictInformation as Conflict, EdgeConstraint, VertexConstraint
from RRA import ASTAR_TwoDimension


class HighLevelNode:
    # CBS约束树中的高层节点。

    def __init__(self):
        self.solution = {}  # 每个智能体的路径
        self.vertex_constraint_dict = {}  # 每个智能体的顶点约束
        self.edge_constraint_dict = {}  # 每个智能体的边约束
        self.f_value_dict = {}  # 每个智能体的f值
        self.cost = 0  # 总代价
        self.enter_sequence = 0  # 节点进入顺序，主要是消除随机性，确保针对同一实例每次搜索结果一致。
        self.conflicts = []  # 冲突列表
        self.conflicts_number = 0  # 冲突数

    def constraint_signature(self):
        # 把约束字典转换成不可修改的tuple，用于节点判重。
        signature = []
        for agent_name in sorted(self.vertex_constraint_dict):
            vertex_constraints = frozenset(self.vertex_constraint_dict[agent_name])
            edge_constraints = frozenset(self.edge_constraint_dict[agent_name])
            signature.append((agent_name, vertex_constraints, edge_constraints))
        return tuple(signature)

    def __eq__(self, other):
        return self.constraint_signature() == other.constraint_signature()

    def __hash__(self):
        return hash(self.constraint_signature())

    def __lt__(self, other):
        # 高层排序保持为：总代价、冲突数、进入顺序。

        return (self.cost, self.conflicts_number, self.enter_sequence) \
            < (other.cost, other.conflicts_number, other.enter_sequence)


def make_agent_dict(agents):
    # 以智能体名称为键保存起点和终点。

    return {agent["name"]: {"start": agent["start"], "goal": agent["goal"]} for agent in agents}


def compute_solution_cost(solution):
    # 代价是所有路径状态数量之和。

    return sum(len(path) for path in solution.values())


def sort_conflicts(conflicts, agent_names):
    # 按冲突时间对冲突列表排序。

    # agent_order = {agent_name: index for index, agent_name in enumerate(agent_names)}
    conflicts.sort(key=lambda conflict: conflict.time)


def add_constraint_from_conflict(node, conflict, constrained_agent):
    # 为冲突中的一个智能体添加对应约束。

    if conflict.type == Conflict.VERTEX:
        constraint = VertexConstraint(conflict.time, conflict.location1)
        node.vertex_constraint_dict[constrained_agent].add(constraint)

    else:
        # 在记录边约束信息时，默认存储的是conflict.agent1的(from_location,to_location),所以如果constrained_agent是conflict.agent1,
        # 则直接添加(from_location,to_location),否则添加(to_location,from_location).
        if constrained_agent == conflict.agent1:
            constraint = EdgeConstraint(conflict.time, conflict.location1, conflict.location2)
            node.edge_constraint_dict[constrained_agent].add(constraint)
        else:
            constraint = EdgeConstraint(conflict.time, conflict.location2, conflict.location1)
            node.edge_constraint_dict[constrained_agent].add(constraint)


def generate_plan(solution):
    # 把State路径转换成YAML可以直接保存的字典。

    plan = {}
    for agent_name, path in solution.items():
        plan[agent_name] = [{"t": state.time, "x": state.x, "y": state.y} for state in path]
    return plan


def search_cbs(agents, obstacles, dimensions, stay_at_goal):
    # 转换为字典，键为智能体名称，值为包含起点和终点的字键值对。
    agent_dict = make_agent_dict(agents)

    # 存取智能体名称
    agent_names = [agent["name"] for agent in agents]

    # 初始化高层节点
    start_node = HighLevelNode()

    # 初始化每个智能体的顶点约束和边约束为空集
    for agent_name in agent_names:
        start_node.vertex_constraint_dict[agent_name] = set()
        start_node.edge_constraint_dict[agent_name] = set()

    # 每个智能体的RRA搜索结果即H值分开保存，同时保存RRA搜索过程中的open和closed集，方便后续继续扩展。
    h_value_dict = {}
    rra_open_dict = {}
    rra_closed_dict = {}

    # 对每个智能体进行RRA搜索，预先计算h值。
    for agent in agents:
        # 反向搜索，由终点到起点，计算h值。
        rra = ASTAR_TwoDimension(agent["goal"], agent["start"], obstacles, dimensions)
        h_values, rra_open, rra_closed = rra.search()
        agent_name = agent["name"]
        h_value_dict[agent_name] = h_values
        rra_open_dict[agent_name] = rra_open
        rra_closed_dict[agent_name] = rra_closed

    # 记录低层A*搜索次数。
    low_level_searches = 0

    for agent in agents:
        agent_name = agent["name"]

        astar = ASTAR(agent_name, agent["start"], agent["goal"], 0, obstacles, dimensions,
                      start_node.vertex_constraint_dict[agent_name], start_node.edge_constraint_dict[agent_name],
                      start_node.solution,
                      stay_at_goal)

        # 计算每个智能体的路径，输入为RRA搜索结果即h值，返回路径、冲突、h值、open集、closed集、f值。
        path, path_conflicts, h_values, rra_open, rra_closed, f_value = astar.search(h_value_dict[agent_name],
                                                                                     rra_open_dict[agent_name],
                                                                                     rra_closed_dict[agent_name])
        low_level_searches += 1

        h_value_dict[agent_name] = h_values
        rra_open_dict[agent_name] = rra_open
        rra_closed_dict[agent_name] = rra_closed

        if not path:
            return None, {
                "generated_ct_nodes": 0,
                "expanded_ct_nodes": 0,
                "low_level_searches": low_level_searches
            }

        start_node.solution[agent_name] = path
        start_node.f_value_dict[agent_name] = f_value
        start_node.conflicts.extend(path_conflicts)

    start_node.cost = sum(start_node.f_value_dict.values())

    start_node.conflicts_number = len(start_node.conflicts)

    # 对冲突进行排序，确保先处理时间早的冲突。
    sort_conflicts(start_node.conflicts, agent_names)

    open_set = {start_node}
    closed_set = set()
    enter_sequence = 1

    # 记录生成的高层节点数量。
    generated_ct_nodes = 1

    # 记录被扩展的高层节点数量。
    expanded_ct_nodes = 0

    while open_set:
        current_node = min(open_set)
        open_set.remove(current_node)
        expanded_ct_nodes += 1

        # 如果当前节点没有冲突，说明已经到达目标点，返回当前节点。
        if not current_node.conflicts:
            statistics = {
                "generated_ct_nodes": generated_ct_nodes,
                "expanded_ct_nodes": expanded_ct_nodes,
                "low_level_searches": low_level_searches,
            }
            return current_node, statistics

        closed_set.add(current_node)

        # 从当前节点中取出第一个冲突。
        first_conflict = current_node.conflicts[0]

        # 标准CBS：分别约束冲突双方，两个可行子节点都加入open。
        for constrained_agent in (first_conflict.agent1, first_conflict.agent2):
            child_node = deepcopy(current_node)

            add_constraint_from_conflict(child_node, first_conflict, constrained_agent)

            astar = ASTAR(constrained_agent, agent_dict[constrained_agent]["start"],
                          agent_dict[constrained_agent]["goal"],
                          0,
                          obstacles,
                          dimensions,
                          child_node.vertex_constraint_dict[constrained_agent],
                          child_node.edge_constraint_dict[constrained_agent],
                          child_node.solution,
                          stay_at_goal)

            path, path_conflicts, h_values, rra_open, rra_closed, f_value = astar.search(
                h_value_dict[constrained_agent], rra_open_dict[constrained_agent],
                rra_closed_dict[constrained_agent])

            low_level_searches += 1

            h_value_dict[constrained_agent] = h_values
            rra_open_dict[constrained_agent] = rra_open
            rra_closed_dict[constrained_agent] = rra_closed

            # 一个子节点无解时只丢弃该分支，继续搜索另一个分支。
            if not path:
                continue

            child_node.solution[constrained_agent] = path
            child_node.f_value_dict[constrained_agent] = f_value
            child_node.cost = sum(child_node.f_value_dict.values())
            child_node.enter_sequence = enter_sequence
            enter_sequence += 1

            # 只有constrained_agent的路径发生了变化。
            # 所以保留其他智能体之间原有的冲突，再加入A*返回的新路径冲突，就构成新的冲突列表。
            child_node.conflicts = [conflict for conflict in child_node.conflicts
                                    if conflict.agent1 != constrained_agent
                                    and conflict.agent2 != constrained_agent]
            child_node.conflicts.extend(path_conflicts)
            
            child_node.conflicts_number = len(child_node.conflicts)
            sort_conflicts(child_node.conflicts, agent_names)

            # 子节点加入open集。
            if child_node not in closed_set:
                old_size = len(open_set)
                open_set.add(child_node)
                if len(open_set) > old_size:
                    generated_ct_nodes += 1

    statistics = {
        "generated_ct_nodes": generated_ct_nodes,
        "expanded_ct_nodes": expanded_ct_nodes,
        "low_level_searches": low_level_searches}
    return None, statistics


def main():
    # 在PyCharm中直接点击运行或调试按钮时，会使用下面三个参数。

    # 获取项目根目录
    PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
    INPUT_FILE = os.path.join(PROJECT_DIR, "../maps/map_warehouse/warehouse_map.yaml")
    OUTPUT_FILE = os.path.join(PROJECT_DIR, "output.yaml")
    STAY_AT_GOAL = True

    # 如果提供了命令行参数，则用命令行参数覆盖默认配置。
    input_path = sys.argv[1] if len(sys.argv) >= 2 else INPUT_FILE
    output_path = sys.argv[2] if len(sys.argv) >= 3 else OUTPUT_FILE
    stay_at_goal = STAY_AT_GOAL

    if len(sys.argv) >= 4:
        if sys.argv[3] == "stay":
            stay_at_goal = True
        elif sys.argv[3] == "disappear":
            stay_at_goal = False
        else:
            print("终点行为只能是stay或disappear")
            return 1

    # 记录算法开始时间
    start_time = datetime.datetime.now()

    # 读取输入文件
    with open(input_path, "r", encoding="utf-8") as input_file:
        parameters = yaml.load(input_file, Loader=yaml.FullLoader)

    # 存取地图大小
    dimensions = parameters["map"]["dimensions"]

    # 存取障碍物
    obstacles = {
        tuple(location) for location in parameters["map"]["obstacles"]
    }

    # 存取智能体起点终点和智能体名称
    agents = parameters["agents"]

    # 运行CBS
    solution_node, statistics = search_cbs(agents, obstacles, dimensions, stay_at_goal)

    if solution_node is None:
        print("no solution")
        return 1

    output = {
        "schedule": generate_plan(solution_node.solution),
        "cost": compute_solution_cost(solution_node.solution),
    }
    with open(output_path, "w", encoding="utf-8") as output_file:
        yaml.safe_dump(output, output_file, sort_keys=False)

    # 计算执行时间
    execution_time = datetime.datetime.now() - start_time
    print("solution found")
    print("cost:", solution_node.cost)
    print("generated CT nodes:", statistics["generated_ct_nodes"])
    print("expanded CT nodes:", statistics["expanded_ct_nodes"])
    print("low-level searches:", statistics["low_level_searches"])
    print("execution time:", execution_time)
    return 0


if __name__ == "__main__":
    sys.exit(main())
