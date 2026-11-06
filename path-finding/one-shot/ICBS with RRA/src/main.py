import datetime
import sys
from copy import deepcopy
import os
import yaml

from astar import ASTAR, ConflictInformation as Conflict, EdgeConstraint, VertexConstraint
from RRA import ASTAR_TwoDimension
from mdd import compute_mdd


class HighLevelNode:
    # ICBS约束树中的一个高层节点。

    def __init__(self):
        self.solution = {}  # 每个智能体的路径
        self.vertex_constraint_dict = {}  # 每个智能体的顶点约束
        self.edge_constraint_dict = {}  # 每个智能体的边约束
        self.f_value_dict = {}  # 每个智能体的f值
        self.cost = 0  # 总代价
        self.enter_sequence = 0  # 节点进入顺序，主要是消除随机性，确保针对同一实例每次搜索结果一致。
        self.conflicts = [[] for _ in range(0, 3)]  # 分为三个子列表，分别对应三种不同类型的冲突
        self.conflicts_number = 0  # 冲突数
        self.mdd = {}  # 每个智能体的MDD

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

    conflicts[0].sort(key=lambda conflict: conflict.time)
    conflicts[1].sort(key=lambda conflict: conflict.time)
    conflicts[2].sort(key=lambda conflict: conflict.time)


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


def mdd_layer_size(mdd, agent_name, time, stay_at_goal):
    # 取得某智能体在指定时刻的MDD层大小，用于判断冲突是否为cardinal。

    if time < 0:
        return 0

    # MDD只覆盖该智能体自己路径的时间范围，层索引为 0..len(path)-1。
    # 路径结束之后：
    # stay模式下智能体不消失，位置按路径终点补齐，因此这些时间层只有
    # 终点这一个状态，层大小记为1。注意这不代表它"到达终点后就不能动"——
    # 离开终点让路再返回都发生在它自己的路径内部，路径最后一步仍是终点。
    # disappear模式下路径结束后不再有状态，冲突不会出现在这些时间点。
    if time >= len(mdd[agent_name]):
        return 1 if stay_at_goal else 0

    return len(mdd[agent_name][time])


def is_forced(mdd, agent_name, time, stay_at_goal):
    # 该智能体在指定时刻是否只有一个可能位置，即被强制。
    return mdd_layer_size(mdd, agent_name, time, stay_at_goal) == 1


def classify_conflict(all_conflicts, conflicts, mdd, stay_at_goal):
    for conflict in all_conflicts:

        if conflict.type == Conflict.VERTEX:
            # 顶点冲突：判断t层是否只有一个可行状态。
            forced_1 = is_forced(mdd, conflict.agent1, conflict.time, stay_at_goal)
            forced_2 = is_forced(mdd, conflict.agent2, conflict.time, stay_at_goal)
        else:
            # 边冲突：只有出发层(time-1)和到达层(time)都只有一个可行状态，
            # 才能确定这条边是被强制的。
            forced_1 = (is_forced(mdd, conflict.agent1, conflict.time - 1, stay_at_goal)
                        and is_forced(mdd, conflict.agent1, conflict.time, stay_at_goal))
            forced_2 = (is_forced(mdd, conflict.agent2, conflict.time - 1, stay_at_goal)
                        and is_forced(mdd, conflict.agent2, conflict.time, stay_at_goal))

        # 判断是否为cardinal冲突
        if forced_1 and forced_2:
            conflicts[0].append(conflict)
        # 判断是否为semi-cardinal冲突
        elif forced_1 != forced_2:
            conflicts[1].append(conflict)
        else:
            conflicts[2].append(conflict)


def search_icbs(agents, obstacles, dimensions, stay_at_goal):
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

    # 存取所有冲突，方便后续分类处理
    all_conflicts = []

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

        all_conflicts.extend(path_conflicts)

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

    # 计算每个智能体的MDD
    for agent in agents:
        agent_name = agent["name"]
        start_node.mdd[agent_name] = compute_mdd(obstacles, start_node.edge_constraint_dict[agent_name],
                                                 start_node.vertex_constraint_dict[agent_name],
                                                 len(start_node.solution[agent_name]),
                                                 agent_dict[agent_name]['start'],
                                                 agent_dict[agent_name]['goal'], dimensions)

    start_node.cost = sum(start_node.f_value_dict.values())
    start_node.conflicts_number = len(all_conflicts)
    # 对冲突进行分类
    classify_conflict(all_conflicts, start_node.conflicts, start_node.mdd, stay_at_goal)

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

        # 默认执行BP
        flag_if_bp = True

        # 记录BP是否命中，命中后只保留该子节点，丢弃另一个分支
        bp_triggered = False

        # 执行BP时，用于存储子节点
        normal_child_node = []

        current_node = min(open_set)
        open_set.remove(current_node)
        expanded_ct_nodes += 1

        # 如果当前节点没有冲突，说明已经到达目标点，返回当前节点。
        # conflicts分为三个子列表，只有三个子列表都为空才算无冲突。
        if not any(current_node.conflicts):
            statistics = {
                "generated_ct_nodes": generated_ct_nodes,
                "expanded_ct_nodes": expanded_ct_nodes,
                "low_level_searches": low_level_searches,
            }
            return current_node, statistics

        closed_set.add(current_node)

        # 如果当前冲突是cardinal冲突，则无需执行BP
        if len(current_node.conflicts[0]) != 0:
            first_conflict = current_node.conflicts[0][0]
            flag_if_bp = False
        elif len(current_node.conflicts[1]) != 0:
            first_conflict = current_node.conflicts[1][0]
        else:
            first_conflict = current_node.conflicts[2][0]

        # ICBS：分别约束冲突双方，两个可行子节点都加入open。
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

            # 把三种不同类型的冲突合并起来，方便后续处理

            child_node_conflicts = [conflict for ele in child_node.conflicts for conflict in ele]
            # 只有constrained_agent的路径发生了变化。
            # 所以保留其他智能体之间原有的冲突，再加入A*返回的新路径冲突，就构成新的冲突列表。
            all_conflicts = [conflict for conflict in child_node_conflicts
                             if conflict.agent1 != constrained_agent
                             and conflict.agent2 != constrained_agent]

            all_conflicts.extend(path_conflicts)

            child_node.conflicts_number = len(all_conflicts)
            # 重新初始化冲突列表，方便后续填入
            child_node.conflicts = [[] for _ in range(0, 3)]

            # 其余智能体路径不变，因此只更新变化了路径的智能体的MDD
            child_node.mdd[constrained_agent] = compute_mdd(obstacles,
                                                            child_node.edge_constraint_dict[constrained_agent],
                                                            child_node.vertex_constraint_dict[constrained_agent],
                                                            len(child_node.solution[constrained_agent]),
                                                            agent_dict[constrained_agent]['start'],
                                                            agent_dict[constrained_agent]['goal'], dimensions)

            classify_conflict(all_conflicts, child_node.conflicts, child_node.mdd, stay_at_goal)

            sort_conflicts(child_node.conflicts, agent_names)

            if flag_if_bp:
                if child_node.cost == current_node.cost and child_node.conflicts_number < current_node.conflicts_number:
                    if child_node not in closed_set:
                        old_size = len(open_set)
                        open_set.add(child_node)
                        if len(open_set) > old_size:
                            generated_ct_nodes += 1
                    bp_triggered = True
                    break
                else:
                    normal_child_node.append(child_node)
            else:
                if child_node not in closed_set:
                    old_size = len(open_set)
                    open_set.add(child_node)
                    if len(open_set) > old_size:
                        generated_ct_nodes += 1

        # BP没有命中时，把所有有解的子节点都加入open。
        if flag_if_bp and not bp_triggered:
            for child_node in normal_child_node:
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
    # 运行ICBS
    solution_node, statistics = search_icbs(agents, obstacles, dimensions, stay_at_goal)

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
