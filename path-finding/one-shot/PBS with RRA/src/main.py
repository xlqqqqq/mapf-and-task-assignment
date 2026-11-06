"""PBS高层约束数的搜索。"""

import datetime
import sys
from copy import deepcopy
import os
import yaml

from astar import ASTAR, find_pair_conflicts
from RRA import ASTAR_TwoDimension
from priority import PriorityDAG


class HighLevelNode:
    # PBS约束树中的一个高层节点。

    def __init__(self, agent_names):
        self.solution = {}  # 每个智能体的路径
        self.f_value_dict = {}  # 每个智能体的f值
        self.cost = 0  # 总代价
        self.enter_sequence = 0  # 节点进入顺序，主要是消除随机性，确保每次搜索结果一致。
        self.priority_dag = PriorityDAG(agent_names)  # 优先级图
        self.depth = 0  # 深度
        self.conflicts = []  # 冲突列表
        self.conflicts_number = 0  # 冲突数

    def __lt__(self, other):
        # 高层排序保持为:深度、总代价、冲突数、进入顺序。

        return (self.depth, self.cost, self.conflicts_number, self.enter_sequence) \
            < (other.depth, other.cost, other.conflicts_number, other.enter_sequence)

    def state_key(self):
        # 高层状态由优先级图和全部路径唯一确定。
        priorities = frozenset(
            (higher_agent, lower_agent)
            for lower_agent, higher_agents in self.priority_dag.higher.items()
            for higher_agent in higher_agents
        )
        paths = tuple(sorted(
            (agent_name, tuple((state.time, state.x, state.y) for state in path))
            for agent_name, path in self.solution.items()
        ))
        return priorities, paths

    def __eq__(self, other):
        return self.state_key() == other.state_key()

    def __hash__(self):
        return hash(self.state_key())


def make_agent_dict(agents):
    # 以智能体名称为键保存起点和终点。

    return {agent["name"]: {"start": agent["start"], "goal": agent["goal"]} for agent in agents}


def compute_solution_cost(solution):
    # 代价是所有路径状态数量之和。

    return sum(len(path) for path in solution.values())


def generate_plan(solution):
    # 把State路径转换成YAML可以直接保存的字典。

    plan = {}
    for agent_name, path in solution.items():
        plan[agent_name] = [{"t": state.time, "x": state.x, "y": state.y} for state in path]
    return plan


def sort_conflicts(conflicts, agent_names):
    # 按冲突时间对冲突列表排序。

    # agent_order = {agent_name: index for index, agent_name in enumerate(agent_names)}
    conflicts.sort(key=lambda conflict: conflict.time)


def update_priority_dag(update_node, higher_agent, lower_agent):
    # 更新PBS约束树中的优先级图，返回需要重规划的低优先级智能体拓扑序；
    # add_priority(i, j) 表示i的优先级低于j，因此被降级的智能体作为第一个参数。
    status, affected = update_node.priority_dag.add_priority(lower_agent, higher_agent)

    # CYCLE说明再加这条边会自相矛盾；IMPLIED说明优先级关系已经通过传递关系成立，不需要加边；
    if status == PriorityDAG.CYCLE or status == PriorityDAG.IMPLIED:
        return None

    order = update_node.priority_dag.topological_sort(affected)

    return order


def collect_conflicts(replanned_agents, solution, stay_at_goal):
    # 重规划全部结束后，按最终路径统计重规划智能体产生的新冲突。
    # 之所以不再在重规划循环里边搜边记，是因为轮到一个智能体时，排在它后面的低优先级后代
    # 还持有旧路径，因此当低优先级后代重规划路径后，此前高优先级记下的冲突在最终解里可能已经不存在。

    conflicts = []
    replanned_names = [agent_name for agent_name in solution if agent_name in replanned_agents]

    for index, agent1 in enumerate(replanned_names):
        path1 = solution[agent1]

        # 重规划的智能体之间彼此配对，统计冲突。
        for agent2 in replanned_names[index + 1:]:
            conflicts.extend(find_pair_conflicts(agent1, path1, agent2, solution[agent2], stay_at_goal))

        # 与未被重规划的智能体配对，统计冲突。
        for agent2, path2 in solution.items():
            if agent2 in replanned_agents:
                continue
            conflicts.extend(find_pair_conflicts(agent1, path1, agent2, path2, stay_at_goal))

    return conflicts


def search_pbs(agents, obstacles, dimensions, stay_at_goal):
    # 转换为字典，键为智能体名称，值为包含起点和终点的字键值对。
    agent_dict = make_agent_dict(agents)

    # 存取智能体名称
    agent_names = [agent["name"] for agent in agents]

    # 初始化高层节点
    start_node = HighLevelNode(agent_names)

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

        higher_agents = set()

        astar = ASTAR(agent_name, agent["start"], agent["goal"], 0, obstacles, dimensions,
                      start_node.solution,
                      stay_at_goal)

        # 计算每个智能体的路径，输入为RRA搜索结果即h值，返回路径、h值、open集、closed集、f值。
        path, h_values, rra_open, rra_closed, f_value = astar.search(h_value_dict[agent_name],
                                                                     rra_open_dict[agent_name],
                                                                     rra_closed_dict[agent_name],
                                                                     higher_agents)

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

    # 所有智能体的初始路径确定后，再统一统计冲突，与子节点的冲突统计方式保持一致。
    start_node.conflicts = collect_conflicts(set(agent_names), start_node.solution, stay_at_goal)
    start_node.conflicts_number = len(start_node.conflicts)
    start_node.cost = sum(start_node.f_value_dict.values())

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

        # PBS：分别尝试两种优先序，让冲突中的一方高于另一方。
        for higher_agent, lower_agent in ((first_conflict.agent1, first_conflict.agent2),
                                          (first_conflict.agent2, first_conflict.agent1)):

            child_node = deepcopy(current_node)
            order = update_priority_dag(child_node, higher_agent, lower_agent)
            if order is None:
                continue

            # 按拓扑序重规划被降级的智能体及其所有低优先级后代。
            # 拓扑序保证轮到一个智能体时，它避让的高优先级路径都已经更新完毕。
            replanned_agents = set()
            branch_valid = True

            for agent in order:

                # 获取所有优先级高于当前智能体的高优先级智能体。
                higher_agents = child_node.priority_dag.get_ancestors(agent)

                astar = ASTAR(agent, agent_dict[agent]["start"],
                              agent_dict[agent]["goal"],
                              0,
                              obstacles,
                              dimensions,
                              child_node.solution,
                              stay_at_goal)

                path, h_values, rra_open, rra_closed, f_value = astar.search(
                    h_value_dict[agent], rra_open_dict[agent],
                    rra_closed_dict[agent], higher_agents)

                low_level_searches += 1

                h_value_dict[agent] = h_values
                rra_open_dict[agent] = rra_open
                rra_closed_dict[agent] = rra_closed

                # 只要有智能体无解，整个子节点就不可行。
                if not path:
                    branch_valid = False
                    break

                child_node.solution[agent] = path
                child_node.f_value_dict[agent] = f_value
                replanned_agents.add(agent)

            if not branch_valid:
                continue

            # 统计重规划智能体产生的新冲突。
            new_conflicts = collect_conflicts(replanned_agents, child_node.solution, stay_at_goal)

            child_node.cost = sum(child_node.f_value_dict.values())
            child_node.enter_sequence = enter_sequence
            enter_sequence += 1
            child_node.depth += 1

            # 所有被重规划智能体的旧冲突都已失效，保留其余智能体之间原有的冲突，
            # 再加入本次重规划产生的新冲突。
            child_node.conflicts = [conflict for conflict in child_node.conflicts
                                    if conflict.agent1 not in replanned_agents
                                    and conflict.agent2 not in replanned_agents]
            child_node.conflicts.extend(new_conflicts)

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

    # 运行PBS
    solution_node, statistics = search_pbs(agents, obstacles, dimensions, stay_at_goal)

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
