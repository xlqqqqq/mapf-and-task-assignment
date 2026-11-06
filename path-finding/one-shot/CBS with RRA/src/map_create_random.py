"""生成随机障碍物情况下的MAPF测试用例。"""

import os
import random

import yaml


# ========================== 在这里修改生成参数 ==========================

MAP_WIDTH = 30               # 地图宽度，对应 x 的取值范围 0 ~ 19
MAP_HEIGHT = 30              # 地图高度，对应 y 的取值范围 0 ~ 19
OBSTACLE_RATIO = 0.20        # 障碍物数量占地图总格子数的比例
AGENT_COUNT = 10             # 智能体数量
PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_FILE = os.path.join(PROJECT_DIR, "../maps/map_random/random_map.yaml")

# 设置为整数可以重复生成完全相同的地图，例如 RANDOM_SEED = 10。
# 设置为 None 时，每次运行都会生成不同的地图。
RANDOM_SEED = None


def is_reachable(free_positions, start, goal):
    """判断两个自由格子之间是否连通。"""

    # 从起点出发，用栈做深度优先遍历，直到找到终点或遍历完。
    visited = {start}
    stack = [start]

    while stack:
        x, y = stack.pop()
        if (x, y) == goal:
            return True
        for neighbor in (
            (x, y + 1),
            (x, y - 1),
            (x - 1, y),
            (x + 1, y),
        ):
            if neighbor in free_positions and neighbor not in visited:
                visited.add(neighbor)
                stack.append(neighbor)

    return False


def create_random_map(
    width,
    height,
    obstacle_ratio,
    agent_count,
    output_file,
    random_seed=None,
):
    """生成随机地图并保存为YAML文件。"""

    if width <= 0 or height <= 0:
        raise ValueError("地图的宽度和高度必须大于 0")
    if not 0 <= obstacle_ratio < 1:
        raise ValueError("障碍物比例必须满足 0 <= ratio < 1")
    if agent_count <= 0:
        raise ValueError("智能体数量必须大于 0")

    # random.seed() 用于控制随机数。
    # 当种子相同时，生成的障碍物、起点和终点也相同，方便重复实验。

    random.seed(random_seed)

    all_positions = [
        (x, y)
        for x in range(width)
        for y in range(height)
    ]

    total_cells = width * height
    obstacle_count = round(total_cells * obstacle_ratio)
    free_cell_count = total_cells - obstacle_count

    # 每个智能体需要一个起点和一个终点。这里让所有起点和终点都不重复，
    # 因此至少需要 agent_count * 2 个非障碍物格子。
    if free_cell_count < agent_count * 2:
        raise ValueError(
            "非障碍物格子不足，请减小障碍物比例或智能体数量"
        )

    # random.sample(sequence, number) 会从序列中随机抽取不重复的元素。
    obstacles = set(random.sample(all_positions, obstacle_count))
    free_positions = [
        position
        for position in all_positions
        if position not in obstacles
    ]

    # 起点和终点必须互相可达，否则低层搜索会因隔断而失败。
    # 障碍物比例过高时可能始终采样不出可达的起点和终点，这里限制重试次数，避免死循环。
    max_attempts = 100
    for _ in range(max_attempts):
        start_and_goal_positions = random.sample(
            free_positions,
            agent_count * 2,
        )
        start_positions = start_and_goal_positions[:agent_count]
        goal_positions = start_and_goal_positions[agent_count:]

        # 检查每一对起点-终点是否可达。
        if all(
            is_reachable(free_positions, start, goal)
            for start, goal in zip(start_positions, goal_positions)
        ):
            break
    else:
        raise ValueError(
            "尝试100次后仍无法为所有智能体生成可达的起点和终点，请降低障碍物比例"
        )
    agents = []
    for index in range(agent_count):
        agents.append(
            {
                # 测试文件中的 start 和 goal 是列表，所以这里使用 list()。
                "start": list(start_positions[index]),
                "goal": list(goal_positions[index]),
                "name": "agent" + str(index),
            }
        )

    output = {
        "agents": agents,
        "map": {
            "dimensions": [width, height],
            # 障碍物使用 tuple，yaml.dump() 会输出 !!python/tuple 标签，
            # 与原项目测试文件的障碍物格式一致。
            "obstacles": sorted(obstacles),
        },
    }

    output_directory = os.path.dirname(output_file)
    if output_directory:
        os.makedirs(output_directory, exist_ok=True)

    with open(output_file, "w", encoding="utf-8") as yaml_file:
        # 这里不能使用 safe_dump()，因为 safe_dump() 默认不支持 tuple。
        # default_flow_style=None 会把 [x, y] 这样的简单列表写在一行。
        yaml.dump(
            output,
            yaml_file,
            Dumper=yaml.Dumper,
            sort_keys=False,
            default_flow_style=None,
        )

    print("随机测试用例生成完成")
    print("地图大小:", width, "x", height)
    print("智能体数量:", agent_count)
    print("输出文件:", output_file)


if __name__ == "__main__":
    create_random_map(
        MAP_WIDTH,
        MAP_HEIGHT,
        OBSTACLE_RATIO,
        AGENT_COUNT,
        OUTPUT_FILE,
        RANDOM_SEED,
    )

