"""生成仓库地图下的MAPF测试用例。"""

import os
import random

import yaml


# ========================== 在这里修改生成参数 ==========================

MAP_WIDTH = 30               # 地图宽度，对应 x 的取值范围 0 ~ 29
MAP_HEIGHT = 10              # 地图高度，对应 y 的取值范围 0 ~ 9
AGENT_COUNT = 10             # 智能体数量

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_FILE = os.path.join(PROJECT_DIR, "../maps/map_warehouse/warehouse_map.yaml")

# 设置为整数可以重复生成完全相同的地图，例如 RANDOM_SEED = 10。
# 设置为 None 时，每次运行都会生成不同的地图。
RANDOM_SEED = None


def create_warehouse_map(
    width,
    height,
    agent_count,
    output_file,
    random_seed=None,
):
    """生成随机地图并保存为YAML文件。"""

    if width <= 0 or height <= 0:
        raise ValueError("地图的宽度和高度必须大于 0")
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

    obstacles = {
        (x + i, y + j)
        for x in (5, 12, 19)
        for y in (1, 4, 7)
        for i in range(6)
        for j in range(2)
    }

    free_positions = [
        position
        for position in all_positions
        if position not in obstacles
    ]

    start_and_goal_positions = random.sample(
        free_positions,
        agent_count * 2,
    )
    start_positions = start_and_goal_positions[:agent_count]
    goal_positions = start_and_goal_positions[agent_count:]

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
    create_warehouse_map(
        MAP_WIDTH,
        MAP_HEIGHT,
        AGENT_COUNT,
        OUTPUT_FILE,
        RANDOM_SEED,
    )

