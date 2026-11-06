# 多智能体路径规划与任务分配系列算法

本项目开源个人研究生期间复现的算法，主要面向多智能体路径规划与任务分配领域，包括：

- **One-shot MAPF**：CBS、ICBS、ECBS、PBS（已实现）；EECBS、PIBT（未更新）
- **Life-long MAPF**：RHCR（未更新）
- **任务分配**：Google OR-Tools、ALNS、DRLH（未更新）
- **ROS 仿真**：基于Gazebo和TurtleBot3 Burger的仿真实现（未更新）

算法均基于Python实现，个人水平有限，所有算法均以最简单的逻辑和语法实现，可能会有冗余和不高效的地方，请谅解。希望对你有所帮助。欢迎Star。
(CBS系列算法的整体框架参考自早年在GitHub上找到的一个开源项目，但原仓库链接已丢失，暂时无法标注出处。如有冒犯，敬请谅解，也欢迎知情者告知该项目的来源，以便补上出处。)

## 算法及相关说明

> 状态说明：“已更新”表示算法已实现并开源；“未更新”表示尚未上传，还在整理中。

### One-shot MAPF

| 算法 | 全称 | 特点 | 状态 | 说明 |
| --- | --- | --- | --- | --- |
| CBS | Conflict-Based Search | 兼具完备性和最优性 | 已更新 | 经典 MAPF 最优求解算法 |
| ICBS | Improved CBS | 兼具完备性和最优性 | 已更新 | 在 CBS 基础上对冲突分类（cardinal/semi-cardinal/non-cardinal），优先处理cardinal冲突，并使用绕过（bypass）策略减少高层节点。我们未实现原论文中的Merge和Restart策略 |
| ECBS | Enhanced CBS | 完备，有界次优 | 已更新 | 在 CBS 基础上通过聚焦搜索（Focal Search）加快求解，采用次优权重（默认 w=1.05，可在 `main.py` 中调整），并以冲突数量作为辅助启发值（Conflict Heuristics） |
| PBS | Priority-Based Search | 不完备，无界次优 | 已更新 | 通过动态维护智能体优先级图（DAG），按拓扑序重规划低优先级智能体，快速得到无冲突路径 |
| EECBS | Explicit Estimation CBS | 完备，有界次优 | 未更新 | 在 ECBS 基础上引入显式估计搜索（EES）和在线学习的高层启发值，进一步提升求解效率 |
| PIBT | Priority Inheritance with Backtracking | 不完备，无最优性保证 | 未更新 | 基于优先级继承与回溯的迭代式求解器，每个时间步求解快、可扩展性好，适合life-long MAPF和大规模智能体场景 |

上述已实现的四种算法均使用RRA（Reverse Resumable A*）作为低层A*的启发式：从目标点反向搜索得到精确距离，减少低层搜索的无效扩展。

**相关论文**

- CBS：[PDF](https://cdn.aaai.org/ojs/18222/18222-77-21738-1-2-20210717.pdf)
- ICBS：[PDF](https://ijcai.org/proceedings/2015/0110.pdf)
- ECBS：[PDF](https://scispace.com/pdf/suboptimal-variants-of-the-conflict-based-search-algorithm-4kjbc9vt4n.pdf)
- PBS：[PDF](https://scispace.com/pdf/searching-with-consistent-prioritization-for-multi-agent-1a38z9a6ds.pdf)
- EECBS：[PDF](https://cdn.aaai.org/ojs/17466/17466-13-20960-1-2-20210518.pdf)
- PIBT：[PDF](https://ijcai.org/Proceedings/2019/0076.pdf)

### Life-long MAPF

| 算法 | 全称 | 状态 | 说明 |
| --- | --- | --- | --- |
| RHCR | Rolling-Horizon Collision Resolution | 未更新 | 将life-long MAPF划分为多个时间窗口，在每个窗口内求解one-shot子问题并执行前几步，滚动推进 |

**相关论文**

- RHCR：[PDF](https://cdn.aaai.org/ojs/17344/17344-13-20838-1-2-20210518.pdf)

### 任务分配

| 算法 | 全称 | 状态 | 说明 |
| --- | --- | --- | --- |
| Google OR-Tools | OR-Tools | 未更新 | Google 开源的优化套件，可用于任务分配问题的建模与求解 |
| ALNS | Adaptive Large Neighborhood Search | 未更新 | 自适应大邻域搜索，通过破坏-修复算子与自适应算子选择求解任务分配 |
| DRLH | Deep Reinforcement Learning Hyperheuristic | 未更新 | 使用强化学习得到的策略网络替代 ALNS 的自适应算子选择层 |


**相关链接**
- 任务分配模型参考：[PDF](https://www.cs.sfu.ca/~hangma/pub/aamas17.pdf)
- OR-Tools：[官方文档](https://developers.google.cn/optimization/routing/pickup_delivery#python)
- ALNS：[WEB](https://pubsonline.informs.org/doi/epdf/10.1287/trsc.1050.0135)
- DRLH：[PDF](https://repository.uantwerpen.be/docman/irua/b2b398/196806.pdf)



## 环境要求

### 路径规划

- Python >= 3.8.5
- PyYAML
- tkinter（仅可视化功能需要；Windows/macOS通常自带，Ubuntu 需执行 `sudo apt install python3-tk`）

```bash
pip install pyyaml
```

### 任务分配

持续更新。

### ROS仿真

持续更新。



## 快速开始

```bash
git clone <你的仓库地址>
cd "<仓库名>/path-finding/one-shot/CBS with RRA"
python src/main.py maps/map_random/random_map.yaml output.yaml stay
```

命令参数：

| 参数 | 说明 |
| --- | --- |
| 第 1 个 | 输入地图文件路径 |
| 第 2 个 | 输出解文件路径 |
| 第 3 个 | `stay`：智能体到达终点后停留；`disappear`：到达后从地图中消失 |

运行其他算法时，进入对应目录并执行相同命令

```bash
cd "../ECBS with RRA"
python src/main.py maps/map_random/random_map.yaml output.yaml stay
```

求解成功后终端会输出总代价、生成和扩展的高层节点数、低层搜索次数以及运行时间。

### 在 IDE 中直接运行（PyCharm）

如果不想使用命令行，也可以直接在IDE中运行和调试。以PyCharm 为例：

1. 用PyCharm打开某个算法目录（例如`CBS with RRA/`）。
2. 打开`src/main.py`，直接点击右上角的运行按钮。此时会使用`main()`里定义的默认参数：输入`../maps/map_warehouse/warehouse_map.yaml`，输出`output.yaml`，终点行为`stay`。
3. 如需切换地图或算法参数，可以直接修改`main()`里的默认变量，或通过`Run → Edit Configurations`在`Parameters`中传入命令行参数（格式同上文的命令参数表）。
4. 在关键代码处打断点即可单步调试，逐步观察高层搜索与低层A*的执行流程，便于学习算法原理。

地图生成脚本（`map_create_*.py`）和可视化脚本（`visualisation.py`）同样支持直接运行和调试。

## 输入格式

输入文件为YAML，包含地图和智能体两部分：

```yaml
map:
  dimensions: [30, 30]          # 地图尺寸 [宽, 高]
  obstacles:
    - !!python/tuple [0, 4]     # 障碍物坐标 [x, y]
    - !!python/tuple [0, 5]
agents:
  - name: agent0
    start: [16, 21]             # 起点
    goal: [5, 8]                # 终点
  - name: agent1
    start: [28, 2]
    goal: [19, 14]
```

坐标范围：`0 <= x < dimensions[0]`，`0 <= y < dimensions[1]`。障碍物使用`!!python/tuple`存储，程序通过`yaml.FullLoader`读取。

`maps/`目录下提供了两个示例：

- `maps/map_random/random_map.yaml`：随机障碍物地图
- `maps/map_warehouse/warehouse_map.yaml`：仓库风格地图

## 输出格式

```yaml
schedule:
  agent0:
    - {t: 0, x: 16, y: 21}
    - {t: 1, x: 16, y: 20}
    - {t: 2, x: 17, y: 20}
  agent1:
    - {t: 0, x: 28, y: 2}
cost: 221
```

- `schedule`：每个智能体在每个时间步的位置，按 `t` 排序。
- `cost`：总代价，等于所有智能体路径长度（状态数）之和。

`solution found`表示求解成功；若输出`no solution`，则说明当前实例无解。

## 可视化

使用tkinter播放求解结果：

```bash
python src/visualisation.py
```

运行前修改`visualisation.py`顶部的`MAP_FILE`和 `SOLUTION_FILE`，分别指向地图和解文件。还可以调整：

- `STAY_AT_GOAL`：到达终点后停留或消失
- `TIME_INTERVAL`：自动播放的时间间隔（毫秒）
- `CELL_SIZE`：每个网格的像素大小

界面支持开始、暂停、前进一步、后退一步，以及按智能体编号高亮显示路径。

## 地图生成

提供了两个地图生成脚本：

- `src/map_create_random.py`：生成随机障碍物地图
- `src/map_create_warehouse.py`：生成仓库风格地图

修改文件顶部的参数（地图尺寸、障碍物比例、智能体数量、随机种子和 `OUTPUT_FILE`）后直接运行即可。

## 项目结构

```text
.
├── path-finding/
│   ├── one-shot/
│   │   ├── CBS with RRA/
│   │   │   ├── src/            # 源码
│   │   │   └── maps/           # 地图
│   │   ├── ECBS with RRA/
│   │   │   ├── src/            # 源码
│   │   │   └── maps/           # 地图
│   │   ├── ICBS with RRA/
│   │   │   ├── src/            # 源码
│   │   │   └── maps/           # 地图
│   │   └── PBS with RRA/
│   │       ├── src/            # 源码
│   │       └── maps/           # 地图
│   └── life-long/              # 规划中
├── task-assignment/            # 规划中
└── ros/                        # 规划中
```

每个算法目录中的核心文件：

| 文件 | 作用 |
| --- | --- |
| `main.py` | 高层搜索（冲突树 / 优先级树）与程序入口 |
| `astar.py` | 低层带约束 A* 与冲突检测 |
| `RRA.py` | 反向搜索，计算 RRA 启发值 |
| `RRA_continue.py` | 在低层搜索之间继续未完成的反向搜索，避免重复计算 |
| `mdd.py` | MDD（仅 ICBS 使用） |
| `priority.py` | 优先级 DAG（仅 PBS 使用） |
| `visualisation.py` | tkinter 可视化 |

## Roadmap

- [x] one-shot MAPF：CBS / ICBS / ECBS / PBS
- [ ] one-shot MAPF：EECBS / PIBT
- [ ] life-long MAPF：RHCR
- [ ] 任务分配：Google OR-Tools / ALNS / DRLH
- [ ] ROS 集成：Gazebo + TurtleBot3 Burger


## 许可证

本项目采用 MIT 许可证，详见 [LICENSE](LICENSE)。
