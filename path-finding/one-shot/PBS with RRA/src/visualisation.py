"""读取地图和 CBS 路径解，并使用 tkinter 播放路径。"""

import os
import tkinter as tk
from tkinter import messagebox
from tkinter import ttk

import yaml


# ====================== 直接运行时在这里修改参数 ======================

# __file__ 表示当前 visualisation.py 文件的位置。
# 使用绝对路径后，在 PyCharm 中直接点击运行按钮也可以正确找到 YAML 文件。
PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
MAP_FILE = os.path.join(PROJECT_DIR, "../maps/map_warehouse/warehouse_map.yaml")
SOLUTION_FILE = os.path.join(PROJECT_DIR, "output.yaml")

# True：智能体到达终点后留在原地；False：路径结束后从地图中消失。
STAY_AT_GOAL = True

# 自动播放时，每隔多少毫秒前进一个时间步。
TIME_INTERVAL = 500

# 每个地图网格的像素大小。地图超过窗口大小时可以使用滚动条查看。
CELL_SIZE = 20
MAX_CANVAS_WIDTH = 1000
MAX_CANVAS_HEIGHT = 700

# 所有智能体使用相同颜色，通过圆形中的编号进行区分。
AGENT_COLOR = "#2878B5"
OBSTACLE_COLOR = "#333333"
GOAL_COLOR = "#D9534F"


def load_yaml(file_path):
    """按照主程序的方式读取 YAML 文件。"""

    with open(file_path, "r", encoding="utf-8") as yaml_file:
        # 地图中的障碍物可能使用 !!python/tuple，因此使用 FullLoader。
        return yaml.load(yaml_file, Loader=yaml.FullLoader)


class Visualisation:
    """管理地图绘制、时间步以及四个控制按钮。"""

    def __init__(self, root, map_data, solution_data):
        self.root = root
        self.root.title("路径可视化")

        self.map_data = map_data
        self.solution_data = solution_data

        dimensions = self.map_data["map"]["dimensions"]
        self.map_width = dimensions[0]
        self.map_height = dimensions[1]
        self.obstacles = self.map_data["map"]["obstacles"]
        self.schedule = self.solution_data["schedule"]

        self.current_time = 0
        self.max_time = 0
        self.is_playing = False
        self.after_id = None

        self.agent_names = self.get_agent_names()
        self.agent_numbers = {
            agent_name: number
            for number, agent_name in enumerate(self.agent_names)
        }
        self.paths = self.create_path_dict()
        self.goals = self.create_goal_dict()

        self.create_widgets()
        self.draw_static_map()
        self.draw_agents()

        # 点击窗口右上角关闭按钮时，先停止定时器，再关闭窗口。
        self.root.protocol("WM_DELETE_WINDOW", self.close_window)

    def get_agent_names(self):
        """按照地图 YAML 中的顺序为智能体分配编号。"""

        agent_names = []

        for agent in self.map_data["agents"]:
            agent_name = agent["name"]
            if agent_name in self.schedule:
                agent_names.append(agent_name)

        # 如果路径解中存在地图列表没有包含的名称，也将其追加到末尾。
        for agent_name in self.schedule:
            if agent_name not in agent_names:
                agent_names.append(agent_name)

        return agent_names

    def create_path_dict(self):
        """把 YAML 中的路径转换成便于按照时间查询的字典。"""

        paths = {}

        for agent_name in self.agent_names:
            paths[agent_name] = {}

            for state in self.schedule[agent_name]:
                time = state["t"]
                position = (state["x"], state["y"])
                paths[agent_name][time] = position

                if time > self.max_time:
                    self.max_time = time

        return paths

    def create_goal_dict(self):
        """保存路径解中每个智能体对应的终点。"""

        goals = {}
        for agent in self.map_data["agents"]:
            if agent["name"] in self.agent_numbers:
                goals[agent["name"]] = tuple(agent["goal"])
        return goals

    def create_widgets(self):
        """创建地图画布、滚动条、编号说明和控制按钮。"""

        content_frame = ttk.Frame(self.root, padding=8)
        content_frame.pack(fill=tk.BOTH, expand=True)

        map_pixel_width = self.map_width * CELL_SIZE
        map_pixel_height = self.map_height * CELL_SIZE

        canvas_frame = ttk.Frame(content_frame)
        canvas_frame.grid(row=0, column=0, sticky="nsew")

        self.canvas = tk.Canvas(
            canvas_frame,
            width=min(map_pixel_width, MAX_CANVAS_WIDTH),
            height=min(map_pixel_height, MAX_CANVAS_HEIGHT),
            background="white",
            scrollregion=(0, 0, map_pixel_width, map_pixel_height),
        )
        self.canvas.grid(row=0, column=0, sticky="nsew")

        horizontal_scrollbar = ttk.Scrollbar(
            canvas_frame,
            orient=tk.HORIZONTAL,
            command=self.canvas.xview,
        )
        horizontal_scrollbar.grid(row=1, column=0, sticky="ew")

        vertical_scrollbar = ttk.Scrollbar(
            canvas_frame,
            orient=tk.VERTICAL,
            command=self.canvas.yview,
        )
        vertical_scrollbar.grid(row=0, column=1, sticky="ns")

        self.canvas.configure(
            xscrollcommand=horizontal_scrollbar.set,
            yscrollcommand=vertical_scrollbar.set,
        )

        # 右侧列表说明“编号 -> 智能体名称”的对应关系。
        agent_frame = ttk.LabelFrame(content_frame, text="智能体编号", padding=5)
        agent_frame.grid(row=0, column=1, padx=(8, 0), sticky="ns")

        self.agent_list = tk.Listbox(agent_frame, width=18)
        self.agent_list.pack(side=tk.LEFT, fill=tk.Y)

        agent_scrollbar = ttk.Scrollbar(
            agent_frame,
            orient=tk.VERTICAL,
            command=self.agent_list.yview,
        )
        agent_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.agent_list.configure(yscrollcommand=agent_scrollbar.set)

        for agent_name in self.agent_names:
            agent_number = self.agent_numbers[agent_name]
            self.agent_list.insert(tk.END, str(agent_number) + " : " + agent_name)

        control_frame = ttk.Frame(content_frame, padding=(0, 8, 0, 0))
        control_frame.grid(row=1, column=0, columnspan=2)

        self.start_button = ttk.Button(
            control_frame,
            text="开始",
            width=10,
            command=self.start,
        )
        self.start_button.grid(row=0, column=0, padx=4)

        self.pause_button = ttk.Button(
            control_frame,
            text="暂停",
            width=10,
            command=self.pause,
        )
        self.pause_button.grid(row=0, column=1, padx=4)

        self.forward_button = ttk.Button(
            control_frame,
            text="前进一步",
            width=10,
            command=self.forward_one_step,
        )
        self.forward_button.grid(row=0, column=2, padx=4)

        self.backward_button = ttk.Button(
            control_frame,
            text="后退一步",
            width=10,
            command=self.backward_one_step,
        )
        self.backward_button.grid(row=0, column=3, padx=4)

        self.time_label = ttk.Label(control_frame)
        self.time_label.grid(row=0, column=4, padx=(15, 4))

        content_frame.rowconfigure(0, weight=1)
        content_frame.columnconfigure(0, weight=1)
        canvas_frame.rowconfigure(0, weight=1)
        canvas_frame.columnconfigure(0, weight=1)

    def get_cell_rectangle(self, x, y):
        """把地图坐标转换成 Canvas 中一个网格的矩形坐标。"""

        left = x * CELL_SIZE

        # 地图使用左下角为原点，Canvas 使用左上角为原点，因此需要翻转 y。
        top = (self.map_height - y - 1) * CELL_SIZE
        right = left + CELL_SIZE
        bottom = top + CELL_SIZE

        return left, top, right, bottom

    def draw_static_map(self):
        """绘制网格、障碍物以及每个智能体的终点。"""

        for x in range(self.map_width):
            for y in range(self.map_height):
                left, top, right, bottom = self.get_cell_rectangle(x, y)
                self.canvas.create_rectangle(
                    left,
                    top,
                    right,
                    bottom,
                    outline="#D8D8D8",
                    fill="white",
                )

        for obstacle in self.obstacles:
            left, top, right, bottom = self.get_cell_rectangle(
                obstacle[0],
                obstacle[1],
            )
            self.canvas.create_rectangle(
                left,
                top,
                right,
                bottom,
                outline=OBSTACLE_COLOR,
                fill=OBSTACLE_COLOR,
            )

        # 终点使用红色边框和 G编号 表示，例如 G0 表示智能体 0 的终点。
        for agent_name, goal in self.goals.items():
            left, top, right, bottom = self.get_cell_rectangle(goal[0], goal[1])
            self.canvas.create_rectangle(
                left + 3,
                top + 3,
                right - 3,
                bottom - 3,
                outline=GOAL_COLOR,
                width=2,
            )
            self.canvas.create_text(
                (left + right) / 2,
                top + 8,
                text="G" + str(self.agent_numbers[agent_name]),
                fill=GOAL_COLOR,
                font=("Arial", max(7, CELL_SIZE // 5)),
            )

    def get_agent_position(self, agent_name):
        """取得智能体在当前时间的位置。"""

        path = self.paths[agent_name]

        if self.current_time in path:
            return path[self.current_time]

        if not path:
            return None

        last_time = max(path)
        if self.current_time > last_time and STAY_AT_GOAL:
            return path[last_time]

        # 终点消失模式下，超过路径时间后不再绘制该智能体。
        return None

    def draw_agents(self):
        """根据当前时间重新绘制所有智能体。"""

        # agent 是 Canvas 标签。删除该标签只会删除动态智能体，不影响地图。
        self.canvas.delete("agent")

        for agent_name in self.agent_names:
            position = self.get_agent_position(agent_name)
            if position is None:
                continue

            left, top, right, bottom = self.get_cell_rectangle(
                position[0],
                position[1],
            )

            margin = CELL_SIZE * 0.16
            self.canvas.create_oval(
                left + margin,
                top + margin,
                right - margin,
                bottom - margin,
                fill=AGENT_COLOR,
                outline="white",
                width=2,
                tags="agent",
            )
            self.canvas.create_text(
                (left + right) / 2,
                (top + bottom) / 2,
                text=str(self.agent_numbers[agent_name]),
                fill="white",
                font=("Arial", max(8, CELL_SIZE // 3), "bold"),
                tags="agent",
            )

        self.time_label.configure(
            text="时间步：" + str(self.current_time) + " / " + str(self.max_time)
        )

    def start(self):
        """开始或者继续自动播放。"""

        if self.is_playing:
            return

        # 已经播放到末尾时，再次点击开始会从 t=0 重新播放。
        if self.current_time >= self.max_time:
            self.current_time = 0
            self.draw_agents()

        self.is_playing = True
        self.after_id = self.root.after(TIME_INTERVAL, self.play_next_step)

    def pause(self):
        """暂停自动播放。"""

        self.is_playing = False

        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
            self.after_id = None

    def play_next_step(self):
        """自动播放时由 tkinter 定时调用。"""

        self.after_id = None

        if not self.is_playing:
            return

        if self.current_time < self.max_time:
            self.current_time += 1
            self.draw_agents()

        if self.current_time >= self.max_time:
            self.is_playing = False
            return

        self.after_id = self.root.after(TIME_INTERVAL, self.play_next_step)

    def forward_one_step(self):
        """暂停播放，并前进一个时间步。"""

        self.pause()
        if self.current_time < self.max_time:
            self.current_time += 1
            self.draw_agents()

    def backward_one_step(self):
        """暂停播放，并后退一个时间步。"""

        self.pause()
        if self.current_time > 0:
            self.current_time -= 1
            self.draw_agents()

    def close_window(self):
        """停止定时器并关闭窗口。"""

        self.pause()
        self.root.destroy()


def main():
    """读取文件并打开可视化窗口。"""

    root = tk.Tk()

    try:
        map_data = load_yaml(MAP_FILE)
        solution_data = load_yaml(SOLUTION_FILE)
        Visualisation(root, map_data, solution_data)
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
        # messagebox 会弹出错误窗口，比在终端中显示错误更适合直接点击运行。
        root.withdraw()
        messagebox.showerror("读取失败", str(error))
        root.destroy()
        return 1

    root.mainloop()
    return 0


if __name__ == "__main__":
    main()
