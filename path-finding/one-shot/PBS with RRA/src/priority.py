
from collections import deque


class PriorityDAG:

    # add_priority的三种结果
    ADDED = "added"        # 新增了优先关系
    IMPLIED = "implied"    # 关系已被传递闭包蕴含，不需要加边
    CYCLE = "cycle"        # 新关系会成环

    def __init__(self, agents):
        # 直接高优先级邻居
        self.higher = {a: set() for a in agents}
        # 直接低优先级邻居
        self.lower = {a: set() for a in agents}

    # 获取所有高优先级祖先
    def get_ancestors(self, agent):
        visited = set()
        stack = list(self.higher[agent])

        while stack:
            cur = stack.pop()

            if cur not in visited:
                visited.add(cur)
                stack.extend(self.higher[cur])

        return visited

    # 获取所有低优先级后代
    def get_descendants(self, agent):
        visited = set()
        stack = list(self.lower[agent])

        while stack:
            cur = stack.pop()

            if cur not in visited:
                visited.add(cur)
                stack.extend(self.lower[cur])

        return visited

    # 添加关系：i的优先级低于j
    def add_priority(self, i, j):

        # 关系已经通过传递关系成立，不需要加边；调用方据此剪掉这条重复分支。
        if j in self.get_ancestors(i):
            return self.IMPLIED, {i} | self.get_descendants(i)

        # 如果i已经高于j，再添加j->i就会形成环
        if j in self.get_descendants(i):
            return self.CYCLE, set()

        # 添加边 j->i
        self.higher[i].add(j)
        self.lower[j].add(i)

        # 找出i及其所有低优先级后代
        affected = {i} | self.get_descendants(i)

        return self.ADDED, affected

    # 对受影响的智能体进行拓扑排序
    def topological_sort(self, nodes):

        # 计算每个节点的入度，即有多少个高优先级邻居在nodes中
        # 通过优先级比a高的邻居智能体和本次要排序的智能体取交集，得到a的高优先级邻居在nodes中的数量
        indegree = {
            a: len(self.higher[a] & nodes)
            for a in nodes
        }
        # 初始化队列用于BFS，初始队列元素为入度为0的节点，表示这些节点没有高优先级邻居在nodes中，可以直接排序
        queue = deque(
            a for a in nodes if indegree[a] == 0
        )

        order = []

        while queue:
            cur = queue.popleft()
            order.append(cur)

            # 遍历当前节点的所有在nodes中的低优先级邻居
            # 如果邻居的入度减1后为0，说明该邻居没有高优先级邻居在nodes中，可以直接排序
            for nxt in self.lower[cur] & nodes:
                indegree[nxt] -= 1

                if indegree[nxt] == 0:
                    queue.append(nxt)

        if len(order) != len(nodes):
            raise ValueError("Priority graph contains a cycle")

        return order
