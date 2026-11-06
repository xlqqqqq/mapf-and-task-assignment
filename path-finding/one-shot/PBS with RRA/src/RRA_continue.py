"""继续扩展 RRA 中尚未完成的反向搜索。"""

from RRA import GridEnvironment, StateTwoDimension


class ASTAR_two_dimension_two:

    def __init__(self, goal_state, obstacles, dimensions):
        self.goal_state = goal_state
        self.environment = GridEnvironment(obstacles, dimensions)

    def search(self, distance, open_set, closed_set):
        while open_set:
            current = min(open_set, key=lambda state: distance[state])
            open_set.remove(current)
            closed_set.add(current)

            for point in self.environment.get_neighbors(current):
                neighbor = StateTwoDimension(point)
                if neighbor in closed_set:
                    continue

                new_distance = distance[current] + 1
                old_distance = distance.get(neighbor, float("inf"))

                if neighbor not in open_set:
                    open_set.add(neighbor)
                elif new_distance >= old_distance:
                    continue

                distance[neighbor] = new_distance

            if current == self.goal_state:
                return distance, open_set, closed_set

        return distance, open_set, closed_set

