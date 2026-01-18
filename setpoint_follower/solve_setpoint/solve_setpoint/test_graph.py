import itertools
from collections import defaultdict
from task_manager import Task
from solvers.graph_search import build_graph, select_communication_inconsistent_tasks, decomposition_path_guesses

class TaskID(Task):
    id_iter = itertools.count()
    def __init__(self, *args):
        super().__init__(*args)
        self.ID = next(self.id_iter)

    def __repr__(self):
        return super().__repr__() + f"id: {self.ID}"

def main():
    tasks = [TaskID([3, 4], [1,3], [0,10]),
             TaskID([3, 4], [2,5], [0,10]),
             TaskID([3, 4], [1,3], [0,10]),
             TaskID([3, 4], [2,3], [0,10])]
    
    print(tasks[0])
    
    edges = [(1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 2), (3, 6)]

    graph = build_graph(edges, tasks)
    print(graph)

    tasks = select_communication_inconsistent_tasks(graph, tasks)
    print(tasks)
    paths = decomposition_path_guesses(tasks, graph, 3)
    print(paths[0].paths_dict.items())

if __name__ == "__main__":
    main()