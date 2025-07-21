#---------------------------------------------------------------
# https://www.geeksforgeeks.org/shortest-path-unweighted-graph/
#---------------------------------------------------------------


from collections import deque

def bfs(graph, S, par, dist):
    # Queue to store the nodes in the order they are visited
    q = deque()
    # Mark the distance of the source node as 0
    dist[S] = 0
    # Push the source node to the queue
    q.append(S)

    # Iterate until the queue is not empty
    while q:
        # Pop the node at the front of the queue
        node = q.popleft()

        # Explore all the neighbors of the current node
        for neighbor in graph[node]:
            # Check if the neighboring node is not visited
            if dist[neighbor] == float('inf'):
                # Mark the current node as the parent of the neighboring node
                par[neighbor] = node
                # Mark the distance of the neighboring node as the distance of the current node + 1
                dist[neighbor] = dist[node] + 1
                # Insert the neighboring node to the queue
                q.append(neighbor)


def get_shortest_distance(graph, S, D, V):
    # par[] array stores the parent of nodes
    par = [-1] * V

    # dist[] array stores the distance of nodes from S
    dist = [float('inf')] * V

    # Function call to find the distance of all nodes and their parent nodes
    bfs(graph, S, par, dist)

    if dist[D] == float('inf'):
        print("Source and Destination are not connected")
        return

    # List path stores the shortest path
    path = []
    current_node = D
    path.append(D)
    while par[current_node] != -1:
        path.append(par[current_node])
        current_node = par[current_node]

    # Printing path from source to destination
    #for i in range(len(path) - 1, -1, -1):
    #    print(path[i], end=" ")

    return path[::-1]

if __name__ == "__main__":
    V = 8

    S, D = 2, 1

    edges = [[0, 1], [0, 5], [1, 4], [1, 5], [2, 3], [3, 6], [3, 7], [4, 5], [5, 7], [6, 7]]

    graph = [[] for _ in range(V)]

    for edge in edges:
        graph[edge[0]].append(edge[1])
        graph[edge[1]].append(edge[0])

    print(get_shortest_distance(graph, S, D, V))