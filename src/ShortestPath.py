from PriorityQueue import PriorityQueue
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def heuristic(a, b):
    (x1, y1, z1) = a
    (x2, y2, z2) = b
    return abs(x1 - x2) + abs(y1 - y2) + abs(z1 - z2)

def a_star_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        #display_pq(frontier)
        current = frontier.get()
        if current == goal:
            break
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
    return came_from, cost_so_far

def display_pq(pq):
    x = []
    y = []
    z = []
    for e in pq.elements:
        e = e[1]
        x.append(e[0])
        y.append(e[1])
        z.append(e[2])
    fig = plt.figure()
    sp = fig.add_subplot(111, projection='3d')
    sp.scatter(x, y, z)
    plt.show()

def reconstruct_path(came_from, start, goal):
    current = goal
    path_list = []
    while current != start:
        path_list.append(current)
        current = came_from[current]
    path_list.append(start) 
    path_list.reverse() 
    path = []
    for point in path_list:
        path.append(point)
    return path

def shortest_distance(graph, start, goal):
    came_from, cost_so_far = a_star_search(graph, start, goal)
    return cost_so_far[goal]

def get_shortest_path(graph, start, goal):
    came_from, cost_so_far = a_star_search(graph, start, goal)
    return reconstruct_path(came_from, start, goal)

