import random
import sys

# Source: https://www.udacity.com/blog/2021/10/implementing-dijkstras-algorithm-in-python.html
## START
class Graph(object):
    def __init__(self, nodes, init_graph):
        self.nodes = nodes
        self.graph = self.construct_graph(nodes, init_graph)
        
    def construct_graph(self, nodes, init_graph):
        '''
        This method makes sure that the graph is symmetrical. In other words, if there's a path from node A to B with a value V, there needs to be a path from node B to node A with a value V.
        '''
        graph = {}
        for node in nodes:
            graph[node] = {}
        
        graph.update(init_graph)
        
        for node, edges in graph.items():
            for adjacent_node, value in edges.items():
                if graph[adjacent_node].get(node, False) == False:
                    graph[adjacent_node][node] = value
                    
        return graph
    
    def get_nodes(self):
        "Returns the nodes of the graph."
        return self.nodes
    
    def get_outgoing_edges(self, node):
        "Returns the neighbors of a node."
        connections = []
        for out_node in self.nodes:
            if self.graph[node].get(out_node, False) != False:
                connections.append(out_node)
        return connections
    
    def value(self, node1, node2):
        "Returns the value of an edge between two nodes."
        return self.graph[node1][node2]

def dijkstra_algorithm(graph, start_node):
    unvisited_nodes = list(graph.get_nodes())
 
    # We'll use this dict to save the cost of visiting each node and update it as we move along the graph   
    shortest_path = {}
 
    # We'll use this dict to save the shortest known path to a node found so far
    previous_nodes = {}
 
    # We'll use max_value to initialize the "infinity" value of the unvisited nodes   
    max_value = sys.maxsize
    for node in unvisited_nodes:
        shortest_path[node] = max_value
    # However, we initialize the starting node's value with 0   
    shortest_path[start_node] = 0
    
    # The algorithm executes until we visit all nodes
    while unvisited_nodes:
        # The code block below finds the node with the lowest score
        current_min_node = None
        for node in unvisited_nodes: # Iterate over the nodes
            if current_min_node == None:
                current_min_node = node
            elif shortest_path[node] < shortest_path[current_min_node]:
                current_min_node = node
                
        # The code block below retrieves the current node's neighbors and updates their distances
        neighbors = graph.get_outgoing_edges(current_min_node)
        for neighbor in neighbors:
            tentative_value = shortest_path[current_min_node] + graph.value(current_min_node, neighbor)
            if tentative_value < shortest_path[neighbor]:
                shortest_path[neighbor] = tentative_value
                # We also update the best path to the current node
                previous_nodes[neighbor] = current_min_node
 
        # After visiting its neighbors, we mark the node as "visited"
        unvisited_nodes.remove(current_min_node)
    
    return previous_nodes, shortest_path

def print_result(previous_nodes, shortest_path, start_node, target_node):
    path = []
    node = target_node
    
    while node != start_node:
        path.append(node)
        node = previous_nodes[node]
 
    # Add the start node manually
    path.append(start_node)
    
    print("We found the following best path with a value of {}.".format(shortest_path[target_node]))
    print(" -> ".join(reversed(path)))
## STOP

def place_obstacle(position):
    grid[position[0]][position[1]] = 8

def place_obstacles(num_obstacles=101, obstacles=[]):
    if(num_obstacles != 101):
        created_obstacles = []
        for i in range(10):
            created_obstacles.append([])

        for n in range(num_obstacles):
            # Generate obstacle position
            x,y = random.randint(0,9), random.randint(0,9)

            # Check the obstacle isn't at the top-left or bottom-right corner
            while((x==0 and y==0) or (x==9 and y==9)):
                x,y = random.randint(0,9), random.randint(0,9)

            # Check that it does not already exist among the created obstacles
            while y in created_obstacles[x]:
                y = random.randint(0,9)
            
            # Add new position to collection of obstacles
            created_obstacles[x].append(y)

            # Place obstacle in grid
            place_obstacle((x,y))
    else:
        for obstacle in obstacles:
            place_obstacle(obstacle)    

def display_grid():
    for row in grid:
        print(row)

# Create grid
grid = []

# Populate grid
for i in range(10):
    grid.append([0,0,0,0,0,0,0,0,0,0])

# Assign initial position
position = (0,0)

# Place defined obstacles
# place_obstacles(obstacles=[(9,7),(8,7),(6,7),(6,8)])

# Place for 20 obstacles
place_obstacles(num_obstacles=20)

# Print out grid
display_grid()

# Create adjacency matrix
## Create nodes (0 to 99)
nodes = [str(x) for x in range(100)]

init_matrix = {}

for node in nodes:
    init_matrix[node] ={}

## Create paths
current = 0
current_str = str(current)
x,y = 0,0
current_position = (x,y)
for row in grid:
    y = 0
    for column in grid:
        # Get cells all around
        left, right, up, down = (x,y-1), (x,y+1), (x-1,y), (x+1,y)
        dl_up, dr_up, dl_down, dr_down = (x-1,y-1), (x-1,y), (x+1,y-1), (x+1,y+1)

        neighbours = [(left, current-1), (right, current+1), (up, current-10), (down, current+10), (dl_up, current-11), (dr_up, current-9), (dl_down, current+9), (dr_down, current+11)]
        
        # If the cell contains a 0, add a 1 to the conjunction
        for neighbour in neighbours:
            neighbour_position = neighbour[0]
            neighbour_number = neighbour[1]

            if ((neighbour_position[0] < 0) or (neighbour_position[1] < 0) or (neighbour_position[0] > 9) or (neighbour_position[1] > 9)):
                continue
            else:
                if (grid[neighbour_position[0]][neighbour_position[1]] == 0):
                    init_matrix[current_str][str(neighbour_number)] = 1
        y += 1
        current += 1
    x += 1

graph = Graph(nodes, init_matrix)

# Run Djikstra's algorithm
previous_nodes, shortest_path = dijkstra_algorithm(graph=graph, start_node=0)

# Print result
print_result(previous_nodes, shortest_path, start_node='0', target_node='99')
