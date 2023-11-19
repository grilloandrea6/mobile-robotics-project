import sys
import math

class Global_Navigation(object):
    def __init__(self, nodes):
        self.nodes = nodes
        self.graph = {}
        for node in nodes:
            self.graph[node] = {}

    def add_node(self,a):
        if not isinstance(a,tuple) or len(a) != 2:
            print("ERROR - wrong parameters to function add_node")
            exit(-1)
        self.graph[a] = {}


    def add_edge(self,a,b):
        if not isinstance(a,tuple) or not isinstance(b,tuple) or len(a) != 2 or len(b) != 2:
            print("ERROR - wrong parameters to function add_edge")
            exit(-1)

        my_dist = self.dist(a,b)
        self.graph[a][b] = my_dist
        self.graph[b][a] = my_dist
    
    def get_nodes(self):
        return self.nodes.copy()
    
    def dist(self, a, b):
        if not isinstance(a,tuple) or not isinstance(b,tuple) or len(a) != 2 or len(b) != 2:
            print("ERROR - wrong parameters to function value")
            exit(-1)
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)   
     
    def get_outgoing_edges(self, a):
        if not isinstance(a,tuple) or len(a) != 2:
            print("ERROR - wrong parameters to function add_node")
            exit(-1)

        connections = []
        for out_node in self.nodes:
            if self.graph[a].get(out_node, False) != False:
                connections.append(out_node)
        return connections

    def dijkstra(self, start_node):
        unvisited_nodes = self.get_nodes()

        # cost of getting to that node so far
        shortest_path = {}
    
        # previous node to get to that node in the optimal path, so far
        previous_nodes = {}
    
        # max_value to initialize "infinity" value of unvisited nodes   
        max_value = sys.maxsize
        for node in unvisited_nodes:
            shortest_path[node] = max_value
        # starting node value = 0   
        shortest_path[start_node] = 0
        
        # The algorithm executes until we visit all nodes
        while len(unvisited_nodes) != 0:
            # The code block below finds the node with the lowest score
            current_min_node = unvisited_nodes[0]
            for node in unvisited_nodes: # Iterate over the nodes
                if shortest_path[node] < shortest_path[current_min_node]:
                    current_min_node = node
                    
            # The code block below retrieves the current node's neighbors and updates their distances
            neighbors = self.get_outgoing_edges(current_min_node)
            for neighbor in neighbors:
                tentative_value = shortest_path[current_min_node] + self.dist(current_min_node, neighbor)
                if tentative_value < shortest_path[neighbor]:
                    shortest_path[neighbor] = tentative_value
                    previous_nodes[neighbor] = current_min_node
    
            # After visiting its neighbors, we mark the node as "visited"
            unvisited_nodes.remove(current_min_node)
        
        return previous_nodes, shortest_path
    
    def path_planning(self,start_node,target_node):
        previous_nodes, shortest_path = self.dijkstra(start_node)
        node = target_node
        path = []
        while node != start_node:
            path.append(node)
            node = previous_nodes[node]
        path.reverse()

        print("We found the following best path with a value of {}.".format(shortest_path[target_node]))
        print(str(start_node) + " -> " + " -> ".join(str(a) for a in path))

        return path
