import sys
sys.path.append('../')
from global_navigation import Global_Navigation

nodes = [(1,2),(3,4),(5,6),(10,11)]

global_nav = Global_Navigation(nodes)

global_nav.add_edge((1,2),(3,4))
global_nav.add_edge((1,2),(5,6))
global_nav.add_edge((3,4),(10,11))
global_nav.add_edge((10,11),(5,6))


start_node = (1,2)
target_node= (10,11)


path = global_nav.path_planning(start_node,target_node)

print(path)