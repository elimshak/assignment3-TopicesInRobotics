#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from collections import defaultdict


class Graph:
    def __init__(self):
        self.edges = defaultdict(list)
        self.weights = {}

    def add_edge(self, from_node, to_node, weight):
        # Note: assumes edges are bi-directional
        self.edges[from_node].append(to_node)
        self.edges[to_node].append(from_node)
        self.weights[(from_node, to_node)] = weight
        self.weights[(to_node, from_node)] = weight


def dijsktra(graph, initial, end):
    # shortest paths is a dict of nodes
    # whose value is a tuple of (previous node, weight)
    shortest_paths = {initial: (None, 0)}
    current_node = initial
    visited = set()

    while current_node != end:
        # add current node to set of visited nodes
        visited.add(current_node)
        # take all edges from current node to other nodes
        destinations = graph.edges[current_node]
        # take the weight of current node
        weight_to_current_node = shortest_paths[current_node][1]

        for next_node in destinations:
            # take the weight of the edge between current and neighboor node
            weight = graph.weights[(current_node, next_node)] + weight_to_current_node
            # if next node didn't recognized till now, we add it to shortest paths
            if next_node not in shortest_paths:
                shortest_paths[next_node] = (current_node, weight)
            # takes the min weight from shortest paths and check which weight is lowest
            else:
                current_shortest_weight = shortest_paths[next_node][1]
                if current_shortest_weight > weight:
                    # if so, update the weight to the current weight
                    shortest_paths[next_node] = (current_node, weight)

        # check if there're still nodes that not visited in this path
        next_destinations = {node: shortest_paths[node] for node in shortest_paths if node not in visited}
        if not next_destinations:
            return "Route Not Possible"
        # next node is the destination with the lowest weight
        current_node = min(next_destinations, key=lambda k: next_destinations[k][1])

    # Work back through destinations in shortest path
    path = []
    while current_node is not None:
        path.append(current_node)
        next_node = shortest_paths[current_node][0]
        current_node = next_node
    # Reverse path
    path = path[::-1]
    return path


graph = Graph()

# Nodes:
a = (1.92, 0.17)
b = (2.26, -1.78)
c = (4.4, -1.78)
d = (2.4, 1.5)
e = (5.5, 1.5)
f = (5.8, 3.25)
g = (7.5, 3.25)
h = (5.8, -0.55)
j = (8, - 0.75)


#   b  - a - d
#   \        \
#   c        |
#        h - e - f
#        |        |
#        j       g
#

edges = [
    ('a', 'b', 1),
    ('a', 'd', 1),
    ('b', 'c', 1),
    ('d', 'e', 1),
    ('e', 'f', 1),
    ('e', 'h', 1),
    ('h', 'j', 1),
    ('f', 'g', 1)
]

#building the edges in graph
for edge in edges:
    graph.add_edge(*edge)

waypoints = [[eval(i)] for i in dijsktra(graph, 'a', 'j')]

def goal_pose(pose):
	goal_pose = MoveBaseGoal()
	goal_pose.target_pose.header.frame_id = 'map'
	goal_pose.target_pose.pose.position.x = pose[0][0]    
	goal_pose.target_pose.pose.position.y = pose[0][1]    
	goal_pose.target_pose.pose.position.z = 0.0   
	goal_pose.target_pose.pose.orientation.x = 0.0    
	goal_pose.target_pose.pose.orientation.y = 0.0   
	goal_pose.target_pose.pose.orientation.z = 0.0   
	goal_pose.target_pose.pose.orientation.w = 1.0

	return goal_pose



if __name__ == '__main__':
    rospy.init_node('patrol')    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  
    client.wait_for_server()

    for pose in waypoints:
        goal = goal_pose(pose)
        client.send_goal(goal)
        client.wait_for_result()