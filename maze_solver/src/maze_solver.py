#!/usr/bin/env python
from copy import deepcopy
from heapq import heappush, heappop
from itertools import count
import rospy
from sensor_msgs.msg import Range
import os
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from math import pi
from math import inf


class Node:
    '''Node data structure for a maze representation.'''

    def __init__(self, node_id, parent=None, child1=None, child2=None):
        '''Constructor for the node with the required parameters.'''
        self.node_id = node_id
        self.parent = parent
        self.children = [child1, child2]

    def get_children(self):
        children = []
        for child in self.children:
            if child:
                current_child = (deepcopy(child[0]), child[1], child[2])
                children.append(current_child)
        return children


def file2graph(filename):
    path = os.path.abspath(__file__)
    dir_path = os.path.dirname(path)  # full path of the directory of your script
    file_path = os.path.join(dir_path, filename)  # absolute zip file path

    with open(file_path) as file:
        next(file)
        f = file.read()
    f.strip()
    l1 = f.split('\n')
    l = [entry.split(' ') for entry in l1]
    for i in range(0, len(l)):
        l[i] = [float(entry) if entry != '-' else None for entry in l[i]]
    nodes = []
    for i in range(0, len(l)):
        nodes.append(Node(node_id=l[i][0], child1=None if l[i][1] == None else [l[i][1], l[i][2], l[i][3]],
                          child2=None if l[i][1] == None or l[i][4] == None else [l[i][4], l[i][5], l[i][6]]))
    for n in nodes:
        for child in n.children:
            i = 0
            if child == None:
                i += 1
                break
            child_id = int(child[0])
            child[0] = nodes[child_id - 1]
            nodes[child_id - 1].parent = n
            i += 1
    return nodes[0]


def solution(goal_node):
    l = []
    current_node = goal_node
    while current_node:
        l.append(current_node.node_id)
        current_node = current_node.parent
    l.reverse()
    return l


def ucs(root, goal):
    '''Unifrom cost graph search implementation.'''
    counter = count()
    frontier = [(0, next(counter), root)]
    # initially we haven't found any solution yet
    current_solution = None
    while frontier:
        acc_cost, _, node = heappop(frontier)
        # if a solution was found, and the current node cost is >= to the solution cost
        # then we are sure that no better solution can be found, so return the current solution
        if current_solution and (acc_cost >= current_solution[1]):
            # print("UCS took %d iterations"%(c))
            return solution(current_solution[0])
        # if the current node is a solution
        if node.node_id == goal:
            # we found a solution, skip the children of this node, and continue to check
            # if a better solution can be found
            current_solution = (node, acc_cost)
            continue

        children = node.get_children()
        for child in children:
            child[0].parent = node
            heappush(frontier, (child[2] + acc_cost, next(counter), child[0]))
    return solution(current_solution[0])


setpoint_pub = None
motor_pub = None
path = None
current_node = None
setpoint = 0
timer = -inf


def us_callback(msg):
    global setpoint
    global timer
    us_range = msg.range
    setpoint_msg = Vector3()
    if us_range > 0.6:
        if rospy.get_rostime().secs - timer > 5:
            setpoint_msg.x = setpoint
            setpoint_msg.y = 1
            setpoint_pub.publish(setpoint_msg)
        return None
    if rospy.get_rostime().secs - timer <= 5:
        return None
    setpoint = node2setpoint()
    setpoint_msg.x = setpoint
    setpoint_pub.publish(setpoint_msg)
    #print('moving to new node at ', setpoint)
    timer = rospy.get_rostime().secs
    #print('timer set at ', timer)


def node2setpoint():
    global current_node
    global setpoint
    new_node_id = path[path.index(current_node.node_id) + 1]
    for child in current_node.children:
        if child and child[0].node_id == new_node_id:
            setpoint += child[1] * pi / 180
            current_node = child[0]
            if setpoint > pi:
                setpoint %= -pi
            elif setpoint < -pi:
                setpoint %= pi
            return setpoint


if __name__ == '__main__':
    current_node = file2graph("maze.txt")
    path = ucs(current_node, 16)
    rospy.init_node('maze_solver', anonymous=True)
    rospy.Subscriber("sonar", Range, us_callback)
    current_node = current_node.children[0][0]
    setpoint_pub = rospy.Publisher('pid_setpoint', Vector3, queue_size=1000)
    motor_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1000)
    rospy.spin()
