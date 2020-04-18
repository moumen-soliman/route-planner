#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math #Needed for distance formula
import heapq #Needed for Priority Queue 

"""
Class used to encapsulate a graph node for use in
the Priority Queue
"""
class graph_node:
    def __init__(self,value,priority):
        self.value = value #this is the node number 0,1,2,3...etc.
        self.priority = priority #this will be computed via f = g + h
        
    #Python method redefined to compare priority, needed in A*
    def __lt__(self, other):
        return self.priority < other.priority
    
    #Define equality between nodes
    def __eq__(self, other):
        return self.value == other.value
    
    #Return a string version of graph_node, used for testing
    def __str__(self):
        return str(self.value)+ " "+str(self.priority)+" "+str(self.neigh)+" "+str(self.inter)

"""
Usual Euclidean distance formula to be used
in all computations, admissible here.
"""
def dist(x1,y1,x2,y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

"""    
Helper method to return a shortest path in list form
"""
def get_path(predecessor, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = predecessor[current]
    path.append(start)
    path.reverse()#To print from start -> goal
    return path
"""
A* Algorithm adapted from http://theory.stanford.edu/~amitp/GameProgramming/
"""
def shortest_path(M,start,goal):
    #Obtain dictionary of (x,y) coordinates M.intersections
    inter = M.intersections
    #Obtain list of lists of adjacent nodes M.roads
    roads = M.roads
    #Create frontier list to be explored
    frontier = []
    #Make frontier into a Priority Queue
    heapq.heapify(frontier) 
    #Encapsulate start as a graph node
    start_node = graph_node(start, 0)
    #Add the start node to the frontier priority queue
    heapq.heappush(frontier,start_node) 
    #Dictionary to track previous node
    origin = {}
    #Dictionary to track current cost
    curr_cost = {}
    #Intialize origin and curr_cost dictionaries
    origin[start] = None
    curr_cost[start] = 0
    #Encapsulate goal as a graph_node for comparison
    goal_node = graph_node(goal, 0)
    #Main body of the algorithm
    while len(frontier) > 0:
        #Retrieve the lowest priority node from frontier
        current_node = heapq.heappop(frontier)
        #If lowest priority and at goal then finish
        if current_node == goal_node:
            break
        #Examine all possible neighbors from current
        #Note: next here is a pure number, not a graph_node
        #avoid the overhead of the class graph_node where possible 
        for next in roads[current_node.value]:
            #***Get current_node (x,y) coordinates***
            current_coords = inter[current_node.value]
            x_curr = current_coords[0] 
            y_curr = current_coords[1] 
            #************************************
            #***Get next (x,y) coordinates***
            next_coords = inter[next]
            x_next = next_coords[0] 
            y_next = next_coords[1]
            #************************************
            #Compute cost from current intersection to next intersection
            #This is: current cost + straightline distance cost 
            new_cost = curr_cost[current_node.value] + dist(x_curr,y_curr,x_next,y_next)
            if next not in curr_cost.keys() or new_cost < curr_cost[next]:
                #Update curr_cost of next
                curr_cost[next] = new_cost
                #***Get goal (x,y) coordinates***
                goal_coords = inter[goal]
                x_goal = goal_coords[0] 
                y_goal = goal_coords[1] 
                #************************************
                #Priority is new_cost + straightline distance cost next to goal 
                priority = new_cost + dist(x_goal,y_goal, x_next,y_next)
                #Encapsulate next as graph_node
                next_node = graph_node(next, priority)
                #Push the next_node onto the Priority Queue
                heapq.heappush(frontier,next_node)
                #Update the origin of next
                origin[next] = current_node.value
    #Apply helper function to origin dictionary and return
    return get_path(origin, start, goal)
    
    
    
 
