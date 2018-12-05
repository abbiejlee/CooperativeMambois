import numpy as np 
from numpy import linalg as LA 


room_x_min = 0 #cm
room_x_max = 1200 #cm
room_y_min = 0 #cm
room_y_max = 1200 #cm
room_z_min = 0 #cm
room_z_max = 1200 #cm
room_up_limit = (room_x_max, room_y_max, room_z_max)
room_low_limit = (room_x_min, room_y_min, room_z_min)


safety_margin = 5 #cm 

step_dist = 3 #cm

def nextPos(init_pos, dirction_pos):
    #input: initial position (x, y, z) in cm
    #       direction position (x, y, z) in cm 
    #output: next position (x, y, z) in cm
    pt_diff = direction_pos - init_pos
    dist = LA.norm(pt_diff)
    if dist > 3:
        k = step_dist/dist
        next_pos = init_pos + k*(pt_diff)
        return next_pos
    else:
        return direction_pos

def ptInRoom(pt):
    #input: a random position in 3D space (x, y, z) in cm
    #output: a boolean (True if input position is inside of the room within the safety margin) 
    low_diff = np.absolute(np.subtract(pt, room_low_limit))
    up_diff = np.absolute(np.subtract(pt, room_up_limit))
    for coord_diff in (low_diff+up_diff):
        if coord_diff < safety_margin:
            return False
    return True

def randomPt():
    #output: a random position (x, y, z) in cm
    rand_x = np.random.random_integers(room_x_min,room_x_max)
    rand_y = np.random.random_integers(room_y_min,room_y_max)
    rand_z = np.random.random_integers(room_z_min,room_z_max)
    return (rand_x, rand_y, rand_z)

def path2Goal(init_pos, goal_pos):
    #input: 
    #output: an array of positions (x,y,z) 3cm(step_dist) apart from init_pos to goal_pos
    path = [init_pos]
    while LA.norm(goal_pos-path[-1]) > step_dist:
        next_pt = nextPos(path[-1], goal_pos)
        path.append(next_pt)
    path.append(goal_pos)
    return path

def ifGoal():
    #input:
    #output: goal_pos if goal is detected; None if goal is not found

def ptInGoal(pt):
    #output: True if pt is in goal region

def pathPlan(init_pos):
    #input: init_pos is the current position of the drone
    #output:
    while not ptInGoal(init_pos):
        goal_pos = ifGoal()
        if goal_pos == None:
            rand_goal = randomPt()
            if ptInRoom(rand_goal):
                return path2Goal(init_pos, rand_goal)
        else:
            return path2Goal(init_pos, goal_pos)