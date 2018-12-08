import numpy as np 
from numpy import linalg as LA 
import cv2
from ColorSegmentation import cd_color_segmentation

#describing the 3D space with cylinderical coordinates 
room_theta_min = 0 #degree
room_theta_max = 360 #degree
room_r_min = 0 #cm
room_r_max = 1200 #cm
room_z_min = 0 #cm
room_z_max = 1200 #cm
room_up_limit = (room_theta_max, room_r_max, room_z_max)
room_low_limit = (room_theta_min, room_r_min, room_z_min)

#the drone is not allowed to fly within safety_margin distance to the wall
safety_margin = 5 #cm 

#how far can the drone move at each step
step_dist = 3 #cm


def nextPos(init_pos, dirction_pos):
    #find the next position to move to at the step_distance
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
    #check if a cooridnate is in the room 
    #input: a random position in 3D space (theta, r, z) in deg/cm
    #output: a boolean (True if input position is inside of the room within the safety margin) 
    if (pt[0] >= room_theta_min) and (pt[0] <= room_theta_max):
        if (pt[1] >= room_r_min) and (pt[1] <= room_r_max):
            if (pt[2] >= room_z_min) and (pt[2] <= room_z_max):
                return True
    return False


def randomPt():
    #output: a random position (theta, r, z) in deg/cm
    rand_theta = np.random.random_integers(room_theta_min,room_theta_max)
    rand_r = np.random.random_integers(room_r_min,room_r_max)
    rand_z = np.random.random_integers(room_z_min,room_z_max)
    return (rand_theta, rand_r, rand_z)


def path2Goal(init_pos, goal_pos):
    #return an array of (x,y,z) positions from the drone's current location to the target location
    #input: inital position in (x, y, z) in cm and goal position in (theta, r, z) in deg/cm
    #output: an array of positions (x,y,z) 3cm(step_dist) apart from init_pos to goal_pos
    path = [init_pos]
    goal_pos_x = init_pos[0] + goal_pos[1]*np.cos(goal_pos[0])
    goal_pos_y = init_pos[1] + goal_pos[1]*np.sin(goal_pos[0])
    goal_pos_z = goal_pos[2]
    goal_pos_xyz = (goal_pos_x, goal_pos_y, goal_pos_z)
    while LA.norm(goal_pos_xyz-path[-1]) > step_dist:
        next_pt = nextPos(path[-1], goal_pos_xyz)
        path.append(next_pt)
    path.append(goal_pos_xyz)
    return path

def ifGoal():
    #input:
    #output: goal_pos if goal is detected; None if goal is not found
    [box_pt, ln_color, target_threshold] = cd_color_segmentation(frame)



#def ptInGoal(pt):
    #output: True if pt is in goal region
def targetDist(bounding_box_area):
    target_dist_in = 42595.62+2565277*np.exp(-0.07396415*bounding_box_area)
    target_dist_cm = target_dist_in*2.54
    return target_dist_cm

def pathPlan(init_pos, init_orien, d_theta = 10):
    #input: init_pos is the current (x,y,z) in cm position of the drone
    #       init_orien is the current (yaw, pitch, roll) in degree of the drone
    #       d_theta = how much the drone is rotating every step in degrees
    #output: [path1, path2]
    #        path1 is an array of (x,y,z) in cm; path2 is (theta, r, z) in deg/cm   
    [box_pt, ln_color, target_threshold, contour_area] = cd_color_segmentation(frame)
    if target_threshold == 0: # no target identified; yaw (change theta)
        if init_orien <= 360:
            path1 = [init_pos]
            path2 = [(init_orien[0]+d_theta, 0, init_pos[2])]
        else:
            path1 = [(init_pos[0], init_pos[1], init_pos[2]+step_dist)]
            path2 = [(init_orien[0], 0, init_pos[2]+step_dist)]
    elif target_threshold == 1: # identify possible target; fly closer
        goal_dist = targetDist(contour_area)
        goal_pos = (init_orien[0], goal_dist, init_pos[2])
        path1 = path2Goal(init_pos, goal_pos)
        path2 = [init_orien]
    elif target_threshold == 2: # being in the shooting range; stay where it is
        path1 = [init_pos]
        path2 = [init_orien]
    else: #if target_threshold == 3: # being too close to the target; back track
        path1 = [init_pos]
        path2 = [init_orien]
    return [path1, path2]

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)

    while(cap.isOpened()):
        # Capture frame-by-frame
        ret, frame = cap.read()

        pathPlan(init_pos, init_orien)

        cv2.rectangle(frame, box_pt[0], box_pt[1], ln_color,3)
        
        # Display the resulting frame
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()