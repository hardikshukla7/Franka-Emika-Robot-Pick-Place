import sys
import numpy as np
from copy import deepcopy
from math import pi

import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds
from lib.IK_position_null import IK
from lib.calculateFK import FK
import copy
import time

def pick_pose(T_b_w, T0e, detector, q_pseudo_orig):
    H_ee_camera = detector.get_H_ee_camera()
    start_time = int(time.time())
    list_poses = []
    while True:
        duration = int(time.time()) - start_time
        if duration == 10:
            return q_pseudo_orig
        list_poses = detector.get_detections()
        if len(list_poses)>0:
            break
    (name_block, pose_block) = list_poses[0]
    print(name_block,'\n',pose_block)


    (name_block, pose_block) = list_poses[0]
    print(name_block,'\n',pose_block)
    seed = q_pseudo_orig
    # pose[3,2] = pose[3,2]+0.00025
    target_pose = H_ee_camera @ pose_block
    print('\nTarget pose : ',target_pose)
    a = copy.deepcopy(target_pose)
    # x,y = np.argwhere((a==1)|(a==-1))[0]
    x,y = np.argwhere(np.abs(a[:3,:3]) == np.max(np.abs(a[:3,:3])))[0]
    a = np.delete(a, x, axis=0)
    a = np.delete(a,y, axis=1)
    theta = np.abs(np.arccos(np.abs(a[0][0])))
    T = np.array([[np.cos(theta), -np.sin(theta), 0,0],[np.sin(theta), np.cos(theta),0,0],[0,0,1,0],[0,0,0,1]])
    T[:,3] = target_pose[:,3]
    T = T0e @ T
    # target_pose = T0e @ target_pose ##definitely does not work to brink to base coord and then find yaw
    # T[:,3] = target_pose[:,3]
    T_step1 = copy.deepcopy(T)
    T_step1[2,3] = 0.5
    print('\nTarget pose step1 : ',T_step1)
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(T_step1, seed, method='J_pseudo', alpha=.5)
    print(message_pseudo)
    print(q_pseudo)
    _, T0e, _ = fk.forward(q_pseudo)
    print('\n Toe:', T0e)
    arm.safe_move_to_position(q_pseudo)
    print('\nTarget pose step1 : ',T)
    seed = q_pseudo
    ## check in real robot and remove or change these offsets
    # T[1,3] = T[1,3] - 0.02
    T[2,3] = 0.225
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(T, seed, method='J_pseudo', alpha=.5)
    print(message_pseudo)
    print(q_pseudo)
    _, T0e, _ = fk.forward(q_pseudo)
    print('\n Toe:', T0e)
    arm.safe_move_to_position(q_pseudo)
    return q_pseudo
    ## check in real robot and remove or change these offsets
    # T[1,3] = T[1,3] - 0.02


def black_center(T_b_w):
    seed = np.array([0,0,0,-pi/2,0,pi/2,pi/4])

    target = np.array([
        [1,0,0,0.562],
        [0,-1,0,-1.159],
        [0,0,-1,0.5],
        [0,0,0, 1],
    ])

    target_b = T_b_w @ target

    # Using pseudo-inverse
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(target_b, seed, method='J_pseudo', alpha=.5)
    print(message_pseudo)
    print(q_pseudo)
    arm.safe_move_to_position(q_pseudo)
    return q_pseudo

def red_center(T_b_w):
    target_red = np.array([
        [1,0,0,0.612],
        [0,-1,0,-0.81],
        [0,0,-1,0.5],
        [0,0,0, 1],
    ])
    target_r = T_b_w @ target_red
    seed  = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
    # Using pseudo-inverse
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(target_r, seed, method='J_pseudo', alpha=.5)
    print(message_pseudo)
    print(q_pseudo)
    arm.safe_move_to_position(q_pseudo)
    _, T0e, _ = fk.forward(q_pseudo)
    print('\n Toe:', T0e)
    return q_pseudo, target_r

def place_red(offset, T_b_w, target_r, q_pseudo):

    target_r[2,3] = offset
    seed  = q_pseudo
    # Using pseudo-inverse
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(target_r, seed, method='J_pseudo', alpha=.5)
    print(message_pseudo)
    print(q_pseudo)
    arm.safe_move_to_position(q_pseudo)
    _, T0e, _ = fk.forward(q_pseudo)
    print('\n Toe:', T0e)
    arm.exec_gripper_cmd(0.5)

    return q_pseudo

if __name__ == "__main__":
    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")
    arm = ArmController()
    detector = ObjectDetector()

    start_position = np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866])
    arm.safe_move_to_position(start_position) # on your mark!

    print("\n****************")
    if team == 'blue':
        print("** BLUE TEAM  **")
    else:
        print("**  RED TEAM  **")


    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!

    # STUDENT CODE HERE
    ik = IK()
    fk = FK()


    T_b_w = np.array([[1,0,0,0],\
                    [0,1,0,0.990],\
                    [0,0,1,0],\
                    [0,0,0,1]])
    if team == 'blue':
        print("** BLUE TEAM  **")
    else:
        print("**  RED TEAM  **")
#---------------------
        q_pseudo_bc = black_center(T_b_w)
        _, T_bc, _ = fk.forward(q_pseudo_bc)
        q_pseudo_rc, target_r = red_center(T_b_w)

        ## dynamic waiting strategy
        for i in range(2):

            arm.exec_gripper_cmd(0.5)
            q_pseudo_dc_stage1 = np.array([ 0.94324187,  0.31310008 , 0.29354613 ,-1.22159909, -0.08934577 , 1.52230772,  2.00432444]) #-----------correct_new
            arm.safe_move_to_position(q_pseudo_dc_stage1)#-----------------correct_new

            q_pseudo_dc_stage1 = np.array([ 0.55863543 , 1.33343726 , 0.81158586 ,-0.72196063,  0.67091963  ,1.08982746, -1.06556549]) #----------correct_new


            arm.safe_move_to_position(q_pseudo_dc_stage1) ## old outside table position ----correct_new


            q_pseudo_dc = [ 0.65556672 , 1.10832589 , 0.74500119 ,-1.11663743 , 0.53012313 , 1.71518041,-1.16691771] #------------correct_new
            arm.safe_move_to_position(q_pseudo_dc) #----------------- correct_new

            ## ------------------i think this will work on real robot though :) correct new 2 lines after this
            q_pseudo_dc = [ 0.85556672 , 1.10832589 , 0.74500119 ,-1.11663743 , 0.53012313 , 1.71518041,-1.16691771] #------------correct_new old1 -- it hits the blocks and hence force detected on turntable
            arm.safe_move_to_position(q_pseudo_dc) #----------------- correct_new old1

            time.sleep(5) ## i think this should be atleast 15-20 seconds in sim #----------------------correct_new
            arm.exec_gripper_cmd(0)
            arm.safe_move_to_position(q_pseudo_dc_stage1)
            arm.safe_move_to_position(q_pseudo_rc)#---------------------------------------------correct_new


            if i==0:
                target_red = np.array([
                    [1,0,0,0.452],
                    [0,-1,0,-0.9],
                    [0,0,-1,0.5],
                    [0,0,0, 1],
                ])
            if i==1:
                target_red = np.array([
                    [1,0,0,0.452],
                    [0,-1,0,-0.71],
                    [0,0,-1,0.5],
                    [0,0,0, 1],
                ])
            target_r_new = T_b_w  @ target_red

            target_r_new[2,3] = 0.231

            seed  = q_pseudo_rc

            q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(target_r_new, seed, method='J_pseudo', alpha=.5)

            arm.safe_move_to_position(q_pseudo)
            _, T0e, _ = fk.forward(q_pseudo)
            print('\n Toe:', T0e)
            arm.exec_gripper_cmd(0.5)
            arm.safe_move_to_position(q_pseudo_rc)

        ## static block code
        offset = [0.227, 0.277,0.327, 0.377, 0.427, 0.477]#[0.227, 0.277, 0.327, 0.377]
        for i in range(4):

            arm.safe_move_to_position(q_pseudo_bc)

            _, T0e, _ = fk.forward(q_pseudo_bc)
            print('\n Toe:', T0e)
            arm.exec_gripper_cmd(0.5)
            q_pseudo = pick_pose(T_b_w, T0e, detector, q_pseudo_bc)
            arm.exec_gripper_cmd(0)

            arm.safe_move_to_position(q_pseudo_bc)

            arm.safe_move_to_position(q_pseudo_rc)
            q_pseudo = place_red(offset[i], T_b_w, target_r, q_pseudo_rc)
            if i != 3:
                arm.safe_move_to_position(q_pseudo_rc)
            if i == 3:
                target_red = np.array([
                    [1,0,0,0.532],
                    [0,-1,0,-0.81],
                    [0,0,-1,0.6],
                    [0,0,0, 1],
                ])
                target_r = T_b_w @ target_red
                seed  = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
                # Using pseudo-inverse
                q_pseudo_rc, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(target_r, seed, method='J_pseudo', alpha=.5)
                print(message_pseudo)
                print(q_pseudo)
                arm.safe_move_to_position(q_pseudo_rc)
                _, T0e, _ = fk.forward(q_pseudo_rc)
                print('\n Toe:', T0e)
        target_red_old = np.array([
            [1,0,0,0.612],
            [0,-1,0,-0.81],
            [0,0,-1,0.5],
            [0,0,0, 1],
        ])
        target_r_old = T_b_w @ target_red_old
        seed  = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
        # Using pseudo-inverse
        q_pseudo_old, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(target_r_old, seed, method='J_pseudo', alpha=.5)
        print(message_pseudo)
        print(q_pseudo_old)

        ## stack up the dynamic blocks on the table - on top of static
        for i in range(2):
            H_ee_camera = detector.get_H_ee_camera()
            start_time = int(time.time())
            list_poses = []
            while True:
                duration = int(time.time()) - start_time
                if duration == 10:
                    break
                list_poses = detector.get_detections()
                if len(list_poses)>0:
                    break
            print('number of blocks detected:', len(list_poses))
            if len(list_poses)>1:
                for (name_block, pose_block) in list_poses:
                    pose_block_trans = target_r @ H_ee_camera @ pose_block
                    if np.abs((T_b_w.T@pose_block_trans)[0,3] - 0.612) > 0.06: ## 0.06 is fudge factor considering IK
                        ## pick and place section of the code
                        arm.exec_gripper_cmd(0.5)
                        seed = q_pseudo_rc
                        # pose[3,2] = pose[3,2]+0.00025
                        target_pose = H_ee_camera @ pose_block
                        print('\nTarget pose : ',target_pose)
                        a = copy.deepcopy(target_pose)
                        # x,y = np.argwhere((a==1)|(a==-1))[0]
                        x,y = np.argwhere(np.abs(a[:3,:3]) == np.max(np.abs(a[:3,:3])))[0]
                        a = np.delete(a, x, axis=0)
                        a = np.delete(a,y, axis=1)
                        theta = np.abs(np.arccos(np.abs(a[0][0])))
                        T = np.array([[np.cos(theta), -np.sin(theta), 0,0],[np.sin(theta), np.cos(theta),0,0],[0,0,1,0],[0,0,0,1]])
                        T[:,3] = target_pose[:,3]
                        T = target_r @ T
                        # target_pose = T0e @ target_pose ##definitely does not work to brink to base coord and then find yaw
                        # T[:,3] = target_pose[:,3]
                        T_step1 = copy.deepcopy(T)
                        T_step1[2,3] = 0.5
                        print('\nTarget pose step1 : ',T_step1)
                        q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(T_step1, seed, method='J_pseudo', alpha=.5)
                        print(message_pseudo)
                        print(q_pseudo)
                        _, T0e, _ = fk.forward(q_pseudo)
                        print('\n Toe:', T0e)
                        arm.safe_move_to_position(q_pseudo)
                        print('\nTarget pose step1 : ',T)
                        seed = q_pseudo
                        ## check in real robot and remove or change these offsets
                        # T[1,3] = T[1,3] - 0.02
                        T[2,3] = 0.225
                        q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(T, seed, method='J_pseudo', alpha=.5)
                        print(message_pseudo)
                        print(q_pseudo)
                        _, T0e, _ = fk.forward(q_pseudo)
                        print('\n Toe:', T0e)
                        arm.safe_move_to_position(q_pseudo)
                        arm.exec_gripper_cmd(0)
                        #change
                        q_inter = ([0, 0, 0 , -pi/2 , 0 , pi/2 , pi/4])
                        arm.safe_move_to_position(q_inter)
                        #change end
                        q_pseudo = place_red(offset[4+i], T_b_w, target_r_old, q_pseudo_old)
                        arm.safe_move_to_position(q_pseudo_rc)

        ## while True to keep trying dynamic block infinitely for remaining time - vision strategy
        while(True):
            arm.exec_gripper_cmd(0.5)
            q_pseudo_dc_stage1 = np.array([ 0.94324187,  0.31310008 , 0.29354613 ,-1.22159909, -0.08934577 , 1.52230772,  2.00432444]) #-----------correct_new
            arm.safe_move_to_position(q_pseudo_dc_stage1)#-----------------correct_new
            q_pseudo_dc_stage1 = np.array([ 0.55863543 , 1.33343726 , 0.81158586 ,-0.72196063,  0.67091963  ,1.08982746, -1.06556549]) #----------correct_new
            arm.safe_move_to_position(q_pseudo_dc_stage1) ## old outside table position ----correct_new
            q_pseudo_outside = np.array([ 0.55863543 , 1.33343726 , 0.81158586 ,-0.72196063,  0.67091963  ,1.08982746, -1.06556549]) #---------this is for strategy 2
            q_pseudo_outside[-2] = q_pseudo_outside[-2] + 0.785
            arm.safe_move_to_position(q_pseudo_outside) ## old outside table position ----strategy 2
            _, T_45b, _ = fk.forward(q_pseudo_outside)
            H_ee_camera = detector.get_H_ee_camera()
            start_time = int(time.time())
            list_poses = []
            while True:
                duration = int(time.time()) - start_time
                if duration == 10:
                    break
                list_poses = detector.get_detections()
                if len(list_poses)>0:
                    break
            print('number of blocks detected:', len(list_poses))
            tracking_block_name = ''
            tracking_block_pose = ''
            min_x = 0
            min_y = - 0.205
            if len(list_poses) > 0:
              for (name_block, pose_block) in list_poses:
                  if pose_block[0,3] < min_x and pose_block[1,3] < min_y:
                      pose_block_trans = np.linalg.inv(T_b_w) @ T_45b @ H_ee_camera @ pose_block ## converting to world frame because the symmetry is best exploted in world frame coordinates
                      target = copy.deepcopy(np.linalg.inv(T_b_w) @ T_45b)
                      ## exploiting symmetry of the table, we want to grasp the block at the same x, y position near the quadrant near the end effector
                      ## so same x but negative y (another stratrgy is to use radius, but not doing radius for now)
                      target[0,3] = pose_block_trans[0,3]
                      target[1,3] = -pose_block_trans[1,3]

                      target = T_b_w @ target

                      tracking_block_name = name_block
                      tracking_block_pose = pose_block
                      seed  = q_pseudo_outside
                      # Using pseudo-inverse
                      q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(target, seed, method='J_pseudo', alpha=.5)
                      print(message_pseudo)
                      print(q_pseudo)
                      if success_pseudo:
                          found = True
                          min_x = pose_block[0,3]
                          min_y = pose_block[1,3]
                          arm.safe_move_to_position(q_pseudo)
                          time.sleep(20) ## i think this should be atleast 15-20 seconds in sim #----------------------correct_new
                          arm.exec_gripper_cmd(0)
                          arm.safe_move_to_position(q_pseudo_outside)
                          arm.safe_move_to_position(q_pseudo_rc)
                          target_r_new = T_b_w  @ target_red
                          target_red = np.array([
                                  [1,0,0,0.312],
                                  [0,-1,0,-0.96],
                                  [0,0,-1,0.235],
                                  [0,0,0, 1],])
                          target_r_new = T_b_w  @ target_red
                          target_r_new[2,3] = 0.227
                          seed  = q_pseudo_rc
                          # Using pseudo-inverse
                          q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(target_r_new, seed, method='J_pseudo', alpha=.5)
                          q_pseudo[-2] = q_pseudo[-2]- 0.436 #30 degrees pitch rotation
                          print(message_pseudo)
                          print(q_pseudo)
                          arm.safe_move_to_position(q_pseudo)
                          _, T0e, _ = fk.forward(q_pseudo)
                          print('\n Toe:', T0e)
                          arm.exec_gripper_cmd(0.5)
                          arm.safe_move_to_position(q_pseudo_rc)
                          break





    # END STUDENT CODE
