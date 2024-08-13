import sys
import numpy as np
from copy import deepcopy
from math import pi
import rospy
from core.interfaces import ArmController
from core.interfaces import ObjectDetector
from core.utils import time_in_seconds
from lib.IK_position_null import IK
from lib.calculateFK import FK
import copy
import time


# # RED ROBOT CODES
# def black_center(T_b_w):
#     seed = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
#
#     target = np.array([
#         [1,0,0,0.562],
#         [0,-1,0,-1.159],
#         [0,0,-1,0.5],
#         [0,0,0, 1],
#     ])
#
#     target_b = T_b_w @ target
#
#     # Using pseudo-inverse
#     q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(target_b, seed, method='J_pseudo', alpha=.5)
#     print(message_pseudo)
#     print(q_pseudo)
#     arm.safe_move_to_position(q_pseudo)
#     return q_pseudo
#
# def red_center(T_b_w):
#     target_red = np.array([
#         [1,0,0,0.562],
#         [0,-1,0,-0.8],
#         [0,0,-1,0.5],
#         [0,0,0, 1],
#     ])
#     target_r = T_b_w @ target_red
#     seed  = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
#     # Using pseudo-inverse
#     q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(target_r, seed, method='J_pseudo', alpha=.5)
#     print(message_pseudo)
#     print(q_pseudo)
#     arm.safe_move_to_position(q_pseudo)
#     _, T0e, _ = fk.forward(q_pseudo)
#     print('\n Toe:', T0e)
#     return q_pseudo, target_r
#
# def place_red(offset, T_b_w, target_r, q_pseudo):
#
#     target_r[2,3] = offset
#     seed  = q_pseudo
#     # Using pseudo-inverse
#     q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(target_r, seed, method='J_pseudo', alpha=.5)
#     print(message_pseudo)
#     print(q_pseudo)
#     arm.safe_move_to_position(q_pseudo)
#     _, T0e, _ = fk.forward(q_pseudo)
#     print('\n Toe:', T0e)
#     arm.open_gripper()
#     arm.exec_gripper_cmd(0.5, 50)
#     return q_pseudo

# BLUE ROBOT CODES
def black_center_blue(T_b_w):

    seed = np.array([0,0,0,-pi/2,0,pi/2,pi/4])

    target = np.array([
        [1,0,0,0.562],
        [0,-1,0,1.159],
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

def blue_center_dynamic(T_b_w, seed):
    target_blue = np.array([
        [1,0,0,0.562],
        [0,-1,0,0.82],
        [0,0,-1,0.229],
        [0,0,0, 1],
    ])
    target_b = T_b_w @ target_blue
    # seed  = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(target_b, seed, method='J_pseudo', alpha=.5)
    print(message_pseudo)
    print(q_pseudo)
    arm.safe_move_to_position(q_pseudo)
    _, T0e, _ = fk.forward(q_pseudo)
    print('\n Toe:', T0e)
    return q_pseudo, target_b

def blue_center1(T_b_w): #blue_center1(T_b_w, seed)
    target_blue = np.array([
        [1,0,0,0.48],
        [0,-1,0,0.75],
        [0,0,-1,0.231],
        [0,0,0, 1],
    ])
    target_b = T_b_w @ target_blue
    seed  = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(target_b, seed, method='J_pseudo', alpha=.5)
    print(message_pseudo)
    print(q_pseudo)
    arm.safe_move_to_position(q_pseudo)
    _, T0e, _ = fk.forward(q_pseudo)
    print('\n Toe:', T0e)
    return q_pseudo, target_b

def blue_center2(T_b_w):
    target_blue = np.array([
        [1,0,0,0.48],
        [0,-1,0,0.89],
        [0,0,-1,0.231],
        [0,0,0, 1],
    ])
    target_b = T_b_w @ target_blue
    seed  = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(target_b, seed, method='J_pseudo', alpha=.5)
    print(message_pseudo)
    print(q_pseudo)
    arm.safe_move_to_position(q_pseudo)
    _, T0e, _ = fk.forward(q_pseudo)
    print('\n Toe:', T0e)
    return q_pseudo, target_b


def blue_center_real(T_b_w):
    target_blue = np.array([
        [1,0,0,0.61],
        [0,-1,0,0.8],
        [0,0,-1,0.5],
        [0,0,0, 1],
    ])
    target_b = T_b_w @ target_blue
    seed  = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(target_b, seed, method='J_pseudo', alpha=.5)
    print(message_pseudo)
    print(q_pseudo)
    arm.safe_move_to_position(q_pseudo)
    _, T0e, _ = fk.forward(q_pseudo)
    print('\n Toe:', T0e)
    return q_pseudo, target_b




def place_blue(offset, T_b_w, target_b, q_pseudo):

    target_b[2,3] = offset
    seed  = q_pseudo
    # Using pseudo-inverse
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(target_b, seed, method='J_pseudo', alpha=.5)
    print(message_pseudo)
    print(q_pseudo)
    arm.safe_move_to_position(q_pseudo)
    _, T0e, _ = fk.forward(q_pseudo)
    print('\n Toe:', T0e)
    arm.open_gripper()
    arm.exec_gripper_cmd(0.5)
    return q_pseudo

# GENERAL CODES

def pick_pose(T0e, detector, q_pseudo_orig):
    """
    Decides the pose and orientation of the end-effector to pick the block
    """
    print('Deciding the end-effector position to pick the block!')

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
    seed = q_pseudo_orig

    target_pose = H_ee_camera @ pose_block
    print('\nTarget pose : ',target_pose)
    a = copy.deepcopy(target_pose)

    x,y = np.argwhere(np.abs(a[:3,:3]) == np.max(np.abs(a[:3,:3])))[0]
    a = np.delete(a, x, axis=0)
    a = np.delete(a,y, axis=1)
    theta = np.abs(np.arccos(np.abs(a[0][0])))
    T = np.array([[np.cos(theta), -np.sin(theta), 0,0],[np.sin(theta), np.cos(theta),0,0],[0,0,1,0],[0,0,0,1]])
    T[:,3] = target_pose[:,3]
    T = T0e @ T

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
    T[2, 3] = 0.225
    seed = q_pseudo

    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(T, seed, method='J_pseudo', alpha=.5)
    print(message_pseudo)
    print(q_pseudo)
    _, T0e, _ = fk.forward(q_pseudo)
    print('\n Toe:', T0e)
    arm.safe_move_to_position(q_pseudo)
    return q_pseudo







# MAIN FUNCTION

if __name__ == "__main__":
    try:
        team = rospy.get_param("team")

    except KeyError:
        print('Team must be red or blue, make sure you are launching final.launch!')
        exit()

    rospy.init_node('team_script')

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


    if team == 'blue':

        T_b_w = np.array([[1,0,0,0],\
                        [0,1,0,-0.990],\
                        [0,0,1,0],\
                        [0,0,0,1]])
        start_t = time_in_seconds()
        print("** BLUE TEAM  **")
        offset = [0.227, 0.277, 0.327, 0.377]



        q_pseudo_bluec, target_b = blue_center_real(T_b_w)
        # target_blue = np.array([
        #     [1,0,0,0.532],
        #     [0,-1,0,0.8],
        #     [0,0,-1,0.6],
        #     [0,0,0, 1],
        # ])
        # target_b = T_b_w @ target_blue
        # seed  = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
        # q_pseudo_bluec, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(target_b, seed, method='J_pseudo', alpha=.5)
        # print(message_pseudo)
        # print(q_pseudo_bluec)
        # arm.safe_move_to_position(q_pseudo_bluec)
        # _, T0e, _ = fk.forward(q_pseudo_bluec)
        #
        # print('\n Toe:', T0e)
        #
        # print("CONTROL C KARLO")

        for i in range(2):
            print('DYNAMIC BLOCKS')

            print("Moving to stage 1")


            # STRATEGY 1 --------------------------------------------------------------------------------------------------------

            q_pseudo_dc_stage1 = np.array([ -2.6,  1.35 , 1.25 ,-0.72, -0.125 , 1.65,  -0.98])
            q_pseudo_dc_stage2 = np.array([ -2.6,  1.45 , 1.25 ,-0.72, -0.118 , 1.659,  -0.98])
            q_pseudo_dc_stage3 = np.array([ -2.25,  1.45 , 1.25 ,-0.72, -0.118 , 1.659,  -0.98])
            q_pseudo_dc_stage4 = np.array([ -2.25,  0.8 , 1.25 ,-0.72, -0.118 , 1.659,  -0.98])
            q_pseudo_bc = np.array([-0.16319302,  0.12039879, -0.17748208, -1.4767993,   0.02121327,  1.59531908, 0.44546112])
            q_pseudo_dynamic1 = np.array([-0.18141838,  0.03813762, -0.19593728, -2.29919704,  0.01029797,  2.33657912, 0.4010437])
            q_pseudo_dynamic2 = np.array([-0.13559486,  0.43315004, -0.17883098, -1.73939441,  0.09020194,  2.16456132, 0.43662292])

            arm.safe_move_to_position(q_pseudo_dc_stage1)
            arm.exec_gripper_cmd(0.5)

            arm.safe_move_to_position(q_pseudo_dc_stage2)

            arm.safe_move_to_position(q_pseudo_dc_stage3)

            time.sleep(5)
            arm.exec_gripper_cmd(0)

            arm.safe_move_to_position(q_pseudo_dc_stage4)

            arm.safe_move_to_position(q_pseudo_bc)

            if i == 0:
                q_pseudo_dynamic, target_b = blue_center1(T_b_w)
                print('Q position for dynamic one: ', q_pseudo_dynamic)
                arm.exec_gripper_cmd(0.5)


            else:
                q_pseudo_dynamic, target_b = blue_center2(T_b_w)
                print('Q position for dynamic two: ', q_pseudo_dynamic)


            arm.safe_move_to_position(q_pseudo_dynamic)
            arm.exec_gripper_cmd(0.5)
            arm.safe_move_to_position(q_pseudo_bc)


        for i in range(4):
            print('STATIC BLOCKS')
            if i==0:
                q_pseudo_bc = black_center_blue(T_b_w)



            else:
                arm.safe_move_to_position(q_pseudo_bc)

            _, T0e, _ = fk.forward(q_pseudo_bc)
            print('\n Toe:', T0e)
            arm.exec_gripper_cmd(0.5)
            q_pseudo = pick_pose(T0e, detector, q_pseudo_bc)
            arm.exec_gripper_cmd(0)
            arm.safe_move_to_position(q_pseudo_bc)
            if i==0:
                q_pseudo_bluec, target_b = blue_center_real(T_b_w)
            else:
                arm.safe_move_to_position(q_pseudo_bluec)
            # q_pseudo = place_blue(offset[i+2], T_b_w, target_b, q_pseudo_bluec)
            q_pseudo = place_blue(offset[i], T_b_w, target_b, q_pseudo_bluec)

            if i == 3:
                target_blue = np.array([
                    [1,0,0,0.532],
                    [0,-1,0,0.8],
                    [0,0,-1,0.6],
                    [0,0,0, 1],
                ])
                target_b = T_b_w @ target_blue
                seed  = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
                q_pseudo_bluec, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(target_b, seed, method='J_pseudo', alpha=.5)
                print(message_pseudo)
                print(q_pseudo_bluec)
                arm.safe_move_to_position(q_pseudo_bluec)
                _, T0e, _ = fk.forward(q_pseudo_bluec)

                print('\n Toe:', T0e)



            arm.safe_move_to_position(q_pseudo_bluec)
        end_t = time_in_seconds()
        duration = (end_t - start_t)/60
        print('Total time taken:', duration)
    offset = [0.427, 0.477]
    target_blue_old = np.array([
        [1,0,0,0.61],
        [0,-1,0,0.8],
        [0,0,-1,0.5],
        [0,0,0, 1],
    ])
    target_b_old = T_b_w @ target_blue_old
    seed  = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
    q_pseudo_old, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(target_b_old, seed, method='J_pseudo', alpha=.5)
    print(message_pseudo)
    print(q_pseudo_old)

    for i in range(2):
        arm.safe_move_to_position(q_pseudo_bluec)
        arm.exec_gripper_cmd(0.5)
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
            if len(list_poses)>1:
                for (name_block, pose_block) in list_poses:
                    pose_block_trans = target_b @ H_ee_camera @ pose_block
                    if np.abs((T_b_w.T @ pose_block_trans)[0,3] - 0.61) > 0.06: ## 0.06 is fudge factor considering IK
                        ## pick and place section of the code
                        arm.exec_gripper_cmd(0.5)
                        seed = q_pseudo_bluec
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
                        T = target_b @ T
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
                        #change
                        q_pseudo = place_blue(offset[i], T_b_w, target_b_old, q_pseudo_old)
                        arm.safe_move_to_position(q_pseudo_bluec)






# STRATEGY 2 --------------------------------------------------------------------------------------------------------

# q_pseudo_dc_stage1 = np.array([ -2.6,  1.35 , 1.25 ,-0.72, -0.125 , 1.65,  -0.98])
# q_pseudo_dc_stage2 = np.array([ -2.6,  1.532 , 1.25 ,-0.72, 0.25 , 1.65,  -0.98])
# q_pseudo_dc_stage3 = np.array([ -2.25,  1.532 , 1.25 ,-0.72, 0.25 , 1.65,  -0.98])
# q_pseudo_dc_stage4 = np.array([ -2.25,  1.3 , 1.25 ,-0.72, 0.25 , 1.65,  -0.98])
# q_pseudo_dc_stage5 = np.array([ -0.5,  1.3 , 1.25 ,-0.3, 0.25 , 1.65,  -0.98])
# q_pseudo_bc = np.array([-0.16319302,  0.12039879, -0.17748208, -1.4767993,   0.02121327,  1.59531908, 0.44546112])
# q_pseudo_dynamic1 = np.array([-0.18141838,  0.03813762, -0.19593728, -2.29919704,  0.01029797,  2.33657912, 0.4010437])
# q_pseudo_dynamic2 = np.array([-0.13559486,  0.43315004, -0.17883098, -1.73939441,  0.09020194,  2.16456132, 0.43662292])
#
#
# arm.safe_move_to_position(q_pseudo_dc_stage1)

# arm.exec_gripper_cmd(0.5)
#
# arm.safe_move_to_position(q_pseudo_dc_stage2)
#
# arm.safe_move_to_position(q_pseudo_dc_stage3)
#
# time.sleep(8)

# arm.exec_gripper_cmd(0)
#
# arm.safe_move_to_position(q_pseudo_dc_stage4)
#
# arm.safe_move_to_position(q_pseudo_bc)
#
# if i == 0:
#     q_pseudo_dynamic, target_b = blue_center1(T_b_w)
#     print('Q position for dynamic one: ', q_pseudo_dynamic)

#     arm.exec_gripper_cmd(0.5)
#
#
# else:
#     q_pseudo_dynamic, target_b = blue_center2(T_b_w)
#     print('Q position for dynamic two: ', q_pseudo_dynamic)
#
#
# arm.safe_move_to_position(q_pseudo_dynamic)

# arm.exec_gripper_cmd(0.5)
# arm.safe_move_to_position(q_pseudo_bc)
    #
    #
    # else:
    #     T_b_w = np.array([[1,0,0,0],\
    #                     [0,1,0,0.990],\
    #                     [0,0,1,0],\
    #                     [0,0,0,1]])
    #     start_t = time_in_seconds()
    #     print("**  RED TEAM  **")
    #     offset = [0.25, 0.275, 0.325, 0.35]
    #     for i in range(4):
    #         print('STATIC BLOCKS')
    #         if i==0:
    #             q_pseudo_bc = black_center(T_b_w)
    #         else:
    #             arm.safe_move_to_position(q_pseudo_bc)
    #         _, T0e, _ = fk.forward(q_pseudo_bc)
    #         print('\n Toe:', T0e)

    #         arm.exec_gripper_cmd(0.5)
    #         q_pseudo = pick_pose(T0e, detector, q_pseudo_bc)
            # arm.exec_gripper_cmd(0)
    #         arm.safe_move_to_position(q_pseudo_bc)
    #         if i==0:
    #             q_pseudo_rc, target_r = red_center(T_b_w)
    #         else:
    #             arm.safe_move_to_position(q_pseudo_rc)
    #         q_pseudo = place_red(offset[i], T_b_w, target_r, q_pseudo_rc)
    #         arm.safe_move_to_position(q_pseudo_rc)
    #     end_t = time_in_seconds()
    #     duration = (end_t - start_t)/60
    #     print('Total time taken:', duration)
    #

    # END STUDENT CODE
