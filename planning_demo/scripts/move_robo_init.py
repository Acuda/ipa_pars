#!/usr/bin/python

import rospy
from simple_script_server import *
sss = simple_script_server()

if __name__ == "__main__":
	rospy.init_node("move_to_init_position")

	#sss.move("torso","home")
        sss.move("arm_left","side")
        sss.move("arm_right","side")
        sss.move("base",[-3.6,-2.1,1.5]) 
        #sss.move("base",[-3.6,-2.1,1.5])
        gripper_preparation_left = [[-2.5,-1.36,1.35,0.06,0.75,0.7,0.57]]
        gripper_preparation_right = [[2.5,1.36,-1.35,-0.06,-0.75,-0.7,-0.57]]
        gripping_arm_left = [[-2.5,-1.56,1.35,0.06,1.25,0.7,0.57]]
        gripping_arm_right = [[2.5,1.56,-1.35,-0.06,-1.25,-0.7,-0.57]]
        gripping_gripper_left = [[0.3,0.3]]
        gripping_gripper_right = [[0.3,0.3]]
        gripper_right_open = [[0,0]]
        gripper_left_open = [[0,0]]
        carry_left = [[-2.8,-1.56,1.35,0.06,1.25,0.7,0.57]]
        carry_right = [[2.8,1.56,-1.35,-0.06,-1.25,-0.7,-0.57]]
        
#         sss.move("arm_left",gripper_preparation_left)
#         sss.move("arm_right",gripper_preparation_right)
#         sss.move("arm_right",gripping_arm_right)
#         sss.move("arm_left",gripping_arm_left)
#         sss.move("gripper_right",gripping_gripper_right)
#         sss.move("gripper_left",gripping_gripper_left)
#         sss.move("arm_right",carry_right)
#         sss.move("arm_left",carry_left)
#         sss.move("base",[-3.0,2.0,0])
#         sss.move("gripper_right",gripper_right_open)
#         sss.move("gripper_left",gripper_left_open)
#         sss.move("arm_left",gripping_gripper_left)
#         sss.move("arm_right",gripping_gripper_right)
#         sss.move("arm_left",gripper_preparation_left)
#         sss.move("arm_right",gripper_preparation_right)
#         sss.move("arm_left","side")
#         sss.move("arm_right","side")
#         sss.move("base",[0,0,0])
        #sss.move("gripper_right",gripping_gripper_right)
        #sss.move("gripper_left",gripping_gripper_left)
        #sss.move("arm_right","side")
        #sss.move("arm_left","side")
