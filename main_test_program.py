import math
import numpy as np
import ur_dual_controller as urcx

rbtx = urcx.URDualController(lft_robot_ip='10.2.0.50', rgt_robot_ip='10.2.0.51', control_pc_ip='10.2.0.101')
# left randomization
current_lft_jnt_values = rbtx.lft_arm.get_jnt_values()
n_lft_jnt_values = (current_lft_jnt_values + (np.random.rand(6) - .5) * 1 / 12 * math.pi).tolist()
nn_lft_jnt_values = (n_lft_jnt_values + (np.random.rand(6) - .5) * 1 / 12 * math.pi).tolist()
nnn_lft_jnt_values = (nn_lft_jnt_values + (np.random.rand(6) - .5) * 1 / 12 * math.pi).tolist()
# right randomization
current_rgt_jnt_values = rbtx.rgt_arm.get_jnt_values()
n_rgt_jnt_values = (current_rgt_jnt_values + (np.random.rand(6) - .5) * 1 / 12 * math.pi).tolist()
nn_rgt_jnt_values = (n_rgt_jnt_values + (np.random.rand(6) - .5) * 1 / 12 * math.pi).tolist()
nnn_rgt_jnt_values = (nn_rgt_jnt_values + (np.random.rand(6) - .5) * 1 / 12 * math.pi).tolist()
rbtx.move_jntspace_path([current_lft_jnt_values+current_rgt_jnt_values,
                         n_lft_jnt_values+n_rgt_jnt_values,
                         nn_lft_jnt_values+nn_rgt_jnt_values,
                         nnn_lft_jnt_values+nnn_rgt_jnt_values], control_frequency=0.05)