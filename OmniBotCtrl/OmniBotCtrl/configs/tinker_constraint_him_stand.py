# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from configs.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO
from global_config import MAX_ITER,SAVE_DIV
class TinkerConstraintHimRoughCfg( LeggedRobotCfg ):
    class env(LeggedRobotCfg.env):
        num_envs = 1024  #1024 

        n_scan = 187
        n_priv_latent =  4 + 1 + 12 + 12 + 12 + 6 + 1 + 4 + 1 - 3 + 4 -10
        n_proprio = 39 #原始观测
        history_len = 10
        num_observations = n_proprio  + n_priv_latent  + n_scan + history_len*n_proprio
        amao=1
        num_actions = 10
        en_logger = False #wanda

    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.33] # x,y,z [m]

        default_joint_angles = { # = target angles [rad] when action = 0.0
            'J_L0':   0.0,   # [rad]
            'J_L1':  0.12,   # [rad]
            'J_L2':  0.56,   # [rad]
            'J_L3':  -1.12,   # [rad]
            'J_L4_ankle': -0.57,   # [rad]

            'J_R0':   0.0,   # [rad]
            'J_R1':  -0.12,   # [rad]
            'J_R2':  -0.56,   # [rad]
            'J_R3':  1.12,   # [rad]
            'J_R4_ankle': 0.57,   # [rad] 
        }

        default_joint_angles_st = { # = target angles [rad] when action = 0.0
            'J_L0':   0.0,   # [rad]
            'J_L1':  0.12,   # [rad]
            'J_L2':  0.56,   # [rad]
            'J_L3':  -1.12,   # [rad]
            'J_L4_ankle': -0.57,   # [rad]

            'J_R0':   0.0,   # [rad]
            'J_R1':  -0.12,   # [rad]
            'J_R2':  -0.56,   # [rad]
            'J_R3':  1.12,   # [rad]
            'J_R4_ankle': 0.57,   # [rad] 
        }
    # stiffness = {'leg_roll': 200.0, 'leg_pitch': 350.0, 'leg_yaw': 200.0,
    #                 'knee': 350.0, 'ankle': 15}
    # damping = {'leg_roll': 10, 'leg_pitch': 10, 'leg_yaw':
    #             10, 'knee': 10, 'ankle': 10}  关节名字可以区分，mujoco下与其一致
    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        # stiffness = {'joint': 10.0}  # [N*m/rad]
        # damping = {'joint': 0.4}     # [N*m*s/rad]
        stiffness = {'J_L0': 13, 'J_L1': 15,'J_L2': 15, 'J_L3':15, 'J_L4_ankle':13,
                     'J_R0': 13, 'J_R1': 15,'J_R2': 15, 'J_R3':15, 'J_R4_ankle':13}
        damping = {'J_L0': 0.3, 'J_L1': 0.65,'J_L2': 0.65, 'J_L3':0.65, 'J_L4_ankle':0.3,
                   'J_R0': 0.3, 'J_R1': 0.65,'J_R2': 0.65, 'J_R3':0.65, 'J_R4_ankle':0.3}
        
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4
        hip_scale_reduction = 1

        use_filter = True

    class lession():
        stop = False #1000

    class commands( LeggedRobotCfg.control ):
        curriculum = False
        max_curriculum = 1.
        num_commands = 5  # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        resampling_time = 12  # time before command are changed[s]W
        heading_command = True  # if true: compute ang vel command from heading error
        global_reference = False

        class ranges:
            lin_vel_x = [-0.2, 0.2]  # min max [m/s]
            lin_vel_y = [-0.2, 0.2]  # min max [m/s]
            ang_vel_yaw = [-1.0, 1.0]  # min max [rad/s]
            heading = [-3.14, 3.14]
            height = [0.12 , 0.2] # m

    class asset( LeggedRobotCfg.asset ):
        #file = '{ROOT_DIR}/resources/tinker/urdf/tinker_urdf.urdf'
        #file = '{ROOT_DIR}/resources/tinker/urdf/tinker_urdf_inv_4.urdf'
        file = '{ROOT_DIR}/resources/TinkerV2_URDF/urdf/TinkerV2_URDF.urdf'
        #file = '{ROOT_DIR}/resources/tinker/urdf/tinker_urdf_inv_2_wbx(1).urdf'
    
        foot_name = "ankle" #URDF需要具有foot的link
        name = "tinker"
        penalize_contacts_on = ["ankle"]
        terminate_after_contacts_on = ["base_link"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter 自己碰撞会抽搐,同时足底力计算也不准确
        flip_visual_attachments = False #
  
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.29
        clearance_height_target = -0.20#相对站立高度 不能和高度差距太大
        tracking_sigma = 0.5 #0.25 小了探索出来爬行 300  0.15 小惯量  0.25 大惯量

        cycle_time=0.4 #s 
        touch_thr= 6 #N
        command_dead = 0.08
        stop_rate = 0.25
        target_joint_pos_scale = 0.17    # rad
        
        max_contact_force = 170 #N
        class scales( LeggedRobotCfg.rewards.scales ):
            termination = -5.0
            tracking_lin_vel = 2.5
            tracking_ang_vel = 2.0
            base_acc = 0.01
            lin_vel_z = -2.0
            ang_vel_xy = -0.02
            base_height = 0.25
            
            collision = 0.0
            feet_stumble = 0.0
            # action_rate = -0.01
            # action_smoothness=-0.01
            # energy
            powers = -2e-5
            action_smoothness = -0.01
            torques = -1e-5
            dof_vel = -5e-4
            dof_acc = -2e-7

            #stand_still = -0.05
            stand_still_force = -0.1
            stand_2leg = 3

            feet_air_time = 4
            foot_clearance= -5.5
            stumble= -0.015
            
            no_jump = 0.7
            orientation_eular=1.0 # 0.05可以探索爬行
 
            hip_pos= -1
            #feet_rotation = 1e-1
            feet_rotation1 = 1e-1
            feet_rotation2 = 1e-1
            #ankle_pos = 1e-5
            
            feet_contact_forces = -0.01
            #vel_mismatch_exp = 0.3  # lin_z; ang x,y  速度奖励大可以鼓励机器人更多移动，与摆腿耦合
            low_speed = 0.2 
            track_vel_hard = 0.5
            foot_slip = -0.05


    class domain_rand( LeggedRobotCfg.domain_rand):
        randomize_friction = True 
        friction_range = [0.1, 2.75]
        randomize_restitution = True
        restitution_range = [0.0,1.0]
        randomize_base_mass = True
        added_mass_range = [-0.5, 0.5]
        randomize_base_com = True
        added_com_range = [-0.05, 0.05]
        push_robots = True
        # push_interval_s = 15
        # max_push_vel_xy = 0.3
        push_interval_s = 6
        max_push_vel_xy = 0.7
        max_push_ang_vel = 0.6
        # dynamic randomization
        # action_delay = 0.5
        action_noise = 0.015

        randomize_motor = True
        motor_strength_range = [0.8, 1.2]#比例系数

        randomize_kpkd = True
        kp_range = [0.8,1.2]#比例系数
        kd_range = [0.8,1.2]

        randomize_lag_timesteps = True
        lag_timesteps = 10
    
        #old mass randomize new------------------------------
        randomize_all_mass = True
        rd_mass_range = [0.9, 1.1]

        randomize_com = True #link com
        rd_com_range = [-0.02, 0.02]
    
        random_inertia = True
        inertia_range = [0.9, 1.1]

        randomize_motor_offset = True
        motor_offset_range = [-0.045, 0.045] # Offset to add to the motor angles
        
        # add_lag = True #action lag
        # randomize_lag_timesteps = True
        # randomize_lag_timesteps_perstep = False
        # lag_timesteps_range = [5, 40]
        
        add_dof_lag = False
        randomize_dof_lag_timesteps = True
        randomize_dof_lag_timesteps_perstep = False
        dof_lag_timesteps_range = [0, 1]
        
        add_dof_pos_vel_lag = False #影响性能
        randomize_dof_pos_lag_timesteps = False
        randomize_dof_pos_lag_timesteps_perstep = False
        dof_pos_lag_timesteps_range = [0, 1]
        randomize_dof_vel_lag_timesteps = False
        randomize_dof_vel_lag_timesteps_perstep = False
        dof_vel_lag_timesteps_range = [0, 1]
        
        add_imu_lag =  True
        randomize_imu_lag_timesteps = True
        randomize_imu_lag_timesteps_perstep = False
        imu_lag_timesteps_range = [0, 1]
        

    class depth( LeggedRobotCfg.depth):
        use_camera = False
        camera_num_envs = 192
        camera_terrain_num_rows = 10
        camera_terrain_num_cols = 20

        position = [0.27, 0, 0.03]  # front camera
        angle = [-5, 5]  # positive pitch down

        update_interval = 1  # 5 works without retraining, 8 worse

        original = (106, 60)
        resized = (87, 58)
        horizontal_fov = 87
        buffer_len = 2
        
        near_clip = 0
        far_clip = 2
        dis_noise = 0.0
        
        scale = 1
        invert = True
    
    class costs:
        class scales:
            pos_limit = 0.1
            torque_limit = 0.1
            dof_vel_limits = 0.1
            feet_air_time = 0.1
            #acc_smoothness = 0.1
            #collision = 0.1
            #stand_still = 0.1 #站立默认位置
            hip_pos = 0.1 #侧展
            #base_height = 0.1
            #foot_regular = 0.1
            #trot_contact = 0.1

        class d_values:
            pos_limit = 0.0
            torque_limit = 0.0
            dof_vel_limits = 0.0
            feet_air_time = 0.1
            #acc_smoothness = 0.0
            #collision = 0.0
            #stand_still = 0.0
            hip_pos = 0.0
            #base_height = 0.0
            #foot_regular = 0.0
            #trot_contact = 1
 
    class cost:
        num_costs = 5 #需要同步修改 policy
    
    class terrain(LeggedRobotCfg.terrain):
        mesh_type = 'plane'  # "heightfield" # none, plane, heightfield or trimesh
        measure_heights = True
        include_act_obs_pair_buf = False

class TinkerConstraintHimRoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        # entropy_coef = 0.01
        # learning_rate = 1.e-3
        # max_grad_norm = 1
        # num_learning_epochs = 5
        # num_mini_batches = 4 # mini batch size = num_envs*nsteps / nminibatches
        # cost_value_loss_coef = 0.1
        # cost_viol_loss_coef = 1

        # entropy_coef = 0.001
        # learning_rate = 1e-5
        # num_learning_epochs = 2
        # gamma = 0.994
        # lam = 0.9
        # num_mini_batches = 4
    
        value_loss_coef = 1.0
        use_clipped_value_loss = True
        clip_param = 0.2
        entropy_coef = 0.001
        num_learning_epochs = 5
        num_mini_batches = 4    # minibatch size = num_envs*nsteps/nminibatches
        learning_rate = 1.e-4
        schedule = 'adaptive'   # could be adaptive, fixed
        gamma = 0.98
        lam = 0.95
        desired_kl = 0.01
        max_grad_norm = 1.
        weight_decay = 0

    class policy( LeggedRobotCfgPPO.policy):
        init_noise_std = 1.0
        continue_from_last_std = True
        scan_encoder_dims = None#[128, 64, 32]
        actor_hidden_dims = [512, 256, 128]
        critic_hidden_dims = [512, 256, 128]
        #priv_encoder_dims = [64, 20]
        priv_encoder_dims = []
        activation = 'elu' # can be elu, relu, selu, crelu, lrelu, tanh, sigmoid
        # only for 'ActorCriticRecurrent':
        rnn_type = 'lstm'
        rnn_hidden_size = 512
        rnn_num_layers = 1

        tanh_encoder_output = False
        num_costs = 5 #需要同步修改--------------------------------------cost

        teacher_act = False
        imi_flag = False
      
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = 'test_barlowtwins'
        experiment_name = 'rough_go2_constraint'
        policy_class_name = 'ActorCriticMixedBarlowTwins'
        runner_class_name = 'OnConstraintPolicyRunner'
        algorithm_class_name = 'NP3O'
        max_iterations = MAX_ITER #最大训练回合
        save_interval = SAVE_DIV #保存周期
        num_steps_per_env = 24
        resume = False
        resume_path =  '/home/pi/Downloads/back_good/LocomotionWithNP3O-masteroldxgoo2/LocomotionWithNP3O-masteroldx/logs/rough_go2_constraint/Dec17_12-41-23_test_barlowtwins/model_20000.pt'
 

  