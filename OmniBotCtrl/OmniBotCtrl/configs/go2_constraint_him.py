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

class Go2ConstraintHimRoughCfg( LeggedRobotCfg ):
    class env(LeggedRobotCfg.env):
        num_envs = 2048

        n_scan = 187
        n_priv_latent =  4 + 1 + 12 + 12 + 12 + 6 + 1 + 4 + 1 - 3 + 4
        n_proprio = 45
        history_len = 10
        num_observations = n_proprio + n_scan + history_len*n_proprio + n_priv_latent
        en_logger = False

    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.42] # x,y,z [m]
        """
          unitree go2 sdk order:
               -0.1 <-3 FR_hip_joint 0 -> 0.0
               0.8 <- 4 FR_thigh_joint 1 -> 0.9
               -1.5 <- 5 FR_calf_joint 2 -> -1.8
               0.1 <- 0 FL_hip_joint 3 -> 0.0
               0.8 <- 1 FL_thigh_joint 4 -> 0.9
               -1.5 <- 2 FL_calf_joint 5 -> -1.8
               -0.1 <- 9 RR_hip_joint 6 -> 0.0
               1 <- 10 RR_thigh_joint 7 -> 0.9
               -1.5 <- 11 RR_calf_joint 8 -> -1.8
               0.1 <- 6 RL_hip_joint 9 -> 0.0
               1 <- 7 RL_thigh_joint 10 -> 0.9
               -1.5 <- 8 RL_calf_joint 11 -> -1.8
        """
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.1,   # [rad]
            'RL_hip_joint': 0.1,   # [rad]
            'FR_hip_joint': -0.1 ,  # [rad]
            'RR_hip_joint': -0.1,   # [rad]

            'FL_thigh_joint': 0.8,     # [rad]
            'RL_thigh_joint': 1.,   # [rad]
            'FR_thigh_joint': 0.8,     # [rad]
            'RR_thigh_joint': 1.,   # [rad]

            'FL_calf_joint': -1.5,   # [rad]
            'RL_calf_joint': -1.5,    # [rad]
            'FR_calf_joint': -1.5,  # [rad]
            'RR_calf_joint': -1.5,    # [rad]
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'joint': 30.}  # [N*m/rad]
        damping = {'joint': 0.75}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4
        hip_scale_reduction = 1

        use_filter = True

    class commands( LeggedRobotCfg.control ):
        curriculum = False
        max_curriculum = 1.
        num_commands = 4  # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        resampling_time = 10.  # time before command are changed[s]
        heading_command = True  # if true: compute ang vel command from heading error
        global_reference = False

        class ranges:
            lin_vel_x = [-1.0, 1.0]  # min max [m/s]
            lin_vel_y = [-1.0, 1.0]  # min max [m/s]
            ang_vel_yaw = [-1, 1]  # min max [rad/s]
            heading = [-3.14, 3.14]

    class asset( LeggedRobotCfg.asset ):
        file = '{ROOT_DIR}/resources/go2/urdf/go2.urdf'
        foot_name = "foot"
        name = "go2"
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 0 # 1 to disable, 0 to enable...bitwise filter
        flip_visual_attachments = True
  
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.35      #m 机体高度
        clearance_height_target = -0.2 #m 摆动腿设定相对机体高度
        target_feet_height = 0.2 #m

        cycle_time=0.5 #s 
        touch_thr= 5 #N
        command_dead = 0.1
        target_joint_pos_scale = 0.17    # rad

        max_contact_force = 150 #N
        class scales( LeggedRobotCfg.rewards.scales ):
            # reference motion tracking
            foot_clearance= -0.01 #摆动
            default_joint_pos = 0.01 #关节锁定
            # gait
            feet_air_time = 0.0
            collision = 0.0
            feet_stumble = 0.0
            stand_still = 0.0
            foot_slip = -0.01
            # command
            termination = 0.0
            tracking_lin_vel = 1.2
            tracking_ang_vel = 0.7
            lin_vel_z = -2.0
            # energy
            dof_vel = 0
            dof_acc = -2.5e-7
            torques = 0
            powers = -2e-5
            action_rate = -0.01
            action_smoothness=-0.01
            # base
            orientation=0.15 #-0.2  姿态
            ang_vel_xy = -0.05 # 姿态速度
            base_height = 0.1 #-1.0 #高度
 

    class domain_rand( LeggedRobotCfg.domain_rand):
        randomize_friction = True
        friction_range = [0.2, 2.75]
        randomize_restitution = True
        restitution_range = [0.0,1.0]
        randomize_base_mass = True
        added_mass_range = [-1., 3.]
        randomize_base_com = True
        added_com_range = [-0.1, 0.1]
        push_robots = True
        push_interval_s = 15
        max_push_vel_xy = 1

        randomize_motor = True
        motor_strength_range = [0.8, 1.2]

        randomize_kpkd = True
        kp_range = [0.8,1.2]
        kd_range = [0.8,1.2]

        randomize_lag_timesteps = True
        lag_timesteps = 3
    
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
            acc_smoothness = 0.1
            collision = 0.1
            stand_still = 0.1
            hip_pos = 0.1
            base_height = 0.1

        class d_values:#约束的上界
            pos_limit = 0.0
            torque_limit = 0.0
            dof_vel_limits = 0.0
            feet_air_time = 0.1
            acc_smoothness = 0.0
            collision = 0.0
            stand_still = 0.0
            hip_pos = 0.0
            base_height = 0.0
 
    class cost:#约束函数数量
        num_costs = 9
    
    class terrain(LeggedRobotCfg.terrain):#采用地形
        mesh_type = 'plane'  # "heightfield" # none, plane, heightfield or trimesh
        measure_heights = True
        include_act_obs_pair_buf = False

class Go2ConstraintHimRoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
        learning_rate = 1.e-3
        max_grad_norm = 1
        num_learning_epochs = 5
        num_mini_batches = 4 # mini batch size = num_envs*nsteps / nminibatches
        cost_value_loss_coef = 0.1
        cost_viol_loss_coef = 1

    class policy( LeggedRobotCfgPPO.policy):
        init_noise_std = 1.0
        continue_from_last_std = True
        scan_encoder_dims = [128, 64, 32]
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
        num_costs = 9#---------------

        teacher_act = True
        imi_flag = False#---------------
      
    class runner( LeggedRobotCfgPPO.runner ):#重新载入预训练模型resume
        run_name = 'test_barlowtwins'
        experiment_name = 'rough_go2_constraint'
        policy_class_name = 'ActorCriticMixedBarlowTwins'
        runner_class_name = 'OnConstraintPolicyRunner'
        algorithm_class_name = 'NP3O'
        max_iterations = 6000
        num_steps_per_env = 24
        resume = False
        resume_path = '/home/pi/Downloads/LocomotionWithNP3O-master/logs/rough_go2_constraint/Jul11_13-11-11_test_barlowtwins/model_1100.pt'
 

  
