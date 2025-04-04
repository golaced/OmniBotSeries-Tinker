# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

import math
import numpy as np
import mujoco, mujoco_viewer
from tqdm import tqdm
from collections import deque
from scipy.spatial.transform import Rotation as R
from global_config import ROOT_DIR
from configs.tinymal_constraint_him_stand import TinymalConstraintHimRoughCfg
import lcm
import numpy as np
from sim2sim_lcm.lcm_types.my_lcm import Request, Response
import time
import threading
lcm = lcm.LCM()

default_dof_pos=[-0.16,0.68,1.3 ,0.16,0.68,1.3, -0.16,0.68,1.3, 0.16,0.68,1.3]#默认角度需要与isacc一致
action_rl=default_dof_pos
def my_handler(channel, data):
    global action_rl
    msg = Response.decode(data)
    #print("q_exp_rl:",msg.q_exp)
    action_rl=msg.q_exp

class cmd:
    vx = 0.0
    vy = 0.0
    dyaw = 0.0

def quaternion_to_euler_array(quat):
    # Ensure quaternion is in the correct format [x, y, z, w]
    x, y, z, w = quat
    
    # Roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)
    
    # Pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch_y = np.arcsin(t2)
    
    # Yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)
    
    # Returns roll, pitch, yaw in a NumPy array in radians
    return np.array([roll_x, pitch_y, yaw_z])

def get_obs(data):
    '''Extracts an observation from the mujoco data structure
    '''
    q = data.qpos.astype(np.double)
    dq = data.qvel.astype(np.double)
    quat = data.sensor('orientation').data[[1, 2, 3, 0]].astype(np.double)
    r = R.from_quat(quat)
    v = r.apply(data.qvel[:3], inverse=True).astype(np.double)  # In the base frame
    omega = data.sensor('angular-velocity').data.astype(np.double)
    gvec = r.apply(np.array([0., 0., -1.]), inverse=True).astype(np.double)
    
    # 将 q, dq, quat, r, v, omega, gvec 按照 State_Rl.cpp需要的方式组装起来
    
    
    return (q, dq, quat, v, omega, gvec)

def pd_control(target_q, q, kp, target_dq, dq, kd):
    '''Calculates torques from position commands
    '''
    return (target_q - q) * kp + (target_dq - dq) * kd

def _low_pass_action_filter(actions,last_actions):
    actons_filtered = last_actions * 0.2 + actions * 0.8
    return actons_filtered
    
def run_mujoco(cfg):
    global action_rl,default_dof_pos
    """
    通过LCM作为通用接口传输传感器原始数据，接受网络直接输出
    """
    model = mujoco.MjModel.from_xml_path(cfg.sim_config.mujoco_model_path)#载入初始化位置由XML决定
    model.opt.timestep = cfg.sim_config.dt
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data)
    viewer = mujoco_viewer.MujocoViewer(model, data)

    target_q = np.zeros((cfg.env.num_actions), dtype=np.double)
    action = np.zeros((cfg.env.num_actions), dtype=np.double)
    action_flt = np.zeros((cfg.env.num_actions), dtype=np.double)
    last_actions = np.zeros((cfg.env.num_actions), dtype=np.double)
    hist_obs = deque()
    for _ in range(cfg.env.history_len):
        hist_obs.append(np.zeros([1, cfg.env.n_proprio], dtype=np.double))

    count_lowlevel = 0
    # <motor name="FL_hip_joint" joint="FL_hip_joint" gear="1" ctrllimited="true" ctrlrange="-12 12"/>
    # <motor name="FL_thigh_joint" joint="FL_thigh_joint" gear="1" ctrllimited="true" ctrlrange="-12 12"/>
    # <motor name="FL_calf_joint" joint="FL_calf_joint" gear="1" ctrllimited="true" ctrlrange="-12 12"/>

    # <motor name="FR_hip_joint" joint="FR_hip_joint" gear="1" ctrllimited="true" ctrlrange="-12 12"/>
    # <motor name="FR_thigh_joint" joint="FR_thigh_joint" gear="1" ctrllimited="true" ctrlrange="-12 12"/>
    # <motor name="FR_calf_joint" joint="FR_calf_joint" gear="1" ctrllimited="true" ctrlrange="-12 12"/>

    # <motor name="RL_hip_joint" joint="RL_hip_joint" gear="1" ctrllimited="true" ctrlrange="-12 12"/>
    # <motor name="RL_thigh_joint" joint="RL_thigh_joint" gear="1" ctrllimited="true" ctrlrange="-12 12"/>
    # <motor name="RL_calf_joint" joint="RL_calf_joint" gear="1" ctrllimited="true" ctrlrange="-12 12"/>

    # <motor name="RR_hip_joint" joint="RR_hip_joint" gear="1" ctrllimited="true" ctrlrange="-12 12"/>
    # <motor name="RR_thigh_joint" joint="RR_thigh_joint" gear="1" ctrllimited="true" ctrlrange="-12 12"/>
    # <motor name="RR_calf_joint" joint="RR_calf_joint" gear="1" ctrllimited="true" ctrlrange="-12 12"/>
    # }
    
    subscription = lcm.subscribe("LCM_ACTION", my_handler)

    for _ in tqdm(range(int(cfg.sim_config.sim_duration / cfg.sim_config.dt)), desc="Simulating..."):
        
        # Obtain an observation
        q, dq, quat, v, omega, gvec = get_obs(data)#从mujoco获取仿真数据
        q = q[-cfg.env.num_actions:]
        dq = dq[-cfg.env.num_actions:]
        
        # obs_buf =torch.cat((self.base_ang_vel  * self.obs_scales.ang_vel,
        #                     self.base_euler_xyz * self.obs_scales.quat,
        #                     self.commands[:, :3] * self.commands_scale,#xy+航向角速度
        #                     self.reindex((self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos),
        #                     self.reindex(self.dof_vel * self.obs_scales.dof_vel),
        #                     self.action_history_buf[:,-1]),dim=-1)#列表最后一项 [:-1]也就是上一次的

        if 1:
            # 1000hz -> 100hz
            if count_lowlevel % cfg.sim_config.decimation == 0:

                obs = np.zeros([1, cfg.env.n_proprio], dtype=np.float32) #1,45

                eu_ang = quaternion_to_euler_array(quat)
                eu_ang[eu_ang > math.pi] -= 2 * math.pi

                cmd.vx=0.25
                cmd.vy=0.0
                cmd.dyaw= 0.3
                #---send obs
                msg = Request()
                msg.eu_ang=eu_ang
                msg.omega=omega
                msg.command[0]=cmd.vx
                msg.command[1]=cmd.vy
                msg.command[2]=cmd.dyaw
                msg.q=q
                msg.dq=dq
                lcm.publish("LCM_OBS", msg.encode())
                lcm.handle()
                #--LCM传输RL滤波后的输出
                action = np.clip(action_rl, -cfg.normalization.clip_actions, cfg.normalization.clip_actions)
                target_q = action * cfg.control.action_scale + default_dof_pos
            
            target_dq = np.zeros((cfg.env.num_actions), dtype=np.double)
            # Generate PD control
            tau = pd_control(target_q, q, cfg.robot_config.kps,
                             target_dq, dq, cfg.robot_config.kds)  # Calc torques
            tau = np.clip(tau, -cfg.robot_config.tau_limit, cfg.robot_config.tau_limit)  # Clamp torques
            data.ctrl = tau
        else:#air mode test
            obs = np.zeros([1, cfg.env.n_proprio], dtype=np.float32) #1,45
            eu_ang = quaternion_to_euler_array(quat)
            eu_ang[eu_ang > math.pi] -= 2 * math.pi
            target_q = default_dof_pos
            # target_q[0]=0
            # target_q[1]=3
            # target_q[2]=3
            # target_q[3]=0
            # target_q[4]=3
            # target_q[5]=3     
            #print(eu_ang*57.3)
            target_dq = np.zeros((cfg.env.num_actions), dtype=np.double)
            # Generate PD control
            tau = pd_control(target_q, q, cfg.robot_config.kps,
                            target_dq, dq, cfg.robot_config.kds)  # Calc torques
            tau = np.clip(tau, -cfg.robot_config.tau_limit, cfg.robot_config.tau_limit)  # Clamp torques
            data.ctrl = tau

        
        mujoco.mj_step(model, data)
        viewer.render()
        count_lowlevel += 1

    viewer.close()

 

if __name__ == '__main__':#lcm
    import argparse

    parser = argparse.ArgumentParser(description='Deployment script.')
    parser.add_argument('--terrain', action='store_true', default=False)
    args = parser.parse_args()
    
    class Sim2simCfg(TinymalConstraintHimRoughCfg):

        class sim_config:
            if args.terrain:
                mujoco_model_path = f'{ROOT_DIR}/resources/tinymal/xml/world_terrain.xml'
            else:
                mujoco_model_path = f'{ROOT_DIR}/resources/tinymal/xml/world.xml'
            sim_duration = 60.0
            dt = 0.001 #1Khz底层
            decimation = 20 # 50Hz

        class robot_config:
            kp_all = 4
            kd_all = 0.15
            kps = np.array([kp_all, kp_all, kp_all, kp_all, kp_all, kp_all, kp_all, kp_all, kp_all, kp_all, kp_all, kp_all], dtype=np.double)#PD和isacc内部一致
            kds = np.array([kd_all, kd_all, kd_all, kd_all, kd_all, kd_all, kd_all, kd_all, kd_all, kd_all, kd_all, kd_all], dtype=np.double)
            tau_limit = 12. * np.ones(12, dtype=np.double)#nm
    
    run_mujoco(Sim2simCfg())
