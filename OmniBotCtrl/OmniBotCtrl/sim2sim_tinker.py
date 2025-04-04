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
from global_config import ROOT_DIR,SPD_X,SPD_Y,SPD_YAW
from configs.back.tinker_constraint_him import TinkerConstraintHimRoughCfg
import torch

#default_dof_pos=[-0.16,0.68,1.3 ,0.16,0.68,1.3, -0.16,0.68,1.3, 0.16,0.68,1.3]#默认角度需要与isacc一致
default_dof_pos = [0.0,0.08,0.56,-1.12,-0.57,  0.0,-0.08,-0.56,1.12,0.57]

class cmd:
    vx = 0.2
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
    return (q, dq, quat, v, omega, gvec)

def pd_control(target_q, q, kp, target_dq, dq, kd):
    '''Calculates torques from position commands
    '''
    return (target_q - q) * kp + (target_dq - dq) * kd

def _low_pass_action_filter(actions,last_actions):
    flt =0.1
    actons_filtered = last_actions * flt + actions * (1-flt)
    return actons_filtered
    
def run_mujoco(policy, cfg):
    global default_dof_pos
    """
    Run the Mujoco simulation using the provided policy and configuration.

    Args:
        policy: The policy used for controlling the simulation.
        cfg: The configuration object containing simulation settings.

    Returns:
        None
    """
    model = mujoco.MjModel.from_xml_path(cfg.sim_config.mujoco_model_path)#载入初始化位置由XML决定
    model.opt.timestep = cfg.sim_config.dt
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data)
    viewer = mujoco_viewer.MujocoViewer(model, data)

    target_q = np.zeros((cfg.env.num_actions), dtype=np.double)     # 10
    action = np.zeros((cfg.env.num_actions), dtype=np.double)       # 10
    action_flt = np.zeros((cfg.env.num_actions), dtype=np.double)   # 10
    last_actions = np.zeros((cfg.env.num_actions), dtype=np.double) # 10
    hist_obs = deque()
    for _ in range(cfg.env.history_len):
        hist_obs.append(np.zeros([1, cfg.env.n_proprio], dtype=np.double)) # 39

    count_lowlevel = 0

    for _ in tqdm(range(int(cfg.sim_config.sim_duration / cfg.sim_config.dt)), desc="Simulating..."):

        # Obtain an observation
        q, dq, quat, v, omega, gvec = get_obs(data)#从mujoco获取仿真数据
        #print("q1",q)
        q = q[-cfg.env.num_actions:]
        #print("q2",q)
        dq = dq[-cfg.env.num_actions:]
        
        # obs_buf =torch.cat((self.base_ang_vel  * self.obs_scales.ang_vel,
        #                     self.base_euler_xyz * self.obs_scales.quat,
        #                     self.commands[:, :3] * self.commands_scale,#xy+航向角速度
        #                     self.reindex((self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos),
        #                     self.reindex(self.dof_vel * self.obs_scales.dof_vel),
        #                     self.action_history_buf[:,-1]),dim=-1)#列表最后一项 [:-1]也就是上一次的

        if 1:
            # 1000hz ->50hz
            if count_lowlevel % cfg.sim_config.decimation == 0:

                obs = np.zeros([1, cfg.env.n_proprio], dtype=np.float32) #1,45

                eu_ang = quaternion_to_euler_array(quat)
                eu_ang[eu_ang > math.pi] -= 2 * math.pi

                cmd.vx=SPD_X
                cmd.vy=SPD_Y
                cmd.dyaw= SPD_YAW
                #sensor->lcm
                #单次观测
                obs[0, 0] = omega[0] *cfg.normalization.obs_scales.ang_vel
                obs[0, 1] = omega[1] *cfg.normalization.obs_scales.ang_vel
                obs[0, 2] = omega[2] *cfg.normalization.obs_scales.ang_vel
                obs[0, 3] = eu_ang[0] *cfg.normalization.obs_scales.quat
                obs[0, 4] = eu_ang[1] *cfg.normalization.obs_scales.quat
                obs[0, 5] = eu_ang[2] *cfg.normalization.obs_scales.quat
                obs[0, 6] = cmd.vx * cfg.normalization.obs_scales.lin_vel
                obs[0, 7] = cmd.vy * cfg.normalization.obs_scales.lin_vel
                obs[0, 8] = cmd.dyaw * cfg.normalization.obs_scales.ang_vel
                #print("q3",q)
                #print("default_dof_pos",default_dof_pos)

                obs[0, 9:19] = (q-default_dof_pos) * cfg.normalization.obs_scales.dof_pos #g关节角度顺序依据修改为样机
                obs[0, 19:29] = dq * cfg.normalization.obs_scales.dof_vel
                obs[0, 29:39] = last_actions#上次控制指令
                obs = np.clip(obs, -cfg.normalization.clip_observations, cfg.normalization.clip_observations)

                # obs_cpu = obs  # 首先将Tensor移动到CPU，然后转换为NumPy数组 
                # for i in range(3):
                #     print("{:.2f}".format(obs_cpu[0][i]))
                # for i in range(3):  
                #     print("{:.2f}".format(obs_cpu[0][i+3]))

                hist_obs.append(obs) #11,1,45
                hist_obs.popleft() #10,1,45

                n_proprio=cfg.env.n_proprio
                n_priv_latent=cfg.env.n_priv_latent
                n_scan=cfg.env.n_scan
                history_len=cfg.env.history_len
                num_observations= cfg.env.num_observations# cfg.env.n_proprio + cfg.env.history_len*cfg.env.n_proprio

                policy_input = np.zeros([1, num_observations], dtype=np.float16) # 同isaac 完整
                hist_obs_input = np.zeros([1, history_len*n_proprio], dtype=np.float16) # 同isaac 观测buf
                #依据完成模型的顺序实际只采用了前后的观测数据
                policy_input[0,0:n_proprio]=obs
                for i in range(n_priv_latent  + n_scan):#缓存历史观测
                    policy_input[0,n_proprio+i]=0
                for i in range(history_len):#缓存历史观测
                    policy_input[0, n_proprio  + n_priv_latent  + n_scan +i * n_proprio : n_proprio  + n_priv_latent  + n_scan +(i + 1) * n_proprio] = hist_obs[i][0, :]
                #采集模型仅仅使用观测和buf
                for i in range(history_len):#缓存历史观测
                    hist_obs_input[0, i * n_proprio : (i + 1) * n_proprio] = hist_obs[i][0, :]
               
                policy = policy.to('cpu') #policy是half()
                action[:] = policy.act_teacher(torch.tensor(policy_input).half())[0].detach().numpy()#完整模型
                #action[:] = policy(torch.tensor(obs).half(),torch.tensor(hist_obs_input).half())[0].detach().numpy()#jit模型
  
                action = np.clip(action, -cfg.normalization.clip_actions, cfg.normalization.clip_actions)
                action_flt=_low_pass_action_filter(action,last_actions)
                last_actions=action

                target_q = action_flt * 0.25+ default_dof_pos
                # lcm->mujoco
                # print("action_flt:")
                #print("action_flt:",action_flt)
                #print("q:",q)
                #print("target_q:",target_q)

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


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Deployment script.')
    parser.add_argument('--load_model', type=str, default='./modelt.pt',
                        help='Run to load from.')
    parser.add_argument('--terrain', action='store_true', default=False)
    args = parser.parse_args()

    class Sim2simCfg(TinkerConstraintHimRoughCfg):

        class sim_config:
            if args.terrain:
                mujoco_model_path = f'{ROOT_DIR}/resources/TinkerV2_URDF/xml/world_terrain.xml'
            else:
                mujoco_model_path = f'{ROOT_DIR}/resources/TinkerV2_URDF/xml/world.xml'
            sim_duration = 60.0
            dt = 0.001 #1Khz底层
            decimation = 20 # 100Hz

        class robot_config:
            kp_all = 15.0
            kd_all = 0.5
            kps = np.array([kp_all, kp_all, kp_all, kp_all, kp_all, kp_all, kp_all, kp_all, kp_all, kp_all], dtype=np.double)#PD和isacc内部一致
            kds = np.array([kd_all, kd_all, kd_all, kd_all, kd_all, kd_all, kd_all, kd_all, kd_all, kd_all], dtype=np.double)
            tau_limit = 20. * np.ones(10, dtype=np.double)#nm

    policy = torch.load(args.load_model)# 有一个可能得原因是这个 pt文件里只有权重，而没有网络结构。 所以 只能用 torch.load去加载，不能用torch.jit.load
   
    run_mujoco(policy, Sim2simCfg())
