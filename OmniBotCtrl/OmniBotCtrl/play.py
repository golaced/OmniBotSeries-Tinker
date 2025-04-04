
#import cv2
import os

from isaacgym import gymapi
from envs import LeggedRobot,LeggedRobot4Leg,LeggedRobot2Leg
from modules import *
from utils import  get_args, export_policy_as_jit, task_registry, Logger
from configs import *
from utils.helpers import class_to_dict
from utils.task_registry import task_registry
import numpy as np
import torch
from global_config import ROOT_DIR,PLAY_DIR
from global_config import ROBOT_SEL,GAIT_SEL
from PIL import Image as im
from configs.go2_constraint_him import Go2ConstraintHimRoughCfg, Go2ConstraintHimRoughCfgPPO
if GAIT_SEL=='Trot':
    from configs.tinymal_constraint_him_trot import TinymalConstraintHimRoughCfg, TinymalConstraintHimRoughCfgPPO
else:
    from configs.tinymal_constraint_him_stand import TinymalConstraintHimRoughCfg, TinymalConstraintHimRoughCfgPPO

if GAIT_SEL=='Trot':
    from configs.tinker_constraint_him_trot import TinkerConstraintHimRoughCfg, TinkerConstraintHimRoughCfgPPO
else:
    from configs.tinker_constraint_him_stand import TinkerConstraintHimRoughCfg, TinkerConstraintHimRoughCfgPPO

if GAIT_SEL=='Trot':
    from configs.taitan_constraint_him_trot import TaitanConstraintHimRoughCfg, TaitanConstraintHimRoughCfgPPO
else:
    from configs.taitan_constraint_him_stand import TaitanConstraintHimRoughCfg, TaitanConstraintHimRoughCfgPPO

from utils.ploter import Plotter, initCanvas
import matplotlib.pyplot as plt
import random
# conda activate HIT
en_plot = 0

def delete_files_in_directory(directory_path):
   try:
     files = os.listdir(directory_path)
     for file in files:
       file_path = os.path.join(directory_path, file)
       if os.path.isfile(file_path):
         os.remove(file_path)
     print("All files deleted successfully.")
   except OSError:
     print("Error occurred while deleting files.")

def rand_commands(command_ranges,commands):
    commands[0] = random.uniform(command_ranges.lin_vel_x[0], command_ranges.lin_vel_x[1])
    commands[1] = random.uniform(command_ranges.lin_vel_y[0], command_ranges.lin_vel_y[1])
    commands[2] = 0
    # if self.cfg.commands.heading_command:
    commands[3] = random.uniform(command_ranges.heading[0], command_ranges.heading[1])
    # else:
    #     self.commands[env_ids, 2] = torch_rand_float(self.command_ranges["ang_vel_yaw"][0], self.command_ranges["ang_vel_yaw"][1], (len(env_ids), 1), device=self.device).squeeze(1)

    # # set small commands to zero
    #commands[:2] *= (norm(commands[2]) > 0.2)

def play(args):
    
    
    print(torch.__version__)
    print(torch.cuda.is_available())
    print(torch.cuda.device_count())
    torch.version.cuda
    env_cfg, train_cfg = task_registry.get_cfgs(name=args.task)
    # override some parameters for testing
    video_duration = 200 #总体时间s
    
    env_cfg.env.num_envs = min(env_cfg.env.num_envs, 1)
    env_cfg.terrain.num_rows = 3
    env_cfg.terrain.num_cols = 3
    env_cfg.terrain.curriculum = False
    env_cfg.noise.add_noise = False
    #env_cfg.terrain.mesh_type = 'plane'
    env_cfg.domain_rand.push_robots = True
    #env_cfg.domain_rand.randomize_friction = False
    env_cfg.domain_rand.randomize_base_com = False
    env_cfg.domain_rand.randomize_base_mass = False
    env_cfg.domain_rand.randomize_motor = False
    env_cfg.domain_rand.randomize_lag_timesteps = False
    env_cfg.noise.add_noise = True
    env_cfg.domain_rand.randomize_friction = False
    env_cfg.domain_rand.randomize_restitution = False 
    env_cfg.control.use_filter = True
    # prepare environment
    env, _ = task_registry.make_env(name=args.task, args=args, env_cfg=env_cfg)
    
    #amao isaac的策略输入
    obs = env.get_observations()
    # load policy partial_checkpoint_load
    policy_cfg_dict = class_to_dict(train_cfg.policy)
    runner_cfg_dict = class_to_dict(train_cfg.runner)
    actor_critic_class = eval(runner_cfg_dict["policy_class_name"])
    policy: ActorCriticRMA = actor_critic_class(env.cfg.env.n_proprio,#45
                                                      env.cfg.env.n_scan,
                                                      env.num_obs,
                                                      env.cfg.env.n_priv_latent,
                                                      env.cfg.env.history_len,#10
                                                      env.num_actions,
                                                      **policy_cfg_dict)
 
    model_dict = torch.load(os.path.join(ROOT_DIR, PLAY_DIR))#《---------------------调用的网络模型doghome

    #if 1:#full 
    policy.load_state_dict(model_dict['model_state_dict'])
    policy.half()
    policy = policy.to(env.device)
    torch.save(policy,'modelt.pt',)
    print('*****************')
    print(policy)
    #else:#origin
    policy.load_state_dict(model_dict['model_state_dict'])
    policy.half()
    policy = policy.to(env.device)
    policy.save_torch_jit_policy('model_jitt.pt',env.device)

    # policy.save_torch_jit_policy('model_jit.pt',env.device)
    # policy_1 = export_policy_as_jit(policy, 'logs/exported')

    # amao_policy=torch.jit.load('model_jit.pt')

    # clear images under frames folder
    # frames_path = os.path.join(ROOT_DIR, 'logs', train_cfg.runner.experiment_name, 'exported', 'frames')
    # delete_files_in_directory(frames_path)

    # set rgba camera sensor for debug and doudle check
    camera_local_transform = gymapi.Transform()
    camera_local_transform.p = gymapi.Vec3(-0.5, -1, 0.1)
    camera_local_transform.r = gymapi.Quat.from_axis_angle(gymapi.Vec3(0,0,1), np.deg2rad(90))
    camera_props = gymapi.CameraProperties()
    camera_props.width = 512
    camera_props.height = 512

    cam_handle = env.gym.create_camera_sensor(env.envs[0], camera_props)
    body_handle = env.gym.get_actor_rigid_body_handle(env.envs[0], env.actor_handles[0], 0)
    env.gym.attach_camera_to_body(cam_handle, env.envs[0], body_handle, camera_local_transform, gymapi.FOLLOW_TRANSFORM)

    img_idx = 0

    num_frames = int(video_duration / env.dt)
    print(f'gathering {num_frames} frames')
    video = None


    action_rate = 0
    z_vel = 0
    xy_vel = 0
    feet_air_time = 0

    env_cfg.commands.ranges.lin_vel_x = [-0.0, 0.5]
    env_cfg.commands.ranges.lin_vel_y = [-0.5, 0.5]
    env_cfg.commands.ranges.ang_vel_yaw = [-1., 1.]
    env_cfg.commands.ranges.heading = [-1., 1.]

    env.commands[:,0] = 0
    env.commands[:,1] = 0
    env.commands[:,2] = 0
    env.commands[:,3] = 0
    if en_plot:
      plt.ion()
      initCanvas(3, 2, 100)

      plotter0 = Plotter(0, 'base_velx')
      plotter1 = Plotter(1, 'header')

      plotter2 = Plotter(2, 'joint hip')
      plotter3 = Plotter(3, 'joint thigh')
      plotter4 = Plotter(4, 'joint calf')
      # plotter5 = Plotter(5, 'joint 5')
    #  
    for i in range(num_frames):
        action_rate += torch.sum(torch.abs(env.last_actions - env.actions),dim=1)
        z_vel += torch.square(env.base_lin_vel[:, 2])
        xy_vel += torch.sum(torch.square(env.base_ang_vel[:, :2]), dim=1)

        if i %500==0 or i==0:
          commands=[0]*4#初始化为0的4维度
          rand_commands(env_cfg.commands.ranges,commands)
          print("resample comand:",commands)
          env.commands[:,0] = commands[0]#0.35#控制指令
          env.commands[:,1] = commands[1]#
          env.commands[:,2] = commands[2]#
          env.commands[:,3] = commands[3]# #header
        if 0:#force stop
          env.commands[:,0] = 0
          env.commands[:,1] = 0
          env.commands[:,2] = 0
          env.commands[:,3] = 0         
        actions = policy.act_teacher(obs.half())#  
        #print(actions)
        if 0:#debug
            actions[0,0]=0#FL
            actions[0,1]=0
            actions[0,2]=0

            actions[0,3]=0#FR
            actions[0,4]=0
            actions[0,5]=0

            actions[0,6]=0#RL
            actions[0,7]=0
            actions[0,8]=0

            actions[0,9]=0#RR
            # actions[0,10]=3
            # actions[0,11]=3
            print(obs[0,3:6]*57.3)#att
        # actions = torch.clamp(actions,-1.2,1.2)
        # print('amaomao-------------')
        # obs_cpu = obs.detach().cpu().numpy()  # 首先将Tensor移动到CPU，然后转换为NumPy数组 
        # for i in range(3):
        #   print("{:.2f}".format(obs_cpu[0][i]))
        # for i in range(3):  
        #   print("{:.2f}".format(obs_cpu[0][i+3]))
        obs, privileged_obs, rewards,costs,dones, infos = env.step(actions)#
        env.gym.step_graphics(env.sim) # required to render in headless mode
        env.gym.render_all_camera_sensors(env.sim)

        #----ploter
        if en_plot:
          #lin/ang vel
          plotter0.plotLine(env.base_lin_vel[0, 0].item(), env.commands[0, 0].item(), labels=['actual', 'command'])
          plotter1.plotLine(env.base_euler_xyz[0, 2].item(), env.commands[0, 3].item(), labels=['actual', 'command'])
          # actions avg
          plotter2.plotLine(env.dof_pos[0, 0].item(), env.action_avg[0, 0].item(),labels=['q', 'exp'])
          plotter3.plotLine(env.dof_pos[0, 1].item(), env.action_avg[0, 1].item(),labels=['q', 'exp'])
          plotter4.plotLine(env.dof_pos[0, 2].item(), env.action_avg[0, 2].item(),labels=['q', 'exp'])
        if RECORD_FRAMES:
            img = env.gym.get_camera_image(env.sim, env.envs[0], cam_handle, gymapi.IMAGE_COLOR).reshape((512,512,4))[:,:,:3]
            if video is None:
                video = cv2.VideoWriter('record.mp4', cv2.VideoWriter_fourcc(*'MP4V'), int(1 / env.dt), (img.shape[1],img.shape[0]))
            video.write(img)
            img_idx += 1 
    print("action rate:",action_rate/num_frames)
    print("z vel:",z_vel/num_frames)
    print("xy_vel:",xy_vel/num_frames)
    print("feet air reward",feet_air_time/num_frames)
    if RECORD_FRAMES:
      video.release()

    #test model profile
    with torch.profiler.profile(activities=[torch.profiler.ProfilerActivity.CPU, torch.profiler.ProfilerActivity.CUDA]) as prof:
         for i in range(1000):
            with torch.no_grad():
              actions = policy.act_teacher(obs.half())
    print(prof.key_averages().table(sort_by="self_cuda_time_total", row_limit=10))

if __name__ == '__main__':
    task_registry.register("go2N3poHim",LeggedRobot,Go2ConstraintHimRoughCfg(),Go2ConstraintHimRoughCfgPPO())
    task_registry.register("Tinymal",LeggedRobot4Leg,TinymalConstraintHimRoughCfg(),TinymalConstraintHimRoughCfgPPO())
    task_registry.register("Tinker",LeggedRobot,TinkerConstraintHimRoughCfg(),TinkerConstraintHimRoughCfgPPO())
    task_registry.register("Taitan",LeggedRobot2Leg,TaitanConstraintHimRoughCfg(),TaitanConstraintHimRoughCfgPPO())

    RECORD_FRAMES = False
    args = get_args()
    #args.task='Tinymal'
    #args.task='Tinker'
    args.task=ROBOT_SEL
    play(args)
