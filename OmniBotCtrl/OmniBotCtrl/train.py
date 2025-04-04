import numpy as np
import os
import isaacgym
from datetime import datetime
from global_config import ROBOT_SEL,GAIT_SEL
from configs.go2_constraint_him import Go2ConstraintHimRoughCfg, Go2ConstraintHimRoughCfgPPO

if GAIT_SEL=='Trot':
    from configs.tinker_constraint_him_trot import TinkerConstraintHimRoughCfg, TinkerConstraintHimRoughCfgPPO
else:
    from configs.tinker_constraint_him_stand import TinkerConstraintHimRoughCfg, TinkerConstraintHimRoughCfgPPO

if GAIT_SEL=='Trot':
    from configs.taitan_constraint_him_trot import TaitanConstraintHimRoughCfg, TaitanConstraintHimRoughCfgPPO
else:
    from configs.taitan_constraint_him_stand import TaitanConstraintHimRoughCfg, TaitanConstraintHimRoughCfgPPO

if GAIT_SEL=='Trot':
    from configs.tinymal_constraint_him_trot import TinymalConstraintHimRoughCfg, TinymalConstraintHimRoughCfgPPO
else:
    from configs.tinymal_constraint_him_stand import TinymalConstraintHimRoughCfg, TinymalConstraintHimRoughCfgPPO

from utils.helpers import get_args
from envs import LeggedRobot,LeggedRobot2Leg,LeggedRobot4Leg
from utils.task_registry import task_registry

def train(args):
    env, env_cfg = task_registry.make_env(name=args.task, args=args)
    ppo_runner, train_cfg = task_registry.make_alg_runner(env=env, name=args.task, args=args)
    ppo_runner.learn(num_learning_iterations=train_cfg.runner.max_iterations, init_at_random_ep_len=True)

if __name__ == '__main__':
    task_registry.register("go2N3poHim",LeggedRobot,Go2ConstraintHimRoughCfg(),Go2ConstraintHimRoughCfgPPO())
    task_registry.register("Tinymal",LeggedRobot4Leg,TinymalConstraintHimRoughCfg(),TinymalConstraintHimRoughCfgPPO())
    task_registry.register("Tinker",LeggedRobot,TinkerConstraintHimRoughCfg(),TinkerConstraintHimRoughCfgPPO())
    task_registry.register("Taitan",LeggedRobot2Leg,TaitanConstraintHimRoughCfg(),TaitanConstraintHimRoughCfgPPO())
    args = get_args()
    args.task=ROBOT_SEL
    train(args)
