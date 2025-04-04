# OmniBotSeries-Tinker
Building a high-performance bipedal robot that combines Pixar and Disney BDX

<table>
  <tr>
    <td> <img src="https://github.com/OmniBotSeries-Tinker/Home_Image/1.png" alt="1" width="300px" ></td>
    <td> <img src="https://github.com/OmniBotSeries-Tinker/Home_Image/2.png" alt="2" width="300px" ></td>
    <td> <img src="https://github.com/OmniBotSeries-Tinker/Home_Image/3.png" alt="3" width="300px" ></td>
   </tr> 
</table>

We are making a miniature version of the BDX Droid by Disney. It is about 42 centimeters tall with its legs extended.
The full BOM cost should be under $400 !

This repo is kind of a hub where we centralize all resources related to this project. This is a working repo, so there are a lot of undocumented scripts :) We'll try to clean things up at some point.


# State of sim2real

The gait is getting better ! 

https://github.com/OmniBotSeries-Tinker/Home_Image/tinker_real_demo

https://github.com/OmniBotSeries-Tinker/Home_Image/tinker_real_back_push_smaller


# Updates

> Update 02/04/2024: You can try two policies we trained : [this one](BEST_WALK_ONNX.onnx) and [this one](BEST_WALK_ONNX_2.onnx)
> I run with the following arguments :
> python v2_rl_walk_mujoco.py --onnx_model_path ~/BEST_WALK_ONNX_2.onnx -p 32 --commands --cutoff_frequency 40

> Update 15/03/2025: join our discord server to get help or show us your duck :) https://discord.gg/UtJZsgfQGe

> Update 07/02/2025: Big progress on sim2real, see videos above :)

> Update 24/02/2025: Working hard on sim2real ! 

> Update 07/02/2025 : We are writing documentation on the go, but the design and BOM should not change drastically. Still missing the "expression" features, but they can be added after building the robot!

> Update 22/01/2025 : The mechanical design is pretty much finalized (fixing some mistakes here and there). The current version does not include all the "expression" features we want to include in the final robot (LEDs for the eyes, a camera, a speaker and a microphone). We are now working on making it walk with reinforcement learning !

# CAD

https://github.com/OmniBotSeries-Tinker/Home_Image/jixie

See [this document](docs/prepare_robot.md) for getting from a onshape design to a simulated robot in MuJoCo

# PCB

https://github.com/OmniBotSeries-Tinker/Home_Image/PCB

See [this document](docs/prepare_robot.md) for getting from a onshape design to a simulated robot in MuJoCo

# RL stuff

We are switching to Mujoco Playground, see this [repo](https://github.com/apirrone/Open_Duck_Playground)

https://github.com/OmniBotSeries-Tinker/Home_Image/tinker_v2-2025-03-18

We are still working with [AWD](https://github.com/rimim/AWD) too

https://github.com/OmniBotSeries-Tinker/Home_Image/ref_motion_demo

## AI xiaozhi for LLM

https://github.com/OmniBotSeries-Tinker/Home_Image/xiaozhiAI

See [this repo](https://github.com/apirrone/Open_Duck_reference_motion_generator)


# BOM

https://hcn64ij2s2xr.feishu.cn/wiki/DPWDwNWaZiGNvpkbRrGcBXCHn7d

# Build Guide
https://hcn64ij2s2xr.feishu.cn/wiki/AZJxwlvEpiWnRrkeBbGc21U9n7f



## Assembly Guide

https://hcn64ij2s2xr.feishu.cn/wiki/FqmXwnRYliwYFbkXCO7csSHWnfh

# Embedded runtime

This repo contains the code to run the policies on the onboard computer (Raspberry pi zero 2w) https://hcn64ij2s2xr.feishu.cn/wiki/MJ25wew9PijvXlkIoH7cZ9e4nCh

# Training your own policies

When we get a very nice stable and robust walk that transfers to the real robot, we'll provide it here (you can ask for a checkpoint on the discord too if you want to try).

If you want to train your own policies, and contribute to making the ducks walk nicely, see [this document](https://hcn64ij2s2xr.feishu.cn/wiki/LJxZwgL1diHB8ykKU1acFnXknOf)


# Global project TODO

- [ ] Centralize the URDF/MJCF. Separate repo ? Menagerie ? 

> Thanks a lot to HuggingFace and Pollen Robotics for sponsoring this project !