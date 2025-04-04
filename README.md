# OmniBotSeries-Tinker
Building a high-performance bipedal robot that combines Pixar and Disney BDX
<table>
  <tr>
    <td> <img src="https://github.com/golaced/OmniBotSeries-Tinker/blob/main/Home_Image/1.png" alt="1" width="300px" ></td>
    <td> <img src="https://github.com/golaced/OmniBotSeries-Tinker/blob/main/Home_Image/2.png" alt="2" width="300px" ></td>
    <td> <img src="https://github.com/golaced/OmniBotSeries-Tinker/blob/main/Home_Image/3.jpg" alt="3" width="300px" ></td>
   </tr> 
</table>

In this project, we combined Pixar's lamp robot and Disney's BDX robot to build a 60cm high-performance Tinker bipedal robot.
The entire robot uses a brushless motor, runs a reinforcement learning algorithm and comes with an open source tutorial. The cost of the whole machine is less than 15,000 RMB!

# State of sim2real
The gait of TinkerV2 version is highly consistent with the BDX effect ! 

<video src="https://github.com/golaced/OmniBotSeries-Tinker/blob/main/Home_Image/tinker_real_demo.mp4" controls="controls" width="500" height="300"></video>

https://github.com/golaced/OmniBotSeries-Tinker/blob/main/Home_Image/tinker_real_back_push_smaller

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
https://github.com/golaced/OmniBotSeries-Tinker/blob/main/Home_Image/jixie
See [this document](https://hcn64ij2s2xr.feishu.cn/wiki/DPWDwNWaZiGNvpkbRrGcBXCHn7d) for getting from the CAD design of Tinker

# PCB
https://github.com/golaced/OmniBotSeries-Tinker/blob/main/Home_Image/PCB

See [this document](https://hcn64ij2s2xr.feishu.cn/wiki/DPWDwNWaZiGNvpkbRrGcBXCHn7d) for getting from the PCB design of Tinker

# RL stuff
We are switching to IsaccGym, see this [repo](https://hcn64ij2s2xr.feishu.cn/wiki/LJxZwgL1diHB8ykKU1acFnXknOf)
https://github.com/golaced/OmniBotSeries-Tinker/blob/main/Home_Image/tinker_v2-2025-03-18

https://github.com/golaced/OmniBotSeries-Tinker/blob/main/Home_Image/ref_motion_demo

## AI xiaozhi with LLM
Robots can install Xiaozhi AI modules to implement large language model functions !
https://github.com/golaced/OmniBotSeries-Tinker/blob/main/Home_Image/xiaozhiAI

See [this repo](https://github.com/78/xiaozhi-esp32)

# BOM
https://hcn64ij2s2xr.feishu.cn/wiki/DPWDwNWaZiGNvpkbRrGcBXCHn7d

# Build Guide
https://hcn64ij2s2xr.feishu.cn/wiki/AZJxwlvEpiWnRrkeBbGc21U9n7f
## Assembly Guide
https://hcn64ij2s2xr.feishu.cn/wiki/FqmXwnRYliwYFbkXCO7csSHWnfh
# Sofeware Guide
This repo contains the software to run the policies of TVM on the onboard computer (Odroid C4) https://hcn64ij2s2xr.feishu.cn/wiki/MJ25wew9PijvXlkIoH7cZ9e4nCh
# Training your own policies
If you want to train your own policies, see [this document](https://hcn64ij2s2xr.feishu.cn/wiki/LJxZwgL1diHB8ykKU1acFnXknOf)

