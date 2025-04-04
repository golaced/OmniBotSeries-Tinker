#!/usr/bin/env python3
'''=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

安装tvm: pip install pip install apache-tvm
看网络： netron policy_1.pt
jit必须用trace后的模型
======================================================'''

import torch
import tvm
from tvm import relay


# 已经jit trace后的模型导入
model=torch.jit.load("./model_jitt.pt")
# print(model)
model=model.float() #确保模型的所有参数使用浮动点精度（float32），这里进行强制转换为浮动点类型。


shape_list = [("input0",(1,39)),("input1",(1,10,39))] #定义输入张量的形状，模型有两个输入
mod,param=relay.frontend.from_pytorch(model,shape_list)#TVM 的一个前端API，用于将PyTorch模型转换为TVM的Relay IR (Intermediate Representation) 格式。不仅转换了模型的计算图，还包括了参数。

# # ==x64=======
target = tvm.target.Target("llvm", host="llvm")#指定目标架构为x64 CPU，llvm是TVM支持的后端编译器工具链，host="llvm" 指定主机也使用LLVM。
with tvm.transform.PassContext(opt_level=3):#指定优化级别为 3（最高优化级别），以尽可能优化模型的执行性能
    lib = relay.build(mod, target=target, params=param)#将 Relay 中间表示编译成目标平台（x64）上可执行的共享库（so文件）
lib.export_library("./policy_x64_cpu.so")#将生成的共享库导出为.so文件，这个文件可以在x64架构的 CPU 上运行

#==arm64=======
target = tvm.target.Target("llvm -mtriple=aarch64-linux-gnu")#指定目标架构为ARM64。-mtriple=aarch64-linux-gnu用于告诉LLVM编译器目标是ARM64(aarch64)平台上的 Linux 系统
with tvm.transform.PassContext(opt_level=3):
    lib = relay.build(mod, target=target, params=param)
lib.export_library("./policy_arm64_cpu.so", cc='/usr/bin/aarch64-linux-gnu-g++-9')#指定了编译器cc参数为aarch64-linux-gnu-g++-9，这是一个适用于ARM64的C++编译器，通常用于交叉编译


