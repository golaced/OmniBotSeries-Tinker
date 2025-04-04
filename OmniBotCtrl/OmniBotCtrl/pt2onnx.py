# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import torch

ROOT_DIR = os.path.dirname(__file__)
ENVS_DIR = os.path.join(ROOT_DIR,'Env')


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Deployment script.')
    parser.add_argument('--load_model', type=str, default='/home/pi/Downloads/LocomotionWithNP3O-master/modelt.pt',
                        help='Run to load from.')
    parser.add_argument('--terrain', action='store_true', default=False)
    args = parser.parse_args()

    model = torch.load(args.load_model)# 有一个可能得原因是这个 pt文件里只有权重，而没有网络结构。 所以 只能用 torch.load去加载，不能用torch.jit.load
    model.eval()
    model.float()
 
 # Let's create a dummy input tensor  
    dummy_input = torch.randn(1, 3, 32, 32, requires_grad=True)  
    # Export the model   
    torch.onnx.export(model,         # model being run 
         dummy_input,       # model input (or a tuple for multiple inputs) 
         "bset.onnx",       # where to save the model  
         export_params=True,  # store the trained parameter weights inside the model file 
         opset_version=11,    # the ONNX version to export the model to 
         do_constant_folding=True,  # whether to execute constant folding for optimization 
         input_names = ['input'],   # the model's input names 
         output_names = ['output'], # the model's output names 
         dynamic_axes={'input' : {0 : 'batch_size'},    # variable length axes 
                      'output' : {0 : 'batch_size'}}) 
    print(" ") 
    print('Model has been converted to ONNX') 