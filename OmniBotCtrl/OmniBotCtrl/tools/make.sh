#!/bin/bash

#!/bin/bash
cd ..
if [ $1 ];then
    if [ ! -e "build_x64" ]; then
        mkdir build_x64
    fi
    cd build_x64
    cmake .. -Dsim_mode=1
    make -j4 install
    echo === 本机x64，复制.so到mujoco ===
    cp ../nabo_output/libnabo_x64.so ../../loong_mujoco/module/nabo/
else
    if [ ! -e "build_arm64" ]; then
        mkdir build_arm64
    fi
    cd build_arm64
    cmake .. -Dsim_mode=0
    make -j4 install
    echo ==== 实机arm64，复制.so到ssh ===
    cd ../tools/
    ./update_lib.sh
fi


