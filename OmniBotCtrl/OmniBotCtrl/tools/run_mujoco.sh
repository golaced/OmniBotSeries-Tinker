#!/bin/bash

cd ../../loong_mujoco/build
# 删除最近10个以外的log文件
find "../log" -type f -printf '%T@ %p\n' | sort -nr | tail -n +11 | cut -d' ' -f2- | xargs rm -f

./main $1







# cd ../log
# files=$(ls -lt | tail -n +11 | awk '{print $9}')
# for file in $files; do
# 	echo "toybrick"|sudo rm -f "$file"
# done