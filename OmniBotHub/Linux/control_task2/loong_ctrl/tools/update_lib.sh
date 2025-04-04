#!/bin/bash

#sudo apt install expect


# echo update_lib.sh 首次运行，务必修改下方dir路径，匹配机器人中个人项目文件夹！然后注释本两行以继续执行！
# exit

#务必修改dir！否则会覆盖别人的运行环境！
dir="~/lyf/module/nabo/"

sshCopy(){
    expect <(cat <<EOF
    set timeout 1
    spawn scp ../nabo_output/libnabo_arm64.so odroid@192.168.1.$1:$dir
    expect "password:" {
        send "1\n"
        interact
        puts "已更新文件{$1}号机"
    }
EOF
)
}

cat <(sshCopy 246)
sleep 1


