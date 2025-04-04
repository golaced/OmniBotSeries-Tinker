#!/usr/bin/expect

# 定义密码变量
set password "1"

# 定义文件路径和目标
set local_file {model_jitt.pt modelt.pt}
set remote_user "nvidia"
set remote_host "192.168.1.146"
set remote_path "/home/nvidia/sim2sim_lcm"

# 启动 scp 传输
foreach file $local_file {
    spawn scp $file $remote_user@$remote_host:$remote_path
    expect "password:" {
        send "$password\r"
    }
    expect eof
}

# 启动 ssh 会话
spawn ssh $remote_user@$remote_host

# 匹配密码提示并自动输入
expect "password:" {
    send "$password\r"
}

expect "$ " {
    send "cd /home/nvidia/sim2sim_lcm/build\r"
}
expect "$ " {
    send "./udp_publisher_tinker\r"
}
# 不退出，保持远程会话
interact
