程序框架

# 框架核心思想：隔离！

为防止屎山出现，任何添加的功能模块，必须以最小成本、最小作用域的形式进行添加。

# 框架使用（简）

所有常用操作均在tools文件夹内即可完成

* **仿真（x64平台)：**

```
cd tools
./make.sh x      #自动cp生成库到mujoco目录
./run_mujoco.sh  #同mujoco内运行同名脚本，加数字参数可缓速运行
```

* **实机（x64平台编译，arm64平台运行）**

```
cd tools
./make.sh        #自动交叉编译、scp生成库到实机（务必在update_lib.sh中修改scp目标，以免覆盖他人文件夹）
                 #实机内运行./run.sh
```

# 框架结构（简）

框架顶层为状态机fsm

添加功能的最大单位被称为plan，plan在manager内被add进状态机fsm，fsm负责调度所有的plan

为方便操作，manager内采用宏定义区别了仿真与实机plan的切换逻辑

plan内完成各自所需的所有子功能模块，不同plan之间在调度上互相独立

plan之间复用的模块、数据，推荐以单例形式调用

程序初始会调用每个plan的init

程序运行过程中fms会调用plan的进入(hey)、运行(run)、通知退出(bye)、写日志(log)四个接口，plan自身通过运行(run)的返回值负责自身的退出。另外fms会根据plan的优先级强制plan退出
