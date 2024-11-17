# Unitree GO1 开发

Author: HanZhuo
Last updated: 2024/11/17

## 一、基础篇

### 1.unitree 机器狗：有限状态机(FSM)

`unitree_guide`中用到了有限状态机（FSM），FSM主要用于管理和控制机器人的行为，例如机器人的站立、姿态控制、前进等状态。`unitree_guide`中关于FSM的源码详见[FSM](https://github.com/unitreerobotics/unitree_guide/tree/main/unitree_guide/src/FSM)文件夹。

下图是通过宇树遥控器进行状态转换的方法。

![](https://s2.loli.net/2024/11/12/jrcUHvNOEuoG1sV.png)

下面具体介绍状态机。

**Passive**

机器人所有的关节电机被设置为阻尼模式，就是设置关节的期望转动速度为 0 rad/s  且刚度系数(Kp)设置为0.此时关节转动速度越大，相应地也会收到一个越大的阻力，继而快速降低关节的转动速度。如果此时机器人正处于站立状态，机器人将会在重力的作用下缓慢趴下。

**FixedStand**

机器人所有的关节将会缓慢转动到一个给定的关节角度然后锁定。例如你可以将给定的关节角度设置为机器人站立时的关节角度，即可实现机器人的站立。

**FreeStand**

机器人可以响应宇树遥控器或键盘的控制指令，来控制机器人原地站立时的身体高度和姿态。

**Trotting**

在此状态下，用户可以控制机器人行走、转弯、停下。

### 2.Gazebo仿真环境ROS基础控制

首先需要先下载三个ros包: `unitree_guide`、`unitree_ros`、`unitree)legged_msgs`.

接下来在ros工作空间下依次下载，假设ros工作空间名称为unitree_ws

打开一个终端

```bash
cd unitree_ws/src
git clone https://github.com/unitreerobotics/unitree_guide.git
git clone https://github.com/unitreerobotics/unitree_ros.git
git clone https://github.com/unitreerobotics/unitree_ros_to_real.git
```

![](https://s2.loli.net/2024/11/12/43dxqXH2Osh8GwR.png)

然后进入到`unitree_ros_to_real`目录下，将`unitree_legged_msgs`文件夹移动到`unitree_ws/src`目录下，再将`unitree_ros_to_real`文件夹删除，如下图所示
![](https://s2.loli.net/2024/11/12/5yRiwsr9UFKzQkp.png)

现在需要的功能包已经下载好，接下来编译工作空间，在`unitree_ws`目录下

```bash
catkin_make
```

编译成功如下图所示：

![](https://s2.loli.net/2024/11/12/LRo8HFq3vrBySOg.png)

下面我们就可以启动GO1仿真环境并通过键盘实现对GO1的控制了。

```bash
source unitree_ws/devel/setup.bash
roslaunch unitree_guide gazeboSim.launch
```

仿真环境如下图所示：
![](https://s2.loli.net/2024/11/12/x6EZ8zbdPwnVG3D.png)

我的配置是NVIDIA 4060RTX显卡，可以看到上图的下方FPS数据值为60+，我的仿真环境还是比较流畅的，配置低或者虚拟机的话，会比较卡顿是正常的。

```bash
source unitree_ws/devel/setup.bash
sudo ./unitree_ws/devel/lib/unitree_guide/junior_ctrl
```

然后光标停留在这个终端中，按下键盘上`2`之后，机器狗就会站立起来了，如下图
![](https://s2.loli.net/2024/11/12/BLYTyMmqEJtx16K.png)

有些教程中使用命令

```bash
rosrun unitree_guide junior_ctrl
```

这样虽然一开始也可以控制机器狗完成基本动作，但是这行命令运行之后会有

> [ERROR] Function setProcessScheduler failed.

![](https://s2.loli.net/2024/11/12/m7tSbfZAoePyXaq.png)

为什么需要直接使用编译生成的二进制文件却不用`rosrun`的方式，这是因为在机器人运动过程中，需要保证期望的足端力在一个固定的频率下(500hz)求解出来，而这涉及到实时进程的知识。在程序中设置实时进程需要最高权限，所以我们需要用`sudo`直接运行控制器的二进制文件。

启动控制器后，当仿真环境中的机器狗趴在地上时，我们可以通过按下键盘上的“2”将FSM由Passive(初始状态)转换到FixedStand，机器狗就会站立起来了，然后再按下“4”将FSM由FixedStand转到Trotting，然后我们就可以通过键盘来控制机器狗运动了。

按键`W`,`A`,`S`,`D`可以控制机器狗平移运动：向前、左、后、右行走。
按键`J`,`L`可以控制机器狗转向：向左、右转动。
按键`Spacebar`即空格键会令机器狗停下并站立。

以下是FSM状态转换的按键：

- Key '1': FixedStand/FreeStand to Passive
- Key '2': Passive/Trotting to FixedStand
- Key '3': Fixed stand to FreeStand
- Key '4': FixedStand/FreeStand to Trotting
- Key '5': FixedStand to MoveBase
- Key '8': FixedStand to StepTest
- Key '9': FixedStand to SwingTest
- Key '0': FixedStand to BalanceTest

### 3.PC端控制实体 GO1

有两种方式：宇树SDK控制和ROS控制。

#### 3.1 SDK 控制

通过`ssh`进入GO1运动板卡里面，找到里面的`unitree_legged_sdk`，里面应该是还未经过编译的，在实验室的GO1-7里面，我编译后未能成功控制实体运动，于是我从外部导入了新的[unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk.git)，放到主目录，已经编译好了，可以成功控制实体GO1运动，里面有五个example程序：
![](https://s2.loli.net/2024/11/12/P29XRKsAWgUzy5J.png)

例如程序`example_walk`，运行方法如下：

cd到程序所在目录，输入以下命令

```bash
sudo ./example_walk
```

运行程序后，GO1实体机器狗将会做出相应动作。

`unitree_legged_sdk`中除了cpp还有py程序文件

```bash
cd unitree_legged_sdk/example_py
```

![](https://s2.loli.net/2024/11/12/gh352CvQNzRGaqK.png)

#### 3.2 ROS 控制

首先需要下载几个github上[宇树](https://github.com/unitreerobotics)的包，下载时注意版本号，我会在下面说明版本的。

先给大家看一下下载好后的功能包（见下图）有三个包已经下载好了，那么还需要下载`unitree_legged_sdk`和`unitree_legged_real`，忽略`zhuo_go1`

![image-20241117221735345](https://s2.loli.net/2024/11/17/cY62KwdNDrMfblX.png)

接下来开始操作！

下载`unitree_legged_real`:

```bash
git clone -b foroldsdk https://github.com/unitreerobotics/unitree_ros_to_real.git
```

下载`unitree_legged_sdk`:

```
git clone -b v3.5.1 https://github.com/unitreerobotics/unitree_legged_sdk.git
```

（这里下载的都不是最新版本，最新版本大家感兴趣的话可以试一下）

接下来将工作空间编译一下，然后按照[unitree_ros_to_real](https://github.com/unitreerobotics/unitree_ros_to_real/blob/foroldsdk/README.md)的说明操作。
连接GO1的WIFI，然后打开一个终端，输入`ifconfig`查看自己电脑的信息

![](https://s2.loli.net/2024/11/17/WcHR2hVgD8qfIuK.png)

我们需要的是第二部分的网口名称`enp8s0`，每个人的具体名称会不一样，然后修改`unitree_legged_real`下的`ipconfig.sh`文件，将其中的`enx000ec6612921`改为自己的就可以了，之后打开终端

```bash
sudo chmod +x ipconfig.sh
sudo ./ipconfig.sh
```

这时就将端口信息和IP设置好了，不过这条命令是临时的，如果需要自动设置端口信息，进行下面的操作

```bash
sudo gedit /etc/network/interfaces
```

打开`interfaces`后，将下面四行添加进去，然后保存退出

```bash
auto enp8s0
iface enp8s0 inet static
address 192.168.123.162
netmask 255.255.255.0
```

记得要将`enp8s0`改为自己的端口名称。



**3.2.1 网线连接**

网线一端插入GO1背部的网口，另一端插入自己电脑网口，然后将自己电脑的IP改为123网段下的地址与机器狗处于同一局域网下。

![](https://s2.loli.net/2024/11/17/ahel9KIxV3rmcsw.png)

这里使用的地址是`192.168.123.162`，大家也可以使用这个，因为刚刚大家在`interfaces`时用到的地址便是这个，两者保持一致就可以了。

现在就可以启动ROS程序控制GO1了。

首先启动ros消息与udp命令转换的程序

```bash
roslaunch unitree_legged_real real.launch ctrl_level:=highlevel
```

除了`highlevel`也可选择`lowlevel`，这个指的是控制频率的高低

然后启动键盘控制程序，这里包含响应键盘事件发布速度的ros消息和ros消息订阅并将消息发送给`real.launch`，`real.launch`接收到消息立即发送给机器狗。

这里用到的按键有`W`、`A`、`S`、`D`、`J`、`L`这几个按键与前面仿真里的用法一样，另外还有按键`Q`用于终止程序，如果是按其他的按键是令机器狗停下。

如果你按下按键却没有响应，可以按照下面的连接解决。

https://github.com/MAVProxyUser/YushuTechUnitreeGo1/blob/main/README.md#ros1-examples

或者直接用我改好的也可以：

```bash
git clone https://github.com/Benxiaogu/unitree_go1.git
```

只需要将`unitree_go1`里面的`unitree_legged_real`文件夹放到你的工作空间里面就可以了。

然后使用`catkin_make`重新编译一下，重新运行上面的两个程序，这时正常的话就可以控制 GO1 前后左右移动和向左向右转动了。
