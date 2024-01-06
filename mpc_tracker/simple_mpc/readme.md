# Simple_MPC
基于运动学的非线性MPC，实现包括基于阿克曼小车、差速小车和全向移动机器人的轨迹跟踪控制器，仅需提供机器人定位和参考轨迹即可输出控制指令。

优化求解器采用casadi，可自由添加其他非线性约束。基于c++多态实现不同机器人的mpc，可根据需要自行扩展。

## 1. 依赖安装
源码安装casadi求解器
```bash
mkdir ~installation && cd ~installation
git clone https://github.com/casadi
mkdir build && cd build
cmake .. -DWITH_IPOPT=ON -DWITH_EXAMPLES=OFF
make -j8
sudo make install
```

## 2. 编译使用
### (1) 下载编译
```bash
cd catkin_ws/src
git clone https://github.com/WX-James/simple_mpc
cd .. && cakin_make
```

### (2) 控制参数请在/simple_mpc/config文件夹中对应调整

### (3) 话题调整
| 参数                  | 说明                                                         |
| --------------------- | ------------------------------------------------------------ |
| arg name="odom_topic" | 机器人定位Topic, 消息类型: nav_msgs/Odometry                   |
| arg name="traj_topic" | 参考轨迹Topic，消息类型: std_msgs/Float32MultiArray  |
| arg name="cmd_topic"  | 控制指令Topic，消息类型: geometry_msgs/Twist                  |

### (4) 启动MPC轨迹跟踪控制

阿克曼小车mpc
```bash
source devel/setup.bash
roslaunch mpc mpc_ackman.launch
```

差速小车mpc
```bash
source devel/setup.bash
roslaunch mpc mpc_differential.launch
```

全向小车或四足机器人mpc
```bash
source devel/setup.bash
roslaunch mpc mpc_quadruped.launch
```

## 3. 机器人运动学
### 阿克曼小车
基于二自由度自行车模型，控制输入为($v_x$, $\delta$),系统状态方程:

$$
\begin{bmatrix} \dot{p_x} \\
                \dot{p_y} \\
                \dot{\theta}
\end{bmatrix} = 
\begin{bmatrix} \cos{\theta} \\
                \sin{\theta} \\
                \frac{\tan{\delta}}{L}
\end{bmatrix} v_x
$$
### 差速小车
基于两轮差速小车模型，控制输入为($v_x$, $\omega$)，系统状态方程:

$$
\begin{bmatrix} \dot{p_x} \\
                \dot{p_y} \\
                \dot{\theta} 
\end{bmatrix} = 
\begin{bmatrix} v_x \cos{\theta} \\
                v_x \sin{\theta} \\
                \omega 
\end{bmatrix} 
$$

### 全向机器人
控制输入为($v_x$, $v_y$, $\omega$)，系统状态方程：

$$
\begin{bmatrix} \dot{p_x} \\
                \dot{p_y} \\
                \dot{\theta} 
\end{bmatrix} = 
\begin{bmatrix} v_x \cos{\theta} - v_y \sin{\theta} \\
                v_x \sin{\theta} + v_y \cos{\theta} \\
                \omega 
\end{bmatrix} 
$$
