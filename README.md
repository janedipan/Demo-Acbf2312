# 动态避障仿真


依赖安装
---
```bash
nlopt
ipopt
casadi
# 编译过程中报错确实某些msg消息头文件，需要使用catkin_make先编指定package，在恢复默认编译
catkin_make -DCATKIN_WHITELIST_PACKAGES="package1;package2"
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
catkin_make
```

启动
---
稳定版
```bash
source devel/setup.bash
# 选择规划器
roslaunch swarm_test cbf_planning.launch # mpc-dcbf硬约束
roslaunch swarm_test cbf_soft_planning.launch # mpc-dcbf软约束
roslaunch swarm_test kino_planning.launch # voplanner

roslaunch swarm_test start_test.launch
```

数值仿真测试版
```bash
source devel/setup.bash
# 启动全局规划
roslaunch swarm_test acbf_planning1.launch # FSM
# 启动mpc
roslaunch swarM_test start_test.launch
```

Gazebo仿真测试版
```bash
source devel/setup.bash
# 启动Gazebo环境
roslaunch swarm_test start_gazebo_env.launch # 在机器人启动节点那设置 `use_fast_lio_`符号位设置为 `true`-这与fast-lio发布里程相关
# 启动状态估计模块-fast lio
roslaunch swarm_test exp_hardware.launch
# 启动全局规划
# obs_manager部分可以修改感知部分是否使用真值-；
# 不使用真值时，需要将start_gazebo_env.launch中arg /use_sim_time变量的设置为true，以及exp_acbf_planning1.launch的 param /use_sim_time参数的设置为true
roslaunch swarm_test exp_acbf_planning1.launch # FSM
# 启动mpc
roslaunch mpc_dcbf mpc_adsm_c.launch
```


分析单个方法的bag数据
---
```
source devel/setup.bash
roslaunch swarm_test data_process.launch
```


演示多个轨迹
---
```
source devel/setup.bash
roslaunch swarm_test show_all_traj.launch
```
