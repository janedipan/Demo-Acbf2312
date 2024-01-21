# 动态避障仿真


依赖安装
---
```bash
nlopt
ipopt
casadi
```

启动
---
```python
source devel/setup.bash
# 选择规划器
roslaunch swarm_test cbf_planning.launch # mpc-dcbf硬约束
roslaunch swarm_test cbf_soft_planning.launch # mpc-dcbf软约束
roslaunch swarm_test kino_planning.launch # voplanner

roslaunch swarm_test start_test.launch
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
