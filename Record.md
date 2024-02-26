# 修改记录

待完成任务
----------

- [X] 复现MPC-DC，MPC-SCBF，MPC-DCBF方法(在launch文件上留好接口)
- [X] 实现MPC-ACBF方法(涉及障碍物预测轨迹信息)
- [X] 添加路径搜索算法
- [ ] 重新布置障碍物分布
- [ ] 根据MPC参数调整tau值函数参数


时间线
------

1. 24-01-23 修改订阅的障碍物预测轨迹信息 list5->list7
2. 24-01-23 修改订阅的机器人状态信息 list3->list5
3. 24-01-23 修改主CBF函数 关于MPC问题的格式修改-增加controller标志位
4. 24-01-24 复现MPC-DC，MPC-SCBF，MPC-DCBF，MPC-ACBF方法
5. 24-01-24 对MPC预测步数，步长时间的修改同步至mpc_acbf, global_path_publish, obstacle_manager，统一在launch文件参数修改
6. 24-02-25 增加前端全局路径搜索部分 节点文件是global_path_by_adsm.cpp

注意
----

- 修改MPC的预测步数N，预测步长Ts，需要同步修改vomp_planner文件夹中 `obs_manager.hpp`中障碍物预测发布函数，mpc_dcbf文件夹中 `mpc_acbf.py`文件中MPC制定函数；当修改了预测补偿Ts，需要修改mpc_dcbf文件夹中 `global_path_by_rviz.cpp`中step的设置
- 关于前端全局路径搜索参数需要在plan_global_traj.launch文件中修改
