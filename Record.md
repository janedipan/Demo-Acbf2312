# 修改记录

待完成任务
----------

- [X] 复现MPC-DC，MPC-SCBF，MPC-DCBF方法(在launch文件上留好接口)
- [X] 实现MPC-ACBF方法(涉及障碍物预测轨迹信息)
- [X] 添加路径搜索算法
- [X] 重新布置障碍物分布
- [X] 根据MPC参数调整tau值函数参数
- [X] 播放bag的可视化脚本launch文件：机器人odometry, mesh, 障碍物的mesh

时间线
------

1. 24-01-23 修改订阅的障碍物预测轨迹信息 list5->list7
2. 24-01-23 修改订阅的机器人状态信息 list3->list5
3. 24-01-23 修改主CBF函数 关于MPC问题的格式修改-增加controller标志位
4. 24-01-24 复现MPC-DC，MPC-SCBF，MPC-DCBF，MPC-ACBF方法
5. 24-01-24 对MPC预测步数，步长时间的修改同步至mpc_acbf, global_path_publish, obstacle_manager，统一在launch文件参数修改
6. 24-02-25 增加前端全局路径搜索部分 节点文件是global_path_by_adsm.cpp
7. 24-03-15 修改障碍物的运动形式-修改为匀速往返运动形式
8. 24-03-19 增加MPC-ACBF软约束的约束形式
9. 24-04-01 完成 `obstacles_param.yaml`中6个场景的障碍物设置
10. 24-04-10 完成6个场景的数据收集

注意
----

- 修改MPC的预测步数N，预测步长Ts，需要同步修改vomp_planner文件夹中 `obs_manager.hpp`中障碍物预测发布函数，mpc_dcbf文件夹中 `mpc_acbf.py`文件中MPC制定函数；当修改了预测补偿Ts，需要修改mpc_dcbf文件夹中 `global_path_by_rviz.cpp`中step的设置
- 关于前端全局路径搜索参数需要在plan_global_traj.launch文件中修改
- 切换前端路径搜索-安全约束的参数设置在 `plan_global_traj.launch`的文件中，同时路径搜索的主要函数在 `theta_astar.cpp`文件中
- `obs_manager.hpp`文件在traj_planner, swarm_test中都有存在，前者用来参数化表示障碍物轨迹信息，后者在data_processor中实时记录障碍物的信息
- 测试使用到的文件是 `acbf_planning.launch`以及 `start_test.launch`，在前者主要设置 `controller`，`front_dist`，`front_adsm`两个参数用于切换不同的方法，后者设置 `/data/record_sign`用于切换是否启动数据记录
- 录制完bag后，需要使用launch文件启动播bag指定话题，在 `show_mpc_traj.launch文件中`
- 注意对不同的方法需要使用不同的颜色展示轨迹，在 `acbf_planning.launch`文件的 `odom_traj_visualization_node`节点进行颜色配置

---
