################################ PS2手柄的ROS驱动程序 ####################################
1. 硬件要求：
  a.带USB线的PS2手柄
2. 硬件检查
  a. 将手柄连接电脑；
  b. 检查手柄的usb端口号：
    $ ls /dev/input   #一般游戏手柄端口号为js0
  c. 如无手柄端口，请检查硬件问题。

3. 功能包结构
├── CMakeLists.txt
├── launch
│   └── run_joy_stick.launch
├── package.xml
├── readme.txt
└── scripts
    └── joy_stick_manager.py

4. 软件依赖
  a. Ros功能包： joy
    $ sudo apt-get install ros-noetic-joy

4. 功能包使用
  a. 请直接将功能包放置在ROS工作空间的/src下；
  b. 根据项目需要，自行修改 joy_stick/scipts/joy_stick_manager.py 中的topic名；
  c. 回到ROS工作空间的路径下，编译后即可使用：
    $ source devel/setup.bash
    $ roslaunch joy_stick run_joy_stick.launch

5. TurtleSim示例
  a. 启动本功能包的launch
    $ source devel/setup.bash
    $ roslaunch joy_stick run_joy_stick.launch
  b. 启动TurtleSim
    $ rosrun turtlesim turtlesim_node
  c. 使用遥控器自由的玩耍吧 ！





