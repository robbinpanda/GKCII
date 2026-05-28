### 程序（具体调用代码在各可执行文件内修改）
1. 1车2无人机 无人机扫描二维码跟车
./demo0.sh

2. 2车2无人机 无人机扫描二维码跟车
./demo1.sh

3. 1车2无人机 无人机订阅odom跟车
./demo2.sh

4. 2车2无人机 无人机订阅odom跟车
./demo3.sh

5. 1车2无人机 键盘控制
./demo4.sh
### 修改红绿灯数量和位置
需要取消~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/bl-ver6.world文件中的cantilevered_traffic_X_X的注释
