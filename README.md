# mscnav
### 初衷
- 希望学习组合导航和VIO相关内容.
- 了解到组合导航方面开源代码较少,正好自己在学习相关内容,希望可以和有兴趣的小伙伴们一起学习讨论.

### 程序依赖（目前）
- glog 
- Eigen
- OpenCV 3.4

### 使用说明
最新版本对应为dev分支
mscnav使用了submodules形式挂载了tools,因此clone完本程序需要更新tools

```shell
git checkout -b dev origin/dev
git submodule init
git submodule update
```
已经安装完glog和Eigen条件下,可以直接编译程序
```shell
mkdir build && cd build 
cmake .. && make
```

### 功能
- 支持纯惯导推算
- 初始化功能完成,初步测试认为该功能可满足目前使用.支持给定姿态／动对准／加计调平后对准航向三种方式.（面向低成本IMU器件，不考虑高等级设备可以进行静止航向对准）
- 初步完成松组合定位功能，需要进一步测试，准备整理一些开源数据进行测试，核实程序中公式，符号，单位等易错点
- 视觉前端特征点提取，匹配，外点剔除功能完成（基于OpenCV和ORB-SLAM2）
- 目前程序还在持续开发,后续希望能做好松组合,基于msckf的vio+GPS

### 学习交流
- QQ: 1280269817
- e-mail: fangwentaowhu@outlook.com   wtfang@whu.edu.cn
