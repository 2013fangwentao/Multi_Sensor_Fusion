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
已经安装完glog,Eigen和opencv并且可以找到的条件下,可以直接编译程序
```shell
mkdir build && cd build 
cmake .. && make -j3
```

### 功能
- 松组合功能完善
    - 发布版本链接 [release 1.0.0](https://github.com/2013fangwentao/Multi-Sensor-Combined-Navigation/releases) 使用时注意同时下载对应的tools程序，[链接](https://github.com/2013fangwentao/tools/releases)
    - 对应分支为: [release/loose_couple_gnssins](https://github.com/2013fangwentao/Multi-Sensor-Combined-Navigation/tree/release/loose_couple_gnssins)
- 视觉前端特征点提取，匹配，外点剔除功能完成（基于OpenCV和ORB-SLAM2）
- msckf功能在跟进
- 目前程序还在持续开发,后续希望能做好GPS+IMU+CAMERA的定位功能

### 学习交流
- QQ: 1280269817
- e-mail: fangwentaowhu@outlook.com   wtfang@whu.edu.cn
