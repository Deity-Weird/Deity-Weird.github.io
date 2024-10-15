 ---
 title: github pages + jekyll快速搭建blog
 date: 2023-12-27 12:00:00 +0800
 categories: [Blog, Build]
 tags: [blog]
 ---
### 创建功能工作空间
##### `mkdir catkin_ws:创建文件夹`
##### `cd catkin_ws:进入文件夹`
##### `mkdir src:创建代码空间`
##### `cd ..:返回上一级文件夹`
##### `pwd:读取当前根目录`
##### `catkin_make:编译代码，会产生devel(开发空间),build(编译空间)`
##### `catkin_make insatll:产生安装空间`
##### `cd src:进入src`
##### `catkin_create_pkg test_pkg roscpp rospy std_msgs:创建test_pkg 后面三个是依赖`
##### `source devel/steup.bash:创建环境变量`
##### `echo $ROS_PACKAGE_PATH:查看环境变量`
