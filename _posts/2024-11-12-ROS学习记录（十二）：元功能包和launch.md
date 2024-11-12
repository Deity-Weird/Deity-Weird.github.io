---
title: 2024-11-12-ROS学习记录（十二）：元功能包和launch
date: 2024-11-12
categories: [ros,course]
tags: [ros,course]
---

## 元功能包和launch

### 一、元功能包

#### 概念

__MetaPackage__ 是 __Linux__ 的一个文件管理系统的概念。是 __ROS__ 中的一个虚包，里面没有实质性的内容，但是它依赖了其他的软件包，通过这种方法可以把其他包组合起来，我们可以认为它是一本书的目录索引，告诉我们这个包集合中有哪些子包，并且该去哪里下载。

__例如__：
```
    sudo apt install ros-noetic-desktop-full 命令安装ros时就使用了元功能包，该元功能包依赖于ROS中的其他一些功能包，安装该包时会一并安装依赖。
```
还有一些常见的 __MetaPackage：navigation moveit! turtlebot3 ....__
__作用__:
方便用户的安装，我们只需要这一个包就可以把其他相关的软件包组织到一起安装了。

__实现__
新建一个功能包,修改 __package.xml__ 
```xml
 <exec_depend>被集成的功能包</exec_depend>
 .....
 <export>
   <metapackage />
 </export>


<!--下面就是一个例子--------------------------------------->


<?xml version="1.0"?>
<package format="2">
  <name>test_my</name>
  <version>0.0.0</version>
  <description>The test_my package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <maintainer email="deity@todo.todo">deity</maintainer>


  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>TODO</license>


  <!-- Url tags are optional, but multiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://wiki.ros.org/test_my</url> -->


  <!-- Author tags are optional, multiple are allowed, one per tag -->
  <!-- Authors do not have to be maintainers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->


  <!-- The *depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
  <!--   <depend>roscpp</depend> -->
  <!--   Note that this is equivalent to the following: -->
  <!--   <build_depend>roscpp</build_depend> -->
  <!--   <exec_depend>roscpp</exec_depend> -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use build_export_depend for packages you need in order to build against this package: -->
  <!--   <build_export_depend>message_generation</build_export_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use exec_depend for packages you need at runtime: -->
  <!--   <exec_depend>message_runtime</exec_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <!-- Use doc_depend for packages you need only for building documentation: -->
  <!--   <doc_depend>doxygen</doc_depend> -->
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>test_pubsub</exec_depend>
  <exec_depend>test_service</exec_depend>
  <exec_depend>test_param</exec_depend>



  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->
    <metapackage /> 
  </export>
</package>
```


修改 __CMakeLists.txt__ ，固定格式，不要有换行
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(demo)
find_package(catkin REQUIRED)
catkin_metapackage()
```


### 二、launch文件


#### 1.launch标签

##### 1.属性
```xml
deprecated = "弃用声明"
```
告知用户当前 launch 文件已经弃用
##### 2.子级标签
所有其它标签都是launch的子级

##### 3.例
```xml
<node name="turtle1" pkg="turtlesim" type="turtlesim_node" output="screen"/>
<node name="key" pkg="turtlesim" type="turtle_teleop_key" output="screen"/>
```

#### 2.node标签
##### 1.属性
```cpp
pkg="包名"                 //节点所属的包
type="nodeType"            //节点类型(与之相同名称的可执行文件)
name="nodeName"            //节点名称(在 ROS 网络拓扑中节点的名称) 自定义
args="xxx xxx xxx"         //(可选)  将参数传递给节点
machine="机器名"           //在指定机器上启动节点
respawn="true | false"    //(可选)如果节点退出，是否自动重启
respawn_delay=" N"        //(可选)如果 respawn 为 true, 那么延迟 N 秒后启动节点
required="true | false"   //(可选)该节点是否必须，如果为 true,那么如果该节点退出，将杀死整个 roslaunch
ns="xxx"                  //(可选)在指定命名空间 xxx 中启动节点，即给该节点添加一个上级目录，防止重名
clear_params="true | false"//(可选)在启动前，删除节点的私有空间的所有参数，慎用!!!!!!!
output="log | screen"     //(可选)日志发送目标，可以设置为 log 日志文件，或 screen 屏幕,默认是 log
```
##### 2.子级标签
```cpp
env      //环境变量设置
remap    //重映射节点名称
rosparam //参数设置
param    //参数设置
```

##### 3.例

```xml
<!-- <launch deprecated = "弃用声明"> -->
<launch>
    <node name="turtle1" pkg="turtlesim" type="turtlesim_node" output="screen"/>
    <node name="key" pkg="turtlesim" type="turtle_teleop_key" output="screen"/>
</launch>
```


#### 3.include
复用launch文件
##### 1.属性

```cpp
file="$(find 包名)/xxx/xxx.launch" //要包含的文件路径
ns="xxx"                          //(可选)在指定命名空间导入文件
```

##### 2.子级标签
```cpp
env //环境变量设置
arg //将参数传递给被包含的文件
```

##### 3.例
```xml
<launch>
    <include file="$(find test_turtle)/launch/start_turtle.launch" />
    <!-- 空格/别忘了 -->
</launch>
```

#### 4.remap
用于话题重命名
##### 1.属性
```cpp
from="xxx"//原始话题名称
to="yyy"  //目标名称
```

##### 2.子级标签
无

##### 3.例
将乌龟的控制节点改为 __teleop_twist_keyboard__
###### 安装teleop_twist_keyboard

```
sudo apt-get install ros-noetic-teleop-twist-keyboard
```

###### 设置launch文件
```xml
<!-- <launch deprecated = "弃用声明"> -->
<launch>
    <node name="turtle1" pkg="turtlesim" type="turtlesim_node" output="screen">
        <remap from="/turtle1/cmd_vel" to="/cmd_vel" />  
    </node>
    <!-- /node才是结束 -->
    <node name="key" pkg="turtlesim" type="turtle_teleop_key" output="screen"/>
</launch>
```

###### 运行
```
roslaunch test_turtle start_turtle.launch 
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

#### 5.param
__param__ 标签主要用于在参数服务器上设置参数，参数源可以在标签中通过 __value__ 指定，也可以通过外部文件加载，在 __node__ 标签中时，相当于私有命名空间。
##### 1.属性
```cpp
name="命名空间/参数名"
参数名称，可以包含命名空间
value="xxx" (可选)
定义参数值，如果此处省略，必须指定外部文件作为参数源
type="str | int | double | bool | yaml" (可选)
指定参数类型，如果未指定，roslaunch 会尝试确定参数类型，规则如下:
如果包含 '.' 的数字解析未浮点型，否则为整型
"true" 和 "false" 是 bool 值(不区分大小写)
其他是字符串
```

##### 2.子级标签
无

##### 3.例

```xml
<!-- <launch deprecated = "弃用声明"> -->
<launch>
    <!-- 格式1：与node平级 -->
    <param name="param_A" type="int" value="100"/>
    <node name="turtle1" pkg="turtlesim" type="turtlesim_node" output="screen">
        <remap from="/turtle1/cmd_vel" to="/cmd_vel" /> 
        <!--格式2：node的子级  -->
        <param name="param_B" type="double" value="3.14"/>
    </node>
    <node name="key" pkg="turtlesim" type="turtle_teleop_key" output="screen"/>
</launch>
```

#### 6.rosparam

__rosparam__ 标签可以从 __YAML__ 文件导入参数，或将参数导出到 __YAML__ 文件，也可以用来删除参数，__rosparam__ 标签在 __node__ 标签中时被视为私有。
##### 1.属性
```cpp
command="load | dump | delete" (可选，默认 load)
加载、导出或删除参数
file="$(find xxxxx)/xxx/yyy...."
加载或导出到的 yaml 文件
param="参数名称"
ns="命名空间" (可选)
```
##### 2.子级标签
无

##### 3.例

```xml
<!-- <launch deprecated = "弃用声明"> -->
<launch>
    <!-- 格式1：与node平级 -->
    <param name="param_A" type="int" value="100"/>

    <!-- 格式1：与node平级 -->
    <rosparam command="load" file="$(find test_turtle)/launch/params.yaml"/>
    <!-- 导出参数 -->
    <rosparam command="dump" file="$(find test_turtle)/launch/params_out.yaml"/>
    
    <node name="turtle1" pkg="turtlesim" type="turtlesim_node" output="screen">
        <remap from="/turtle1/cmd_vel" to="/cmd_vel" /> 
        <!--格式2：node的子级  -->
        <param name="param_B" type="double" value="3.14"/>

        <!--格式2：node的子级  -->
        <rosparam command="load" file="$(find test_turtle)/launch/params.yaml"/>
    </node>
    <node name="key" pkg="turtlesim" type="turtle_teleop_key" output="screen"/>
</launch>
```
在进行导出操作和删除操作的时候建议用另外一个launch文件
```xml
<launch>
    <!-- 导出参数 -->
    <rosparam command="dump" file="$(find test_turtle)/launch/params_out.yaml"/>
    <!-- 删除参数 -->
    <rosparam command="delete" param="bg_B"/>
</launch>
```

#### 7.group

__group__ 标签可以对节点分组，具有 __ns__ 属性，可以让节点归属某个命名空间
##### 1.属性
```cpp
ns="名称空间" (可选)
clear_params="true | false" (可选)
启动前，是否删除组名称空间的所有参数(慎用....此功能危险)
```
##### 2.子级标签
除了launch 标签外的其他标签

##### 3.例
```xml
<?xml version="1.0"?>
<launch>

    <group ns="frist">
        <node name="turtle1" pkg="turtlesim" type="turtlesim_node" output="screen"/>
        <node name="key" pkg="turtlesim" type="turtle_teleop_key" output="screen"/>
    </group>

    <group ns="second">
        <node name="turtle1" pkg="turtlesim" type="turtlesim_node" output="screen"/>
        <node name="key" pkg="turtlesim" type="turtle_teleop_key" output="screen"/>        
    </group>
</launch>
```

#### 8.arg

__arg__ 标签是用于动态传参，类似于函数的参数，可以增强 __launch__ 文件的灵活性
##### 1.属性
```cpp
name="参数名称"
default="默认值" (可选)
value="数值" (可选)
不可以与 default 并存
doc="描述"
参数说明
```

##### 2.子级标签

    无

##### 3.例
```xml
<?xml version="1.0"?>
<launch>

    <!-- 多个参数用同一个值 -->
    <arg name="car_length" default="1.2" doc="小车长度"/>

    <param name="A" value="$(arg car_length)"/>
    <param name="B" value="$(arg car_length)"/>
    <param name="C" value="$(arg car_length)"/>
</launch>
```