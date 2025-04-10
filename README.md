[TOC]

## 一、Working with 3D Robot  Modeling in ROS

### 1.1 Creating the ROS package for the robot  description

使用如下命令在工作空间下创建一个ROS功能包

```
$ catkin_create_pkg mastering_ros_robot_description_pkg roscpp tf geometry_msgs urdf rviz xacro 
```

其中`mastering_ros_robot_description`为包名，`roscpp tf`等为功能包需要的依赖

在`mastering_ros_robot_description_pkg`文件夹下创建`urdf`、`meshes`、`launch`文件夹

* **urdf** 文件夹：用于存放即将创建的 URDF/Xacro 文件。

* **meshes** 文件夹：用于存储 URDF 文件中需要引用的三维模型文件（如 STL、DAE 格式）。
* **launch** 文件夹：用于保存 ROS 启动文件（.launch 文件），以便快速启动机器人相关节点。

### 1.2  Creating our first URDF model

首先要设计的机器人机构是一个 **云台机构（Pan and Tilt Mechanism）**，其结构如下图所示。该机构包含 **三个连杆** 和 **两个关节**，具体组成如下：

**1. 连杆（Links）**

1. **基座连杆（Base Link）**
   - **类型**：静态连杆（固定不动的底座）
   - **作用**：作为整个机构的安装基础，所有其他连杆均通过关节连接于此。
2. **水平旋转连杆（Pan Link）**
   - **连接方式**：通过第一个关节安装在基座上。
   - **运动**：可绕垂直轴 **水平旋转（Pan）**。
3. **俯仰连杆（Tilt Link）**
   - **连接方式**：通过第二个关节安装在水平旋转连杆末端。
   - **运动**：可绕水平轴 **俯仰倾斜（Tilt）**。

**2. 关节（Joints）**

1. **水平旋转关节（Pan Joint）**
   - **类型**：旋转关节（Revolute）
   - **运动轴**：绕垂直轴（Z 轴）旋转，实现 **水平扫掠**。
2. **俯仰关节（Tilt Joint）**
   - **类型**：旋转关节（Revolute）
   - **运动轴**：绕水平轴（Y 轴）旋转，实现 **俯仰角度调节**。

在`mastering_ros_robot_description_pkg/urdf`下创建文件`pan_tilt.urdf`

使用如下命令检查`urdf`是否包含错误

```
$ check_urdf pan_tilt.urdf
```

输出如下内容表示一切正常

```
robot name is: pan_tilt
---------- Successfully Parsed XML ---------------
root Link: base_link has 1 child(ren)
    child(1):  pan_link
        child(1):  tilt_link
```

我们还可以使用`urdf_to_graphiz`工具以图形化方式查看机器人连杆和关节结构

```
$ urdf_to_graphiz pan_tilt.urdf
```

这个命令将生成两个文件：`pan_tilt.gv`和`pan_tilt.pdf`

下面是`pan_tilt.pdf`的内容，展示了pan_tilt机械结构的关节和连杆图

<img src="C:\Users\wwn\AppData\Roaming\Typora\typora-user-images\image-20250407193946032.png" alt="image-20250407193946032" style="zoom:33%;" />

### 1.3  Visualizing the robot 3D model in RViz

可以在`Rviz`中查看创建好的URDF模型。在`mastering_ros_robot_description_pkg/launch`目录下创建`view_demo.launch`启动文件

使用如下命令启动模型：

```
$ roslaunch mastering_ros_robot_description_pkg view_demo.launch
```

<img src="C:\Users\wwn\AppData\Roaming\Typora\typora-user-images\image-20250407194916834.png" alt="image-20250407194916834" style="zoom: 25%;" />

添加附加的GUI窗口需要使用`joint_state_publisher_gui`软件包

```xml
<node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" />
```

### 1.4 Adding physical and collision properties  to a URDF model

在对机器人进行仿真之前，我们需要定义在机器人连杆中的物理属性，如几何形状、质量、惯性以及连杆的碰撞属性

```xml
 <link name="tilt_link">
        ......
        <collision>
            <geometry>
            <cylinder length="0.4" radius="0.06"/>
            </geometry>
            <origin rpy="0 1.5 0" xyz="0 0 0"/>
        </collision>
        <inertial>
            <mass value="1"/>
            <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        </inertial>
</link>
```

### 1.5  Understanding robot modeling using  xacro

使用xacro简化URDF，在`mastering_ros_robot_description_pkg/urdf`下创建`pan_tilt.xacro`

之后可以使用如下命令将xacro文件转换为URDF文件

```
$ rosrun xacro xacro pan_tilt.xacro --inorder > pan_tilt_generated.urdf
```

同样我们可以在ROS启动文件中使用下面的命令将xacro转换为URDF，并将其作为`robot_description`的参数

```
<param name="robot_description" command="$(find xacro)/xacro $(find mastering_ros_robot_description_pkg)/urdf/pan_tilt.xacro" />
```

### 1.6  Creating the robot description for a  seven DOF robot manipulator

以下是关节名称和机器人类型的关节列表：

|        关节名        | 关节类型 |           角的范围            |
| :------------------: | :------: | :---------------------------: |
|     bottom_joint     |   固定   |               —               |
|  shoulder_pan_joint  |   转动   | $-150^{\circ}\sim114^{\circ}$ |
| shoulder_pitch_joint |   转动   | $-67^{\circ}\sim109^{\circ}$  |
|   elbow_roll_joint   |   转动   | $-150^{\circ}\sim41^{\circ}$  |
|  elbow_pitch_joint   |   转动   | $-92^{\circ}\sim110^{\circ}$  |
|   wrist_roll_joint   |   转动   | $-150^{\circ}\sim150^{\circ}$ |
|  wrist_pitch_joint   |   转动   |  $92^{\circ}\sim113^{\circ}$  |
|  gripper_roll_joint  |   转动   | $-150^{\circ}\sim150^{\circ}$ |
|    finger_joint1     |   滑动   |          $0\sim3cm$           |
|    finger_joint2     |   滑动   |          $0\sim3cm$           |

根据上述规格设计机械臂的xacro文件，在`mastering_ros_robot_description_pkg/urdf`下创建`seven_dof_arm.xacro`文件，使用RViz查看7-DOF机械臂，使用名为`view_arm.launch`的启动文件执行上述任务

<img src="C:\Users\wwn\AppData\Roaming\Typora\typora-user-images\image-20250407210309771.png" alt="image-20250407210309771" style="zoom: 25%;" />

### 1.7 Creating a robot model for the differential  drive mobile robot

差速轮式机器人在机器人底盘的两端安装两个轮子，整个底盘由一个或两个脚轮支撑

在`mastering_ros_robot_description_pkg/urdf`下创建`diff_wheeled_robot.xacro`文件，可视化结果如下图：

<img src="C:\Users\wwn\AppData\Roaming\Typora\typora-user-images\image-20250407211410951.png" alt="image-20250407211410951" style="zoom:50%;" />

在Rviz中显示为

<img src="C:\Users\wwn\AppData\Roaming\Typora\typora-user-images\image-20250407211623695.png" alt="image-20250407211623695" style="zoom: 25%;" />

## 二 Simulating Robots Using  ROS and Gazebo

### 1.1 The Robotic arm simulation model for Gazebo

创建仿真机械臂所需的软件包`seven_dof_arm_gazebo`：

```
$ catkin_create_pkg seven_dof_arm_gazebo gazebo_msgs gazebo_plugins gazebo_ros mastering_ros_robot_description_pkg
```

在`seven_dof_arm_gazebo/launch`下创建`seven_dof_arm_world.launch`文件

启动该文件会获得如下输出

<img src="C:\Users\wwn\AppData\Roaming\Typora\typora-user-images\image-20250408144933823.png" alt="image-20250408144933823" style="zoom: 25%;" />

#### 1.1.1 Adding colors and textures to the Gazebo robot  model

在`seven_dof_arm.xacro`文件中为机械臂连杆添加颜色

```xml
<gazebo reference="base_link">
        <material>Gazebo/White</material>
</gazebo>
```

#### 1.1.2 Adding transmission tags to actuate the model

为了使用ROS控制器驱动机器人，我们需要定义`<transmission>`标签连接执行机构和关节。

```xml
<xacro:macro name="transmission_block" params="joint_name">
	  <transmission name="tran1">
	    <type>transmission_interface/SimpleTransmission</type>
	    <joint name="${joint_name}">
	      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
	    </joint>
	    <actuator name="motor1">
	      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
	      <mechanicalReduction>1</mechanicalReduction>
	    </actuator>
	  </transmission>
</xacro:macro>
```

#### 1.1.3  Adding the gazebo_ros_control plugin

在仿真模型中添加`gazebo_ros_control`插件来解析传动标签并分配适当的硬件接口和控制管理器

```xml
<!-- ros_control plugin -->
<gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
        <robotNamespace>/seven_dof_arm</robotNamespace>
    </plugin>
</gazebo>
```

#### 1.1.4 Adding a 3D vision sensor to Gazebo

在Gazebo中添加一个名为`Asus Xtion Pro`模型的3D视觉传感器

在`seven_dof_arm.xacro`文件中加入下面内容即可添加`Xtion Pro`物理机器人模型

```xml
<xacro:include filename="$(find mastering_ros_robot_description_pkg)/urdf/sensors/xtion_pro_live.urdf.xacro"/>
```

在`mastering_ros_robot_description_pkg/urdf/sensors/xtion_pro_live.gazebo.xacro`文件中，我们可以设置`Xtion Pro`的Gazebo-ROS插件

```xml
<plugin name="${name}_frame_controller" filename="libgazebo_ros_openni_kinect.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>6.0</updateRate>
    <cameraName>${name}</cameraName>
    <imageTopicName>rgb/image_raw</imageTopicName>
    <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
    ......
</plugin> 
```

### 1.2 Simulating the robotic arm with Xtion Pro

现在启动`seven_dof_arm_world.launch`即可实现完整的仿真

使用名为`image_view`的工具查看3D视觉传感器的图像数据：

查看RGB原始图像：

```
$ rosrun image_view image_view image:=/rgbd_camera/rgb/image_raw
```

<img src="C:\Users\wwn\AppData\Roaming\Typora\typora-user-images\image-20250410121028199.png" alt="image-20250410121028199" style="zoom:33%;" />

查看IR原始图像：

```
$ rosrun image_view image_view image:=/rgbd_camera/ir/image_raw
```

<img src="C:\Users\wwn\AppData\Roaming\Typora\typora-user-images\image-20250410121111339.png" alt="image-20250410121111339" style="zoom:33%;" />

查看深度图像

```
$ rosrun image_view image_view image:=/rgbd_camera/depth/image_raw
```

<img src="C:\Users\wwn\AppData\Roaming\Typora\typora-user-images\image-20250410121232526.png" alt="image-20250410121232526" style="zoom:33%;" />

另外我们还可以在Rviz中查看点云数据

使用如下命令启动Rviz：

```
$ rosrun rviz rviz -f /rgbd_camera_optical_frame
```

<img src="C:\Users\wwn\AppData\Roaming\Typora\typora-user-images\image-20250410121625494.png" alt="image-20250410121625494" style="zoom:33%;" />

### 1.3  Moving robot joints using ROS controllers   in Gazebo

#### 1.3.1 Interfacing joint state controllers and joint position  controllers to the arm

将机器人控制器连接到每个关节，首先为两个控制器编写配置文件。关节状态控制器将发布手臂的关节状态，关节位置控制器可以接收每个关节的目标位置并可以让每个关节运动。在`seven_dof_arm_gazebo/config`下创建配置文件`seven_dof_arm_gazebo_control.yaml`

#### 1.3.2 Moving the robot joints

在`seven_dof_arm_gazebo/launch`下创建启动文件`seven_dof_arm_gazebo_control.launch`文件并启动

使用`std_msgs/Float64`类型的消息将所需的关节值发布到关节位置控制器命令话题上

```
$ rostopic pub /seven_dof_arm/joint4_position_controller/command std_msgs/Float64 1.0
```

<img src="C:\Users\wwn\AppData\Roaming\Typora\typora-user-images\image-20250410122852983.png" alt="image-20250410122852983" style="zoom:50%;" />

### 1.4  Simulating a differential wheeled robot in  Gazebo

对差速轮式机器人仿真，首先创建`diff_wheeled_robot_gazebo`功能包，使用`diff_wheeled_gazebo.launch`启动

<img src="C:\Users\wwn\AppData\Roaming\Typora\typora-user-images\image-20250410123612942.png" alt="image-20250410123612942" style="zoom:50%;" />

#### 1.4.1 Adding the laser scanner to Gazebo

接下来在机器人顶部添加激光雷达。我们使用名称为`libgazebo_ros_laser.so`的GazeboRos插件来仿真激光雷达，完整代码在`diff_wheeled_robot_with_laser.xacro`中

在仿真环境中添加一些物体，我们可以在Rviz中使用LaserScan插件查看相应的激光视图

```
$ roslaunch mastering_ros_robot_description_pkg view_mobile_robot.launch 
```

<img src="C:\Users\wwn\AppData\Roaming\Typora\typora-user-images\image-20250410124221794.png" alt="image-20250410124221794" style="zoom: 25%;" />

#### 1.4.2 Moving the mobile robot in Gazebo

要在Gazebo中控制机器人移动，我们需要添加一个名为`libgazebo_ros_diff_drive.so`的插件，在该插件中我们可以提供一些参数例如机器人的车轮关节、车轮间距等。其中最重要的一个参数是:

```xml
<commandTopic>cmd_vel</commandTopic>
```

该参数是插件的速度指令话题，是ROS中一个Twist类型的消息，我们可以将Twist发布到`/cmd_vel`话题中这样就可以看到机器人移动

#### 1.4.3 Adding the ROS teleop node

ROS遥控节点通过接收键盘的输入来发布ROSTwist命令。在该命令中我们可以生产线速度和角速度。遥控节点在`diff_wheeled_robot_control`功能包中实现。脚本文件`diff_wheeled_robot_key`就是遥控节点

使用命令启动完整仿真设置的Gazebo：

```
$ roslaunch diff_wheeled_robot_gazebo diff_wheeled_gazebo_full.launch
```

启动遥控节点

```
$ roslaunch diff_wheeled_robot_control keyboard_teleop.launch
```

启动Rviz可视化机器人状态与激光数据

```
rosrun rviz rviz
```

<img src="C:\Users\wwn\AppData\Roaming\Typora\typora-user-images\image-20250410125642629.png" alt="image-20250410125642629" style="zoom: 25%;" />

## 三 Using the ROS MoveIt!   and Navigation Stack

### 3.1 Generating MoveIt! configuration  package using Setup Assistant tool

Moveit!配置助手是一个图形化的用户工具，用来将机器人安装配置到Moveit!中

1. 启动配置助手工具

   ```
   roslaunch moveit_setup_assistant setup_assistant.launch
   ```

2. 生成自碰撞矩阵

3. 增加虚拟关节

4. 添加规划组

   创建两个规划组，一个用于机械臂，一个用于夹爪

5. 添加机器人姿态

   添加一些固定的姿态，例如拾取/放置位姿

6. 设置机器人末端执行器

   将夹爪规划组设置为末端执行器用来执行拾取/放置操作

7. 添加被动关节

8. 作者信息

9. 生成配置文件

### 3.2 Motion planning of robot in RViz using  MoveIt! configuration package

Moveit!为Rviz提供了一个插件，可以建立新的规划场景，显示规划的输出结果，还可与机器人进行交互

运行演示启动文件：

```
$ roslaunch seven_dof_arm_config demo.launch
```

使用Rviz运动规划插件

<img src="C:\Users\wwn\AppData\Roaming\Typora\typora-user-images\image-20250410131748431.png" alt="image-20250410131748431" style="zoom:25%;" />

接下来配置Moveit!软件包与Gazebo接口

1. 为Moveit!编写控制器配置文件

   用于与Gazebo中来自与Moveit!的轨迹控制器进行通信。在`seven_dof_arm_config/config`下创建`controller.yaml`配置文件

   并且在`seven_dof_arm_config/launch`下创建名为`seven_dof_arm_moveit_  controller_manager.launch`的启动文件启动轨迹控制器

2. 为Gazebo创建控制器配置文件

   创建`trajectory_control.yaml`文件，该文件包含需要与Gazebo一起加载的GazeboROS控制器列表。同时创建名为`seven_dof_arm_bringup_moveit.launch`的启动文件 

我们可以在Rviz中启动运动规划，然后使用下面的命令在Gazebo中执行规划的路径：


<img src="C:\Users\wwn\AppData\Roaming\Typora\typora-user-images\image-20250410133302521.png" alt="image-20250410133302521" style="zoom:25%;" />

### 3.3  Building a map using SLAM

ROS gmapping是开源SLAM算法

#### 3.3.1 Creating a launch file for gmapping

为gmaaping创建启动文件时的主要任务是为`slam_gmapping`节点和`move_base`节点设置相应参数。`slam_gmapping`节点是ROS gmapping软件包的核心节点，订阅激光数据与tf数据，并将占据栅格地图数据作为输出发布。在`diff_wheeled_robot_gazebo/launch`下创建`gmapping.launch`文件

#### 3.3.2 Running SLAM on the differential drive robot

通过使用Willow Gargage地图开始机器人仿真

```
$ roslaunch diff_wheeled_robot_gazebo diff_wheeled_gazebo_full.launch
```

使用如下命令运行gmapping启动文件

```
 $ roslaunch diff_wheeled_robot_gazebo gmapping.launch
```

使用键盘人工遥控机器人

```
 $ roslaunch diff_wheeled_robot_control keyboard_teleop.launch
```

启动Rviz并添加Map显示类型，设置话题为`/map`

<img src="C:\Users\wwn\AppData\Roaming\Typora\typora-user-images\image-20250410150345180.png" alt="image-20250410150345180" style="zoom:25%;" />

### 3.4  Implementing autonomous navigation using  AMCL and a static map

ROS的amcl软件包提供节点用于在静态地图中对机器人进行定位。

#### 3.4.1 创建amcl启动文件

在`diff_wheeled_robot_gazebo/launch`下创建`amcl.launch`启动文件，使用`map_server`节点，我们可以加载gmapping过程创建的地图

在Gazebo中开始机器人仿真

```
$ roslaunch diff_wheeled_robot_gazebo diff_wheeled_gazebo_full.launch
```

加载amcl启动文件

```
$ roslaunch diff_wheeled_robot_gazebo amcl.launch
```

如果acml工作正常，我们可以使用Rviz控制机器人移动到地图上指定位置。在Rviz中启用LaserScan、Map、Path可视化插件，再使用

2D NavGoal按钮我们就可以控制机器人移动到指定位置

<img src="C:\Users\wwn\AppData\Roaming\Typora\typora-user-images\image-20250410151324135.png" alt="image-20250410151324135" style="zoom:25%;" />

<img src="C:\Users\wwn\AppData\Roaming\Typora\typora-user-images\image-20250410151342595.png" alt="image-20250410151342595" style="zoom:25%;" />
