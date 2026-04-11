# 上位机

## 文件传输

1. 网线直连后先配置同网段静态 IP
2. 上位机在设置当中配置：eth0 = 192.168.127.11/24
3. RDK-X5 示例配置：eth0 = 192.168.127.10/24（已经配置完毕）
4. 连通性检查：上位机 ping 192.168.127.10
5. SSH 检查：可登录 sunrise@192.168.127.10（密码sunrise）
6. 代码传输命令（在 dev-branch/scripts 下执行）：
	RDK_HOST=192.168.127.10 bash deploy_to_rdk_x5.sh
    该脚本行为：
	- 在远端创建 /home/sunrise/rm_ws/src
	- 用 rsync 同步 dev-branch 到远端 src
	- 排除 build/install/log
7. 在终端上使用ssh登录上位机 （密码sunrise）
8. 进入rm_ws进行编译

## 功能包分析


各包定位：
### 旧版
- armor_detector：传统视觉装甲板识别旧方案
- rm_camera_driver：大恒相机驱动
- rm_camera_driver_nv12：大恒相机驱动

### 主线
#### - hik_camera：工业相机采集。
```bash
ros2 launch hik_camera hik_camera.launch.py #自启动同
```
##### 启动的节点
/hik_camera
/hik_camera
  Subscribers:
                                                    //没有订阅话题
  Publishers:
    /camera_info: sensor_msgs/msg/CameraInfo        //图像信息
    /hbmem_img: hbm_img_msgs/msg/HbmMsg1080P        //hbmem 共享内存图像
    /image_raw: sensor_msgs/msg/Image               //标准 ROS 图像



  Service Servers:
    /hik_camera/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /hik_camera/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /hik_camera/get_parameters: rcl_interfaces/srv/GetParameters
    /hik_camera/list_parameters: rcl_interfaces/srv/ListParameters
    /hik_camera/set_parameters: rcl_interfaces/srv/SetParameters
    /hik_camera/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /set_camera_info: sensor_msgs/srv/SetCameraInfo


#### - rm_armor_detection：装甲板检测主链
运行检测节点
```bash
ros2 launch rm_armor_detection rm_armor_detection.launch.py 
# 自启动运行的指令
ros2 run rm_armor_detection rm_armor_detection #自启动 
#
```
##### 启动的节点
/dnn_node_sample
/dnn_node_sample
  Subscribers:
    /hbmem_img: hbm_img_msgs/msg/HbmMsg1080P            //订阅了内存共享图像

  Publishers:
    /dnn_node_sample: ai_msgs/msg/PerceptionTargets     //输出的目标

摄像头获取数据->触发回调函数（FeedImg）->预处理（得到input）->推演（Run）->推理结果回调（PostProcess）
->解析模型输出结果（parse）->发布ROS话题




  Service Servers:
    /dnn_node_sample/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /dnn_node_sample/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /dnn_node_sample/get_parameters: rcl_interfaces/srv/GetParameters
    /dnn_node_sample/list_parameters: rcl_interfaces/srv/ListParameters
    /dnn_node_sample/set_parameters: rcl_interfaces/srv/SetParameters
    /dnn_node_sample/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically



如果需要网页显示：
```bash
export WEB_SHOW=TRUE
ros2 launch rm_armor_detection rm_armor_detection.launch.py

```

运行桌面可视化窗口
在 RDK-X5 图形桌面中执行：
```bash
source /opt/tros/humble/setup.bash
source install/setup.bash
export DISPLAY=:0
export XAUTHORITY=/home/sunrise/.Xauthority
ros2 run rm_armor_detection rm_armor_detection_visualizer #自启动同
```
    该窗口会：
    - 订阅 `/image_raw`
    - 订阅 `/dnn_node_sample`
    - 在图上绘制检测框、类别、置信度和关键点
##### 启动的节点
/rm_autoaim_visualizer
/rm_autoaim_visualizer
  Subscribers:
    /dnn_node_sample: ai_msgs/msg/PerceptionTargets
    /image_raw: sensor_msgs/msg/Image
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /rm_autoaim_visualizer/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /rm_autoaim_visualizer/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /rm_autoaim_visualizer/get_parameters: rcl_interfaces/srv/GetParameters
    /rm_autoaim_visualizer/list_parameters: rcl_interfaces/srv/ListParameters
    /rm_autoaim_visualizer/set_parameters: rcl_interfaces/srv/SetParameters
    /rm_autoaim_visualizer/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically



#### - rm_gimbal_bridge：上位机到下位机协议桥接
运行
```bash
ros2 launch rm_gimbal_bridge rm_gimbal_bridge.launch.py
# 自启动运行的指令
ros2 run rm_gimbal_bridge rm_gimbal_bridge_node #自启动（同）
#
```
##### 启动的节点
/rm_gimbal_bridge
/rm_gimbal_bridge
  Subscribers:
    /dnn_node_sample: ai_msgs/msg/PerceptionTargets  //模型推演之后经过处理的结果

  Publishers:
                                                    //没有发布话题





                                                    
  Service Servers:
    /rm_gimbal_bridge/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /rm_gimbal_bridge/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /rm_gimbal_bridge/get_parameters: rcl_interfaces/srv/GetParameters
    /rm_gimbal_bridge/list_parameters: rcl_interfaces/srv/ListParameters
    /rm_gimbal_bridge/set_parameters: rcl_interfaces/srv/SetParameters
    /rm_gimbal_bridge/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically


整机启动
```bash
ros2 launch rm_gimbal_bridge rm_autoaim_system.launch.py
```

示例：
```bash
ros2 launch rm_gimbal_bridge rm_autoaim_system.launch.py serial_port:=/dev/ttyS1 enemy_prefix:=blue_
```


### 主线的其他扩展
- rm_interfaces：消息与服务定义

- rm_utils：公共工具库
    - 扩展卡尔曼滤波器
    - PnP 解算器
    - 弹道补偿器
    - 日志封装
    - URL 路径解析
    - 心跳发布器
- scripts：部署、tmux、服务启动脚本

第三方
- ultralytics-8.2.103：YOLO源码


# 下位机
## 编译烧录

已验证流程（STM32F407，ST-Link SWD）：

1. 工程目录：Gimbal control
2. 编译：
	- make clean
	- make -j$(nproc)
3. 产物：
	- build/Chassis_ins.elf
	- build/Chassis_ins.hex
	- build/Chassis_ins.bin
4. 烧录：
	STM32_Programmer_CLI -c port=SWD -w build/Chassis_ins.hex -v -rst
5. 成功判据：
	- Download in Progress 100%
	- Download verified successfully
	- MCU Reset

环境说明：
- 本机已存在 STM32CubeProgrammer CLI，可通过 STM32_Programmer_CLI 调用。
- 若出现 USB 权限问题，需用 sudo 或配置 udev 规则。

## 文件夹分析

下位机核心目录（简要）：
- Src：主逻辑与任务调度入口。
- Chassis：底盘/云台相关控制逻辑。
- IMU：传感器驱动与姿态相关实现。
- algorithm：PID、滤波、数学与通用算法。
- USB_DEVICE：USB CDC 设备栈。
- Drivers/Middlewares：HAL、CMSIS、FreeRTOS、USB 库。


## task解析





