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
旧版
- armor_detector：传统视觉装甲板识别旧方案
- rm_camera_driver：大恒相机驱动
- rm_camera_driver_nv12：大恒相机驱动

主线
- hik_camera：工业相机采集。
- rm_armor_detection：装甲板检测主链
- rm_gimbal_bridge：上位机到下位机协议桥接



主线的其他扩展
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





