# 智能小车轮腿项目代码库

## 一、项目简介


本仓库用于统一管理代码版本，方便小组成员并行开发、功能测试与问题追踪。

推荐参考：
 - [逐飞科技CYT4BB7开源库](https://gitee.com/seekfree/CYT4BB7_Library/tree/master)
 - [逐飞平衡轮腿宣讲](https://www.bilibili.com/video/BV1RY9mYPE4n/?share_source=copy_web&vd_source=0077a6e04e8df0e5e00852cb5880c7e1)
 - [逐飞摄像头图像处理](https://www.bilibili.com/video/BV1tC411H7K8/?share_source=copy_web&vd_source=0077a6e04e8df0e5e00852cb5880c7e1)
---

## 二、硬件平台

* **主控芯片**：CYT4BB7
* **电机类型**：无刷电机 舵机
* **驱动方式**：UART PWM
* **传感器**：
  * IMU：660RA
  * 摄像头：总钻风
* **通信方式**：UART / SPI / I2C / WiFi / 蓝牙
* **供电方式**：锂电池
---

## 三、软件架构说明

代码基于逐飞开源CYT4BB7模板库，主要分为以下几层：

```
├── libraries/      # 系统、逐飞库
    ├──sdk/             # 芯片官方开发库
    ├──zf_common/       # 逐飞通用库
    ├──zf_components/   # 逐飞助手
    ├──zf_device/       # 逐飞外设驱动库
    ├──zf_driver/       # 逐飞驱动
├── project
    ├──code/            # 用户代码放在这里(头文件记得在zf_common_headfile.h中声明)
    ├──iar/             # iar工程文件
    ├──test/            # 测试用
    └──user/            # 程序入口、中断定义
```

---

