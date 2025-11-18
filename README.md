# YDLIDAR 拆机雷达测试程序

这是一个为闲鱼店：**咸鱼改装工作站** 拆机雷达（YDLIDAR）编写的 ROS 测试程序。
<br>plot.py可以直接运行，用于基础测试。

## 🎯 概览

驱动该拆机雷达，并将其数据发布为标准的 ROS `sensor_msgs/LaserScan` 消息，以便在 RVIZ 中进行可视化或用于后续的机器人导航。

由于此雷达使用了 **`150000`** 的非标准波特率，标准的 C++ `libserial` 库在读取时会导致数据损坏(dont know Y)。

为了解决此问题，本程序采用双节点:
1.  **Python 驱动节点 (`lidar_driver.py`)**: 使用 `pyserial`读取串口、同步帧头，并发布**原始**数据包。
2.  **C++ 解析节点 **: 订阅原始数据包，在 C++ 中执行数据解析，并发布最终的 `/scan` 话题。

## ⚙️ 雷达规格

* **品牌**: YDLIDAR 
* **波特率**: **150000**

根据测试，扫描数据的应答包结构如下：

| 字段    | 大小 (Bytes) | 描述                                                  |
| :---- | :--------- | :-------------------------------------------------- |
| `PH`  | 2B         | **包头**：固定为 `0xAA55`（低位在前）。                          |
| `CT`  | 1B         | **包类型**：`CT[bit 0]=0` 表示点云数据；`CT[bit 0]=1` 表示设备启动包。 |
| `LSN` | 1B         | **采样数量**：表示当前数据包中包含的采样点（距离）数量。                      |
| `FSA` | 2B         | **起始角**：本包数据中第一个采样点对应的角度数据。                         |
| `LSA` | 2B         | **结束角**：本包数据中最后一个采样点对应的角度数据。                        |
| `Si`  | 3B * LSN   | **采样数据**：`LSN` 组采样数据，每组 3 字节，第三位为信号强度。              |

## 📐 解析公式

### 距离解析

距离解算公式如下，其中 `S` 为 2 字节的采样数据 `Si`：

$$
Distance_i = \frac{S_i}{4} \quad (\text{单位: mm})
$$

### 角度解析

角度解析分为两级，

#### 一级解析 (初始角度)

一级解析用于计算每个采样点的初始角度（未修正值）。

1.  **起始角 (FSA) 和结束角 (LSA) 解算：**
    $$
    Angle_{FSA} = \frac{\text{Rshiftbit}(FSA, 1)}{64.0}
    $$
    $$
    Angle_{LSA} = \frac{\text{Rshiftbit}(LSA, 1)}{64.0}
    $$
    *注：`Rshiftbit(data, 1)` 表示将数据右移 1 位。*

2.  **中间角 (Angle_i) 解算：**
    $$
    \text{diff}(Angle) = Angle_{LSA} - Angle_{FSA}
    $$
    $$
    Angle_i = \frac{\text{diff}(Angle)}{LSN - 1} \times (i - 1) + Angle_{FSA} \quad (i = 1, 2, \ldots, LSN)
    $$

#### 二级解析 (角度修正)

二级解析用于根据距离修正每个采样点的角度，以补偿 Lidar 旋转带来的延迟。

1.  **计算角度修正值 (`AngCorrect_i`)：**
    * `IF Distance_i == 0:`
        $$
        AngCorrect_i = 0
        $$
    * `ELSE:`
        $$
        AngCorrect_i = \text{tand}^{-1}\left( \frac{21.8 \times (155.3 - Distance_i)}{155.3 \times Distance_i} \right)
        $$

2.  **计算最终角度：**
    $$
    \text{Final\_Angle}_i = Angle_i + AngCorrect_i
    $$

## 💻 本测试程序架构 (noetic)

* 本测试无角度修正
* **`lidar_driver.py` (Python 驱动)**，由gemini从plot.py修改而来。
    * **职责**: 使用 `pyserial` 在 `150000` 波特率下安全读取串口。
    * **发布**: ` /lidar/raw_packet` (`std_msgs/UInt8MultiArray`)。
* **`lidar_parser.cpp` (C++ 解析器)**
    * **职责**: 订阅原始数据包，在 C++ 中高效执行上述解析公式。
    * **发布**: `/scan` (`sensor_msgs/LaserScan`)。
