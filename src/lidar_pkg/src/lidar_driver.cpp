#include <algorithm>
#include <boost/thread.hpp>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <serial/serial.h> // 使用 libserial 作为串口库
#include <sstream>
#include <vector>

// Python脚本中的魔数
#define PORT_NAME "/dev/ttyUSB0"
#define BAUDRATE 150000

class LidarDriver
{
  public:
    // 构造函数：初始化ROS节点句柄和发布器
    LidarDriver (ros::NodeHandle &nh) : nh_ (nh), is_running_ (false)
    {
        // 1. ROS 发布器 (Publisher)
        // 发布到 /scan 话题，类型为 sensor_msgs::LaserScan
        pub_ = nh_.advertise<sensor_msgs::LaserScan> ("scan", 10);

        // 2. 参数加载 (从ROS参数服务器加载，以支持配置)
        std::string port_name = PORT_NAME;
        int baud_rate = BAUDRATE;
        nh_.param ("port", port_name, port_name);
        nh_.param ("baud_rate", baud_rate, baud_rate);
        nh_.param ("frame_id", frame_id_, std::string ("laser_frame"));

        // 3. 串口初始化
        try
            {
                ser_.setPort (port_name);
                ser_.setBaudrate (baud_rate);
                serial::Timeout to
                    = serial::Timeout::simpleTimeout (1000); // 1秒超时
                ser_.setTimeout (to);
                ser_.open ();
            }
        catch (const serial::IOException &e)
            {
                ROS_FATAL_STREAM ("Cannot open serial port: "
                                  << port_name << " Error: " << e.what ());
                ros::shutdown (); // 严重错误，关闭节点
            }

        if (ser_.isOpen ())
            {
                ROS_INFO_STREAM ("Serial port " << port_name
                                                << " opened successfully at "
                                                << baud_rate);
            }
    }

    // 析构函数：确保线程和串口被正确关闭
    ~LidarDriver () { stop (); }

    // 启动 Lidar 数据读取线程
    void
    start ()
    {
        if (ser_.isOpen () && !is_running_)
            {
                is_running_ = true;
                // 使用 Boost 线程运行串口读取循环
                loop_thread_
                    = boost::thread (&LidarDriver::serialReadLoop, this);
                ROS_INFO ("Lidar read thread started.");
            }
    }

    // 停止 Lidar 数据读取线程
    void
    stop ()
    {
        if (is_running_)
            {
                is_running_ = false;
                if (loop_thread_.joinable ())
                    {
                        loop_thread_.join (); // 等待线程退出
                    }
                if (ser_.isOpen ())
                    {
                        ser_.close ();
                        ROS_INFO ("Serial port closed.");
                    }
                ROS_INFO ("Lidar driver stopped.");
            }
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    serial::Serial ser_;
    boost::thread loop_thread_;
    bool is_running_;
    std::string frame_id_;

    // 结构体用于存储解析出的单个点数据
    struct LidarPoint
    {
        double angle_rad;
        double distance_mm;
    };

    /**
     * @brief 对应Python中的 parse_packet 函数
     * @param packet_data 完整的原始数据包 (AA 55 ...)
     * @return 解析出的点列表
     */
    std::vector<LidarPoint>
    parsePacket (const std::vector<uint8_t> &packet_data)
    {
        // 确保数据包长度足够
        if (packet_data.size () < 8)
            return {};

        // 数据包头部结构：AA 55 CT LSN Fs Ls ...
        uint8_t lsn = packet_data[3]; // LSN: 采样点数量

        // C++中处理原始字节需要注意大小端和对齐问题。
        // 使用 struct.unpack('<H', ...) 对应 Little Endian (小端) unsigned
        // short 4:5 是 Fs_low/high, 6:7 是 Ls_low/high

        uint16_t fsangle_raw = (packet_data[4] | (packet_data[5] << 8));
        uint16_t lsangle_raw = (packet_data[6] | (packet_data[7] << 8));

        // 对应 Python 逻辑：(raw >> 1) / 64.0
        double angle_start_deg = (double)(fsangle_raw >> 1) / 64.0;
        double angle_end_deg = (double)(lsangle_raw >> 1) / 64.0;

        double diff_angle_deg = 0.0;
        if (lsn > 1)
            {
                diff_angle_deg = angle_end_deg - angle_start_deg;
                // 确保角度差在 [0, 360) 范围内
                if (diff_angle_deg < 0)
                    diff_angle_deg += 360.0;
            }

        std::vector<LidarPoint> points;
        points.reserve (lsn);

        for (int i = 0; i < lsn; ++i)
            {
                size_t offset = 8 + i * 3; // 8字节头部 + i * 3字节点数据
                if (offset + 2 >= packet_data.size ())
                    break;

                // 读取距离 (2字节 Little Endian unsigned short)
                uint16_t dist_raw
                    = (packet_data[offset] | (packet_data[offset + 1] << 8));
                // 对应 Python 逻辑：dist_raw / 4.0
                double distance_mm = (double)dist_raw / 4.0;

                double angle_deg = angle_start_deg;
                if (lsn > 1)
                    {
                        // 对应 Python
                        // 逻辑：(diff_angle_deg/(lsn-1))*i+angle_start_deg
                        angle_deg = (diff_angle_deg / (lsn - 1)) * i
                                    + angle_start_deg;
                    }

                // 确保角度在 [0, 360) 范围内
                angle_deg = std::fmod (angle_deg, 360.0);
                if (angle_deg < 0)
                    angle_deg += 360.0;

                if (distance_mm > 0.0)
                    {
                        LidarPoint p;
                        p.angle_rad = M_PI * angle_deg / 180.0; // 转换为弧度
                        p.distance_mm = distance_mm;
                        points.push_back (p);
                    }
            }

        return points;
    }

    /**
     * @brief 对应Python中的 _read_serial_loop 函数
     * 运行在独立线程中，负责串口同步和数据包读取
     */
    void
    serialReadLoop ()
    {
        // 串口同步状态机
        int sync_state = 0;
        std::vector<uint8_t> buffer;

        while (ros::ok () && is_running_)
            {
                try
                    {
                        if (sync_state == 0)
                            {
                                uint8_t byte1;
                                if (ser_.read (&byte1, 1) == 1)
                                    {
                                        if (byte1 == 0xAA)
                                            {
                                                sync_state = 1;
                                            }
                                    }
                            }
                        else if (sync_state == 1)
                            {
                                uint8_t byte2;
                                if (ser_.read (&byte2, 1) == 1)
                                    {
                                        if (byte2 == 0x55)
                                            {
                                                // 头部剩余部分: CT (1 byte) +
                                                // LSN (1 byte)
                                                std::vector<uint8_t>
                                                    header_rest (2);
                                                if (ser_.read (header_rest, 2)
                                                    == 2)
                                                    {
                                                        uint8_t lsn = header_rest
                                                            [1]; // LSN:
                                                                 // 采样点数量
                                                        size_t remaining_bytes
                                                            = 4
                                                              + (lsn
                                                                 * 3); // 4
                                                                       // bytes
                                                                       // (Fs,
                                                                       // Ls) +
                                                                       // LSN *
                                                                       // 3
                                                                       // bytes
                                                                       // (points)

                                                        std::vector<uint8_t>
                                                            packet_payload (
                                                                remaining_bytes);
                                                        if (ser_.read (
                                                                packet_payload,
                                                                remaining_bytes)
                                                            == remaining_bytes)
                                                            {
                                                                // 完整数据包：AA
                                                                // 55 +
                                                                // header_rest
                                                                // +
                                                                // packet_payload
                                                                std::vector<
                                                                    uint8_t>
                                                                    full_packet;
                                                                full_packet
                                                                    .push_back (
                                                                        0xAA);
                                                                full_packet
                                                                    .push_back (
                                                                        0x55);
                                                                full_packet.insert (
                                                                    full_packet
                                                                        .end (),
                                                                    header_rest
                                                                        .begin (),
                                                                    header_rest
                                                                        .end ());
                                                                full_packet.insert (
                                                                    full_packet
                                                                        .end (),
                                                                    packet_payload
                                                                        .begin (),
                                                                    packet_payload
                                                                        .end ());

                                                                // 解析数据包
                                                                std::vector<
                                                                    LidarPoint>
                                                                    points
                                                                    = parsePacket (
                                                                        full_packet);
                                                                if (!points
                                                                         .empty ())
                                                                    {
                                                                        // **关键：直接发布，而不是像Python那样缓存**
                                                                        // ROS节点应该尽快发布数据
                                                                        publishScan (
                                                                            points);
                                                                    }
                                                            }
                                                    }
                                                sync_state
                                                    = 0; // 无论成功与否，重置状态
                                            }
                                        else if (byte2 == 0xAA)
                                            {
                                                sync_state = 1; // 再次同步字节
                                            }
                                        else
                                            {
                                                sync_state = 0; // 同步失败
                                            }
                                    }
                            }
                    }
                catch (const serial::IOException &e)
                    {
                        if (ros::ok ())
                            { // 仅在ROS未关闭时打印错误
                                ROS_ERROR_STREAM (
                                    "Serial read error: " << e.what ());
                            }
                        sync_state = 0;
                    }
            }
        ROS_WARN ("Serial read thread exiting.");
    }

    /**
     * @brief 发布 LaserScan 消息
     * @param points 一帧数据点
     */
    void
    publishScan (const std::vector<LidarPoint> &points)
    {
        // 您的Python脚本是缓存并绘制所有点，而ROS的LaserScan通常发布的是**一圈扫描**或**一个数据包**
        // 假设每个数据包是一个小的角度扇区，我们应将其作为完整的LaserScan来发布，或者
        // 将多个数据包累积成一圈再发布（更复杂）。这里我们**直接将一个数据包中的点作为一次扫描发布**，
        // 实际的Lidar驱动会累积一整圈再发布。

        if (points.empty ())
            return;

        sensor_msgs::LaserScan scan;
        scan.header.stamp = ros::Time::now ();
        scan.header.frame_id = frame_id_; // ROS坐标系名称

        // **关键设置**：提取角度范围和角度增量
        double start_angle_rad = points.front ().angle_rad;
        double end_angle_rad = points.back ().angle_rad;
        size_t num_points = points.size ();

        scan.angle_min = start_angle_rad;
        scan.angle_max = end_angle_rad;

        // 角度增量计算
        if (num_points > 1)
            {
                // 假设点按角度顺序排列
                scan.angle_increment
                    = (end_angle_rad - start_angle_rad) / (num_points - 1);
                // 处理角度跨越0/360度的情况（高级逻辑，这里简化）
            }
        else
            {
                scan.angle_increment = 0.0;
            }

        // 时间设置
        // 假设一圈扫描时间为0.1秒 (10Hz)，则点间时间可以估算
        scan.time_increment
            = 0.1 / 360 * (scan.angle_increment * 180 / M_PI); // 粗略估算

        scan.scan_time = 0.1; // 假设一帧时间

        // 距离设置
        scan.range_min = 0.05; // 最小有效距离，毫米转换为米
        scan.range_max = 4.0;  // 对应Python中的4000mm

        // 填充距离数据 (ranges)
        scan.ranges.resize (num_points);
        for (size_t i = 0; i < num_points; ++i)
            {
                // Lidar数据通常是毫米(mm)，ROS LaserScan需要米(m)
                scan.ranges[i] = points[i].distance_mm / 1000.0;
            }

        // 填充强度数据 (intensities) -
        // 如果Lidar没有提供强度，则留空或使用默认值
        // scan.intensities.resize(num_points, 0.0);

        pub_.publish (scan);
    }
};

int
main (int argc, char **argv)
{
    // ROS 初始化
    ros::init (argc, argv, "lidar_driver_node");
    ros::NodeHandle nh ("~"); // 私有节点句柄，用于加载参数

    LidarDriver driver (nh);

    // 启动驱动
    driver.start ();

    // ROS 主循环，处理回调函数
    // (如定时器、服务、订阅)，但本例中主要工作在独立线程
    ros::spin ();

    // 当 ros::spin() 退出 (例如 Ctrl+C)
    driver.stop ();

    return 0;
}
