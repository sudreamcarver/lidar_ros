#include "ros/init.h"
#include "ros/node_handle.h"
#include <algorithm>
#include <atomic>
#include <boost/thread.hpp>
#include <cmath>
#include <exception>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <serial/serial.h> // 使用 libserial 作为串口库
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

/**
 * @brief 步骤 1: 最小化串口管理类
 * * 严格遵循 RAII (资源获取即初始化) 原则。
 * 构造函数获取资源 (打开串口)。
 * 析构函数释放资源 (关闭串口)。
 */
class LidarPort
{
  public:
    // init serial and open it
    LidarPort (const std::string &port_name, int baud_rate)
        : port_name_ (port_name), baud_rate_ (baud_rate), is_running_ (false)
    {
        ROS_INFO_STREAM ("Attempting to open serial port:"
                         << port_name_ << " at" << baud_rate_ << "baud.");

        try
            {
                // set port_name and baud_rate
                ser_.setPort (port_name_);
                ser_.setBaudrate (baud_rate_);
                // set time out
                serial::Timeout timeout
                    = serial::Timeout::simpleTimeout (1000);
                ser_.setTimeout (timeout);
                ser_.open ();
            }
        catch (const serial::IOException &e)
            {
                // if fail open serial print error
                ROS_FATAL_STREAM ("Failed to open serial port"
                                  << port_name_ << ". Error:" << e.what ());
                // print error and stop run
                throw std::runtime_error (e.what ());
            }

        if (!ser_.isOpen ())
            {
                ROS_FATAL_STREAM (
                    "serial port"
                    << port_name_
                    << "is not open, though no exception was throw.");
                throw std::runtime_error ("port not open");
            }

        ROS_INFO ("Serial port opened successfully");
    }

    ~LidarPort ()
    {
        if (ser_.isOpen ())
            {
                ser_.close ();
                ROS_INFO_STREAM ("Serial port" << port_name_ << " close.");
            }
        stop ();
    }

    /**
     * @brief 启动读取线程
     */
    void
    start ()
    {
        if (is_running_)
            {
                ROS_WARN ("Driver is already running");
                return;
            }
        is_running_ = true;

        // begin read thread
        ROS_INFO ("starting serial read thread.");
        read_thread_ = boost::thread (&LidarPort::serialReadLoop, this);
    }

    /**
     * @brief 停止读取线程
     */
    void
    stop ()
    {
        // 已经停止了
        if (!is_running_)
            {
                return;
            }
        // 向线程发送停止信号
        is_running_ = false;

        //  等待线程执行完毕 (关键!)
        //  join() 会阻塞，直到 serialReadLoop() 函数返回
        if (read_thread_.joinable ())
            {
                read_thread_.join ();
                ROS_INFO ("Serial read thread joined.");
            }

        // close serial

        if (ser_.isOpen ())
            {
                ser_.close ();
                ROS_INFO_STREAM ("Serial port" << port_name_ << "closed.");
            }
    }

  private:
    /**
     * @brief 串口读取循环 (在 read_thread_ 中运行)
     *
     * (步骤 2 的验证阶段: 我们只读取字节并打印，不解析)
     */
    void
    serialReadLoop ()
    {
        ROS_INFO ("serial read thread started.");

        // 循环条件:
        // 1. ROS 必须在运行 (ros::ok())
        // 2. 驱动必须被设置为 "running" (is_running_)
        while (ros::ok () && is_running_)
            {
                try
                    {
                        // read a byte
                        uint8_t byte;
                        if (ser_.read (&byte, 1) == 1)
                            {
                                // 步骤 2 验证:
                                // 我们只打印同步字节 (0xAA) 来确认数据正在流入
                                // ROS_INFO_THROTTLE(1.0, ...) 每 1.0
                                // 秒最多打印一次
                                if (byte == 0xAA)
                                    {
                                        ROS_INFO_THROTTLE (
                                            1.0, "Found sync byte: 0xAA");
                                    }
                            }
                    }
                catch (const serial::IOException &e)
                    {
                        ROS_ERROR_STREAM_THROTTLE (
                            1., "Serial read error:" << e.what ());
                    }
            }
    };
    std::string port_name_;
    int baud_rate_;
    serial::Serial ser_;
    std::atomic<bool> is_running_;
    boost::thread read_thread_;
};

int
main (int argc, char **argv)
{
    ros::init (argc, argv, "lidar_driver_serial_get");
    ros::NodeHandle nh ("~"); // 使用私有节点句柄 (~) 来获取参数

    // 从ROS参数服务器获取配置 (这是最佳实践)
    // 允许我们通过 .launch 文件更改，而无需重新编译
    std::string port = "/dev/ttyUSB0";
    int baudrate = 150000;

    nh.param ("port", port, port);
    nh.param ("baud_rate", baudrate, baudrate);

    try
        {
            LidarPort driver (port, baudrate);
            ROS_INFO ("Lidar node started, C-c to shutdown");
            ros::spin ();
        }
    catch (const std::exception &e)
        {
            ROS_ERROR_STREAM ("node failed to initialize:" << e.what ());
            return 1;
        }

    return 0;
}
