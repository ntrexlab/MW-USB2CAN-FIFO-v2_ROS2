#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "can_msgs/msg/frame.hpp"
#include "rclcpp/parameter.hpp"
#include "CAN_Access.h"
#include <semaphore.h>

/*
    < ROS2 Topic Test >

    ros2 topic pub /usb2can_send can_msgs/msg/Frame '{id: 123, dlc: 8, data: [1, 2, 3, 4, 5, 6, 7, 8], extended: false, rtr: false}'

*/

class Usb2CanNode : public rclcpp::Node
{
public:
    Usb2CanNode() : Node("usb2can_fifo_node")
    {
        int noDevice = CAN_Fifo_ScanSerialNumber();

        if (noDevice <= 0)
        {
            RCLCPP_ERROR(this->get_logger(), "USB2CAN(FIFO) was not detected\r\n");
            return;
        }

        for (int i = 0; i < noDevice; i++)
        {
            const char *serialNumber = CAN_Fifo_GetSerialNumber(i);
            RCLCPP_INFO(this->get_logger(), "Serial Number: %s \r\n", serialNumber);
        }

        h = CAN_OpenFifo(CAN_Fifo_GetSerialNumber(0));

        if (h < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "USB2CAN(FIFO) open failed\r\n");
            return;
        }

        this->declare_parameter<int>("bitrate", 1000);
        this->declare_parameter<int>("filterID", 0);
        this->declare_parameter<int>("filterMask", 0);
        this->declare_parameter<int>("startupTransferMode", 1);
        this->declare_parameter<int>("busOffRecovery", 1);

        this->get_parameter("bitrate", bitrate_);
        this->get_parameter("filterID", filterID_);
        this->get_parameter("filterMask", filterMask_);
        this->get_parameter("startupTransferMode", startupTransferMode_);
        this->get_parameter("busOffRecovery", busOffRecovery_);

        CAN_SetConfig(h, bitrate_, filterID_, filterMask_, startupTransferMode_, busOffRecovery_);
        CAN_GetConfig(h, &bitrate_, &filterID_, &filterMask_, &startupTransferMode_, &busOffRecovery_);

        RCLCPP_INFO(this->get_logger(), "bitrate: %d", bitrate_);
        RCLCPP_INFO(this->get_logger(), "filterID: %d", filterID_);
        RCLCPP_INFO(this->get_logger(), "filterMask: %d", filterMask_);
        RCLCPP_INFO(this->get_logger(), "startupTransferMode: %d", startupTransferMode_);
        RCLCPP_INFO(this->get_logger(), "busOffRecovery: %d", busOffRecovery_);

        CAN_SetTransferMode(h, 1);

        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 1;
        sem_init(&hEvent, 1, 0);
        CAN_SetRxEventNotification(h, &hEvent);

        RCLCPP_INFO(this->get_logger(), "Success USB2CAN(FIFO) Open \r\n");

        publisher_ = create_publisher<can_msgs::msg::Frame>("can_recv", 10);
        subscription_ = create_subscription<can_msgs::msg::Frame>("can_send", 10, std::bind(&Usb2CanNode::can_message_callback, this, std::placeholders::_1));

        timer_1ms = create_wall_timer(std::chrono::milliseconds(1), std::bind(&Usb2CanNode::Timer_1ms, this));
        timer_10ms = create_wall_timer(std::chrono::milliseconds(10), std::bind(&Usb2CanNode::Timer_10ms, this));
        timer_20ms = create_wall_timer(std::chrono::milliseconds(20), std::bind(&Usb2CanNode::Timer_20ms, this));
    }

private:
    void RecvCanMessage(long h)
    {
        can_msgs::msg::Frame can_message;

        char rdata[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        long rid;
        int rlen, ext, rtr;

        int ret = CAN_Recv(h, &rid, &rlen, rdata, &ext, &rtr);
        if (ret)
        {
            can_message.header.stamp = this->now(); 
            can_message.id = rid;
            can_message.dlc = rlen;
            can_message.rtr = rtr;
            can_message.extended = ext;

            for (int i = 0; i < 8; i++)
            {
                can_message.data[i] = (int)(unsigned char)rdata[i];
            }

            publisher_->publish(can_message);
        }
    }

    void can_message_callback(const can_msgs::msg::Frame::SharedPtr msg)
    {
        unsigned char sdata[8];

        RCLCPP_INFO(this->get_logger(), "Received CAN Frame - ID: %u, DLC: %u", msg->id, msg->dlc);

        for (int i = 0; i < msg->dlc; i++)
        {
            sdata[i] = msg->data[i];
            RCLCPP_INFO(this->get_logger(), "Data[%d]: %d", i, msg->data[i]);
        }

        CAN_Send(h, msg->id, msg->dlc, (char *)sdata, msg->extended, msg->rtr);


    }

private:
    void Timer_10ms()
    {
        unsigned char sdata[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22};

        long sid = 10;
        int ret = CAN_Send(h, sid, 8, (char *)sdata, 0, 0);
        if (!ret)
            RCLCPP_ERROR(this->get_logger(), "Send failed\n");
    }
    void Timer_20ms()
    {
        unsigned char sdata[8] = {0xBB, 0xAA, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22};

        long sid = 20;
        int ret = CAN_Send(h, sid, 8, (char *)sdata, 0, 0);
        if (!ret)
            RCLCPP_ERROR(this->get_logger(), "Send failed\n");
    }

    void Timer_1ms()
    {
        int rx_count = CAN_CountRxQueue(h);

        if (rx_count > 0)
        {

            RecvCanMessage(h);
        }
        else
        {
            int result = sem_timedwait(&hEvent, &ts);

            if (!result)
                RecvCanMessage(h);
        }
    }

    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_1ms, timer_10ms, timer_20ms;

    long h, bitrate_;
    unsigned long filterID_, filterMask_;
    int startupTransferMode_, busOffRecovery_;

    sem_t hEvent;
    struct timespec ts;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Usb2CanNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}