#include <bupt_can/bupt_can.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <thread>

#define RUN_MODE 0x7005 /* 0:运控   1:位置   2:速度   3:电流 */
enum _RUN_MODE
{
  MODE_REMOTE,
  MODE_POS,
  MODE_SPEED,
  MODE_ELEC
};
#define LOC_REF 0x7016 /* 位置模式角度命令 */

enum _COMM_TYPE
{
  TYPE_0,
  TYPE_1, /* 运控模式电机控制指令 */
  TYPE_2,
  TYPE_3, /* 电机使能运行 */
  TYPE_4, /* 电机停止运行 */
  TYPE_6 = 6,
  TYPE_7,
  TYPE_17 = 17,
  TYPE_18, /* 参数写入 */
  TYPE_21 = 21,
  TYPE_22,
};
#define CAN_MASTER 0
#define STEP_SIZE 0.5

void print_info(const std::shared_ptr<can_frame> &frame)
{
  std::cout << "id" << frame->can_id << std::endl;
  std::cout << "dlc" << frame->can_dlc << std::endl;
  std::cout << "data";
  for (int i = 0; i < frame->can_dlc; i++)
  {
    std::cout << frame->data[i] << " ";
  }
  std::cout << std::endl;
}

void mi_motor_on(Can &can, uint8_t can_motor_id)
{
  uint32_t can_exd_id = 0;
  can_exd_id |= (TYPE_3 & 0x1f) << 24;
  can_exd_id |= (CAN_MASTER & 0xffff) << 8;
  can_exd_id |= (can_motor_id & 0xff);
  std::array<uint8_t, 8> Txdata = {0};
  can.send_can(can_exd_id, Can::CAN_ID_EXT, 8, Txdata);
}

void mi_motor_zero(Can &can, uint8_t can_motor_id)
{
  uint32_t can_exd_id = 0;
  can_exd_id |= (6 & 0x1f) << 24;
  can_exd_id |= (CAN_MASTER & 0xffff) << 8;
  can_exd_id |= (can_motor_id & 0xff);
  std::array<uint8_t, 8> Txdata = {0};
  Txdata[0] = 1;
  can.send_can(can_exd_id, Can::CAN_ID_EXT, 8, Txdata);
}

/* 切换电机控制模式 */
void mi_motor_mode(Can &can, uint8_t can_motor_id, uint32_t mode)
{
  uint32_t can_exd_id = 0;
  can_exd_id |= (TYPE_18 & 0x1f) << 24;
  can_exd_id |= (CAN_MASTER & 0xffff) << 8;
  can_exd_id |= (can_motor_id & 0xff);

  std::array<uint8_t, 8> TxData;
  TxData[0] = RUN_MODE & 0xFF;         // index 低字节
  TxData[1] = (RUN_MODE >> 8) & 0xFF;  // index 高字节
  TxData[2] = 0x00;                    // 固定为 0
  TxData[3] = 0x00;                    // 固定为 0
  TxData[4] = mode & 0xFF;
  TxData[5] = (mode >> 8) & 0xFF;
  TxData[6] = (mode >> 16) & 0xFF;
  TxData[7] = (mode >> 24) & 0xFF;

  can.send_can(can_exd_id, Can::CAN_ID_EXT, 8, TxData);
}

/* 控制电机角度 */
void mi_motorctrl(Can &can, uint8_t motor_canid, float angle)
{
  uint32_t can_exd_id = 0;

  can_exd_id |= (TYPE_18 & 0x1F) << 24;      // 28-24 位: 通信模式 (5 位)
  can_exd_id |= (CAN_MASTER & 0xFFFF) << 8;  // 23-8 位: 主 CAN ID (16 位)
  can_exd_id |= (motor_canid & 0xFF);        // 7-0 位: 电机 CAN ID (8 位)

  std::array<uint8_t, 8> Txdata;
  Txdata[0] = LOC_REF & 0xFF;         // index 低字节
  Txdata[1] = (LOC_REF >> 8) & 0xFF;  // index 高字节
  Txdata[2] = 0x00;                   // 固定为 0
  Txdata[3] = 0x00;                   // 固定为 0

  memcpy(&Txdata[4], &angle, sizeof(angle));  // 将 float 转换为字节存储在 TxData 中

  can.send_can(can_exd_id, Can::CAN_ID_EXT, 8, Txdata);
}

class Motor_Node : public rclcpp::Node
{
 public:
  Motor_Node() : Node("Motor_Node"), can("can0")
  {
    motor_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/topic", 10,
        std::bind(&Motor_Node::send_motor, this, std::placeholders::_1));  // TODO: 运行前记得修改成具体的 topic
  }

 private:
  void send_motor(const std_msgs::msg::Float32MultiArray msg)
  {
    can.can_start();

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    mi_motor_mode(can, 1, MODE_POS);
    mi_motor_mode(can, 2, MODE_POS);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    mi_motor_zero(can, 1);
    mi_motor_zero(can, 2);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    mi_motor_on(can, 1);
    mi_motor_on(can, 2);

    mi_motorctrl(can, 1, msg.data[0]);
    mi_motorctrl(can, 2, msg.data[1]);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motor_sub;
  Can can;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Motor_Node>());
  rclcpp::shutdown();
  return 0;
}