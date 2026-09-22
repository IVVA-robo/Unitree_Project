#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

// Функция расчета CRC для протокола Unitree
unsigned int calculate_crc(const uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        data = ptr[i];
        for (uint32_t j = 0; j < 32; j++) {
            xbit = (crc >> 31) ^ (data >> 31);
            crc <<= 1;
            if (xbit) {
                crc ^= 0x04C11DB7;
            }
            data <<= 1;
        }
    }
    return crc;
}

class RobotControlNode : public rclcpp::Node
{
public:
    RobotControlNode() : Node("robot_control_node")
    {
        publisher_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&RobotControlNode::send_command, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Узел запущен: отправка команд с CRC!");
    }

private:
    void send_command()
    {
        auto message = unitree_go::msg::LowCmd();
        
        message.head[0] = 0xFE;
        message.head[1] = 0xEF;
        message.level_flag = 0xFF;

        // Включаем первый мотор в режим управления по позиции
        message.motor_cmd[0].mode = 0x01; 
        message.motor_cmd[0].q = 0.0;     
        message.motor_cmd[0].kp = 60.0;   
        message.motor_cmd[0].kd = 5.0;    

        // Вычисляем и записываем контрольную сумму CRC для пакета
        message.crc = calculate_crc(reinterpret_cast<uint32_t*>(&message), 
                                    (sizeof(unitree_go::msg::LowCmd) / 4) - 1);

        publisher_->publish(message);
    }

    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
