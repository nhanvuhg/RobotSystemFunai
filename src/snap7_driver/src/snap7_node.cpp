#include <rclcpp/rclcpp.hpp>
#include "snap7_driver/snap7.h"
#include "plc_msg/msg/plc_read.hpp"
#include "plc_msg/msg/plc_write.hpp"
#include "plc_msg/msg/plc_response.hpp"

using std::placeholders::_1;

class Snap7Node : public rclcpp::Node
{
public:
    Snap7Node() : Node("snap7_node")
    {
        write_sub_ = this->create_subscription<plc_msg::msg::PLCWrite>(
            "plc/write_request", 10,
            std::bind(&Snap7Node::handle_write, this, _1));

        read_sub_ = this->create_subscription<plc_msg::msg::PLCRead>(
            "plc/read_request", 10,
            std::bind(&Snap7Node::handle_read, this, _1));

        read_pub_ = this->create_publisher<plc_msg::msg::PLCResponse>(
            "plc/read_response", 10);
    }

    ~Snap7Node()
    {
        for (auto &pair : plc_clients)
        {
            pair.second->Disconnect();
            delete pair.second;
        }
    }

private:
    std::unordered_map<std::string, TS7Client *> plc_clients;
    rclcpp::Subscription<plc_msg::msg::PLCWrite>::SharedPtr write_sub_;
    rclcpp::Subscription<plc_msg::msg::PLCRead>::SharedPtr read_sub_;
    rclcpp::Publisher<plc_msg::msg::PLCResponse>::SharedPtr read_pub_;

    TS7Client *get_or_connect_client(const std::string &ip)
    {
        if (plc_clients.find(ip) == plc_clients.end())
        {
            TS7Client *new_client = new TS7Client();
            if (new_client->ConnectTo(ip.c_str(), 0, 1) == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Connected to PLC at %s", ip.c_str());
                plc_clients[ip] = new_client;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to connect to PLC at %s", ip.c_str());
                delete new_client;
                return nullptr;
            }
        }
        return plc_clients[ip];
    }

    void handle_read(const plc_msg::msg::PLCRead::SharedPtr msg)
    {
        TS7Client *client = get_or_connect_client(msg->plc_ip);
        if (!client)
            return;

        uint8_t buffer[512];
        int res = client->DBRead(msg->db_number, msg->offset, msg->size * 2, buffer);

        auto response = plc_msg::msg::PLCResponse();
        response.success = (res == 0);
        if (res == 0)
        {
            for (int i = 0; i < msg->size; ++i)
            {
                int16_t value = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
                response.values.push_back(value);
            }
        }

        read_pub_->publish(response);
    }

    void handle_write(const plc_msg::msg::PLCWrite::SharedPtr msg)
    {
        TS7Client *client = get_or_connect_client(msg->plc_ip);
        if (!client)
            return;

        size_t byte_count = msg->values.size() * 2;
        uint8_t buffer[512];
        for (size_t i = 0; i < msg->values.size(); ++i)
        {
            buffer[i * 2] = msg->values[i] >> 8;
            buffer[i * 2 + 1] = msg->values[i] & 0xFF;
        }

        int res = client->DBWrite(msg->db_number, msg->offset, byte_count, buffer);
        if (res == 0)
            RCLCPP_INFO(this->get_logger(), "Wrote to %s: DB%d [%zu INTs]", msg->plc_ip.c_str(), msg->db_number, msg->values.size());
        else
            RCLCPP_ERROR(this->get_logger(), "Write failed to %s: code %d", msg->plc_ip.c_str(), res);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Snap7Node>());
    rclcpp::shutdown();
    return 0;
}
