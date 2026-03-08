#include <gz/transport/Node.hh>
#include "gz/msgs/thermal.pb.h"
#include <iostream>

// Callback function to process received ThermalGroup messages
void ThermalGroupCallback(const gz::msgs::ThermalGroup &msg)
{
    std::cout << "Received ThermalGroup Message:" << std::endl;

    for (int i = 0; i < msg.thermals_size(); ++i)
    {
        const auto &thermal = msg.thermals(i);
        std::cout << "  Thermal " << i + 1 << ":" << std::endl;
        std::cout << "    ID: " << thermal.id() << std::endl;
        std::cout << "    X (ENU East): " << thermal.x() << std::endl;
        std::cout << "    Y (ENU North): " << thermal.y() << std::endl;
        std::cout << "    Zi (CBL height): " << thermal.zi() << std::endl;
        std::cout << "    Wi (updraft strength): " << thermal.wi() << std::endl;
        std::cout << "    Lifetime: " << thermal.lifetime() << std::endl;
        std::cout << "    Birth Time: " << thermal.birth_time() << std::endl;
    }
}

int main(int argc, char **argv)
{
    // Initialize Gazebo Transport node
    gz::transport::Node node;

    // Subscribe to the thermal group message topic
    std::string thermalGroupTopic = "/thermal_msg_group";
    if (!node.Subscribe(thermalGroupTopic, ThermalGroupCallback))
    {
        std::cerr << "Error subscribing to topic [" << thermalGroupTopic << "]" << std::endl;
        return -1;
    }

    std::cout << "Subscribed to topic [" << thermalGroupTopic << "]" << std::endl;

    // Keep the node running to listen for messages
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}

