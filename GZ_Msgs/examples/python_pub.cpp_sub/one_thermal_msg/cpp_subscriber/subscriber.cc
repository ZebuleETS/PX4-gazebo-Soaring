#include <gz/transport/Node.hh>
#include <gz/msgs/thermal.pb.h>
#include <iostream>

// Callback function to process received messages
void ThermalCallback(const gz::msgs::Thermal &msg)
{
    std::cout << "Received Thermal Message:" << std::endl;
    std::cout << "  ID: " << msg.id() << std::endl;
    std::cout << "  X (ENU East): " << msg.x() << std::endl;
    std::cout << "  Y (ENU North): " << msg.y() << std::endl;
    std::cout << "  Zi (CBL height): " << msg.zi() << std::endl;
    std::cout << "  Wi (updraft strength): " << msg.wi() << std::endl;
    std::cout << "  Lifetime: " << msg.lifetime() << std::endl;
    std::cout << "  Birth Time: " << msg.birth_time() << std::endl;
}

int main(int argc, char **argv)
{
    // Initialize Gazebo Transport node
    gz::transport::Node node;

    // Subscribe to the thermal message topic
    std::string thermalTopic = "/thermal_msg";
    if (!node.Subscribe(thermalTopic, ThermalCallback))
    {
        std::cerr << "Error subscribing to topic [" << thermalTopic << "]" << std::endl;
        return -1;
    }

    std::cout << "Subscribed to topic [" << thermalTopic << "]" << std::endl;

    // Keep the node running to listen for messages
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
