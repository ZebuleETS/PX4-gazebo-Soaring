import thermal_pb2
from gz.transport13 import Node
import time

def main():
    node = Node()
    thermal_group_topic = "/thermal_msg_group"

    # Advertise the topic with the ThermalGroup message type
    pub = node.advertise(thermal_group_topic, thermal_pb2.ThermalGroup)

    # Create and populate the ThermalGroup message
    thermal_group = thermal_pb2.ThermalGroup()

    # Add three Thermal messages to the group (fields match thermal.proto)
    thermal1 = thermal_group.thermals.add()
    thermal1.id = 1
    thermal1.x = 500.5       # ENU East (meters)
    thermal1.y = 200.0       # ENU North (meters)
    thermal1.zi = 1500.0     # Convective boundary layer height (m)
    thermal1.wi = 3.0        # Updraft strength (m/s)
    thermal1.lifetime = 300.0
    thermal1.birth_time = 12.12

    thermal2 = thermal_group.thermals.add()
    thermal2.id = 2
    thermal2.x = 600.5
    thermal2.y = 300.0
    thermal2.zi = 1800.0
    thermal2.wi = 4.0
    thermal2.lifetime = 400.0
    thermal2.birth_time = 13.14

    thermal3 = thermal_group.thermals.add()
    thermal3.id = 3
    thermal3.x = 700.5
    thermal3.y = 400.0
    thermal3.zi = 2000.0
    thermal3.wi = 5.0
    thermal3.lifetime = 500.0
    thermal3.birth_time = 19.99
    # Publish the ThermalGroup message in a loop to keep the publisher alive
    while True:
        pub.publish(thermal_group)
        print("Published ThermalGroup message with 3 thermals to /thermal_msg_group")
        time.sleep(1)

if __name__ == "__main__":
    main()
