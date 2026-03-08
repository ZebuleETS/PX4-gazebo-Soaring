import thermal_pb2
from gz.transport13 import Node
import time

def main():
    node = Node()
    thermal_topic = "/thermal_msg"

    # Advertise the topic with the Thermal message type
    pub = node.advertise(thermal_topic, thermal_pb2.Thermal)

    # Create and populate the Thermal message (fields match thermal.proto)
    thermal = thermal_pb2.Thermal()
    thermal.id = 1
    thermal.x = 100.0          # ENU East (meters)
    thermal.y = 200.0          # ENU North (meters)
    thermal.zi = 1500.0        # Convective boundary layer height (m)
    thermal.wi = 3.5           # Updraft strength (m/s)
    thermal.lifetime = 300.0
    thermal.birth_time = 0.0

    # Publish the message in a loop to keep the publisher alive
    while True:
        pub.publish(thermal)
        print("Published message to /thermal_msg")
        time.sleep(1)

if __name__ == "__main__":
    main()
