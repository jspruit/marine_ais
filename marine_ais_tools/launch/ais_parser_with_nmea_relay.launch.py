from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nmea_relay = Node(
        package="marine_ais_tools",
        executable="nmea_relay",
        name="nmea_relay",
        remappings=[
            ("/nmea", "/ais/raw"),
        ],
    )

    ais_parser = Node(
        package="marine_ais_tools",
        executable="ais_parser",
        name="ais_parser",
        remappings=[
            ("/nmea", "/ais/raw"),
            ("/messages", "/ais/messages"),
        ],
    )

    return LaunchDescription([
        nmea_relay,
        ais_parser
    ])