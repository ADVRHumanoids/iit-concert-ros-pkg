#!/usr/bin/env python3
"""Publishes each simulated Bosch USS5 scan as the robot's ultrasound reading.

/bosch_uss5/<sensor>/scan (LaserScan) -> /bosch_uss5/<sensor> (Range): the closest ray within
the scan limits, +inf when no ray hits (REP 117).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range

SENSORS = [f'ultrasound_{side}_{kind}' for kind in ('sag', 'lat') for side in ('fl', 'fr', 'rl', 'rr')]


class Uss5RangeRelay(Node):

    def __init__(self):
        super().__init__('uss5_range_relay')
        self._pubs = {}
        for sensor in SENSORS:
            self._pubs[sensor] = self.create_publisher(Range, f'/bosch_uss5/{sensor}', 10)
            self.create_subscription(LaserScan, f'/bosch_uss5/{sensor}/scan',
                                     lambda msg, s=sensor: self._on_scan(s, msg), qos_profile_sensor_data)

    def _on_scan(self, sensor, scan):
        hits = [r for r in scan.ranges if scan.range_min <= r <= scan.range_max]
        msg = Range()
        msg.header = scan.header
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = scan.angle_max - scan.angle_min
        msg.min_range = scan.range_min
        msg.max_range = scan.range_max
        msg.range = min(hits) if hits else math.inf
        self._pubs[sensor].publish(msg)


def main():
    rclpy.init()
    rclpy.spin(Uss5RangeRelay())


if __name__ == '__main__':
    main()
