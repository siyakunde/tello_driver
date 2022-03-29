#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from mavros_msgs.msg import RCIn



class Forwarder:
    def __init__(self, ranges: dict) -> None:
        self.transmitter_ranges = ranges
        self.mavros_subscriber = rospy.Subscriber("/mavros/rc/in", RCIn, callback=self.__transmitter_cb)
        self.joy_publisher = rospy.Publisher("/joy", Joy, queue_size=1)

        return

    
    def __normalize_to_range(self, value: int, range_min: int, range_max: int) -> int:
        """
        Normalize a value to the range [-1, 1]
        """
        return int(2 * (value - range_min) / (range_max - range_min) - 1)


    def __transmitter_cb(self, msg: RCIn) -> None:
        """
        Convert the MAVROS message into a joy message and forward the converted message

        Joy Axes:
            - 0: Left/Right Axis stick left
            - 1: Up/Down Axis stick left
            - 2: Left/Right Axis stick right
            - 3: Up/Down Axis stick right
            - 4: RT
            - 5: LT
            - 6: cross key left/right
            - 7: cross key up/down
        """
        # Create a new Joy message to publish
        joy_msg = Joy()

        # NOTE: RCIn channel indices need to be determined and updated accordingly
        joy_msg.axes = [
            self.__normalize_to_range(msg.channels[0], self.transmitter_ranges['yaw_min'], self.transmitter_ranges['yaw_max']),
            self.__normalize_to_range(msg.channels[1], self.transmitter_ranges['throttle_min'], self.transmitter_ranges['throttle_max']),
            self.__normalize_to_range(msg.channels[2], self.transmitter_ranges['roll_min'], self.transmitter_ranges['roll_max']),
            self.__normalize_to_range(msg.channels[3], self.transmitter_ranges['pitch_min'], self.transmitter_ranges['pitch_max']),
            0,
            0,
            0,
            0
        ]

        # Publish the converted message
        self.joy_publisher.publish(joy_msg)

        return



def main() -> None:
    # Initialize the node
    rospy.init_node('forwarder_node', anonymous=True)

    # Instantiate a new forwarder with the set ranges
    forwarder = Forwarder(
        {
            'max_thrust': rospy.get_param('~max_thrust'),
            'min_thrust': rospy.get_param('~min_thrust'),
            'max_yaw': rospy.get_param('~max_yaw'),
            'min_yaw': rospy.get_param('~min_yaw'),
            'max_pitch': rospy.get_param('~max_pitch'),
            'min_pitch': rospy.get_param('~min_pitch'),
            'max_roll': rospy.get_param('~max_roll'),
            'min_roll': rospy.get_param('~min_roll')
        }
    )

    # Run until the node is stopped
    rospy.spin()

    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass