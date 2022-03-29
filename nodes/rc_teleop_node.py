#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from mavros_msgs.msg import RCIn
from geometry_msgs.msg import Twist



class RCState:
    def __init__(self):
        self.rc_channels = [
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ]

        self.emergency_stop = False
        self.flight_mode = None
        self.land = False

        return



class RCTeleOpNode:
    def __init__(self, ranges: dict, eps: int=5) -> None:

        # Maintain the current state
        self.rc_prev_state = RCState()

        # Epsilon value used to provide a buffer for switch pwm values
        self.eps = eps

        # The parameterized RC settings
        self.transmitter_ranges = ranges

        # ROS Subscribers
        self.mavros_subscriber = rospy.Subscriber("/mavros/rc/in", RCIn, callback=self.__transmitter_cb)

        # ROS Publishers
        self.pub_cmd_out = rospy.Publisher('cmd_vel', Twist, queue_size=10, latch=False)
        self.pub_manual_takeoff = rospy.Publisher('manual_takeoff', Empty, queue_size=1, latch=False)
        self.pub_emergency = rospy.Publisher('emergency', Empty,  queue_size=1, latch=False)
        self.pub_land = rospy.Publisher('land', Empty,  queue_size=1, latch=False)

        return

    
    def __normalize_to_range(self, value: int, range_min: int, range_max: int) -> int:
        """
        Normalize a value to the range [-1, 1]
        """
        return int(2 * (value - range_min) / (range_max - range_min) - 1)


    def __get_current_state(self, channels) -> RCState:
        """
        Extract the current state

        Channel 4 (Switch E):
            965: Manual
            1515: AltHold
            2065: Computer

        Channel 5 (Switch C):
            965: Emergency Stop
            2065: Land
        """
        state = RCState()

        # Process Channels 4 (Switch E)
        if channels[4] <= self.transmitter_ranges['manual_mode'] + self.eps and \
           channels[4] >= self.transmitter_ranges['manual_mode'] - self.eps:
            state.flight_mode = 'manual'
        elif channels[4] <= self.transmitter_ranges['althold_mode'] + self.eps and \
           channels[4] >= self.transmitter_ranges['althold_mode'] - self.eps:
            state.flight_mode = 'althold'
        elif channels[4] <= self.transmitter_ranges['computer_mode'] + self.eps and \
           channels[4] >= self.transmitter_ranges['computer_mode'] - self.eps:
            state.flight_mode = 'computer'
        else:
            state.flight_mode = None

        # Process Channel 5 (Switch C)
        if channels[5] <= self.transmitter_ranges['emergency_stop'] + self.eps and \
           channels[5] >= self.transmitter_ranges['emergency_stop'] - self.eps:
           state.emergency_stop = True
        elif channels[5] <= self.transmitter_ranges['land'] + self.eps and \
           channels[5] >= self.transmitter_ranges['land'] - self.eps:
           state.land = True

        return state


    def __transmitter_cb(self, msg: RCIn) -> None:
        """
        Convert the MAVROS message into a joy message and forward the converted message
        """
        # Extract the current state
        rc_current_state = self.__get_current_state(msg.channels)

        if not self.rc_prev_state.flight_mode == 'manual' and rc_current_state.flight_mode == 'manual':
            self.pub_manual_takeoff.publish()

        # Process emergency stop
        if not self.rc_prev_state.emergency_stop and rc_current_state.emergency_stop:
            self.pub_emergency.publish()
            return

        # Process land
        if not self.rc_prev_state.land and rc_current_state.land:
            self.pub_land.publish()

        # Manual control mode
        if rc_current_state.flight_mode == 'manual':
            cmd = Twist()
            cmd.linear.x = -self.__normalize_to_range(msg.channels[0], self.transmitter_ranges['roll_min'], self.transmitter_ranges['roll_max'])
            cmd.linear.y = self.__normalize_to_range(msg.channels[1], self.transmitter_ranges['pitch_min'], self.transmitter_ranges['pitch_max'])
            cmd.linear.z = self.__normalize_to_range(msg.channels[2], self.transmitter_ranges['throttle_min'], self.transmitter_ranges['throttle_max'])
            cmd.angular.z = -self.__normalize_to_range(msg.channels[3], self.transmitter_ranges['yaw_min'], self.transmitter_ranges['yaw_max'])
            
            self.pub_cmd_out.publish(cmd)

        # Copy remaining previous state
        self.rc_prev_state = rc_current_state

        return



def main() -> None:
    # Initialize the node
    rospy.init_node('forwarder_node', anonymous=True)

    # Instantiate a new forwarder with the set ranges
    teleop_node = RCTeleOpNode(
        {
            'max_thrust': rospy.get_param('~max_thrust'),
            'min_thrust': rospy.get_param('~min_thrust'),
            'max_yaw': rospy.get_param('~max_yaw'),
            'min_yaw': rospy.get_param('~min_yaw'),
            'max_pitch': rospy.get_param('~max_pitch'),
            'min_pitch': rospy.get_param('~min_pitch'),
            'max_roll': rospy.get_param('~max_roll'),
            'min_roll': rospy.get_param('~min_roll'),
            'manual_mode': rospy.get_param('~manual_mode'),
            'althold_mode': rospy.get_param('~althold_mode'),
            'computer_mode': rospy.get_param('~computer_mode'),
            'emergency_stop': rospy.get_param('~emergency_stop'),
            'land': rospy.get_param('~land')
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