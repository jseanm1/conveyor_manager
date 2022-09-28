#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

import sys


class Talker:
    """Read the commands from a text file and publish at a set frequency."""

    def __init__(self, input_file, freq):
        self.input_file = input_file
        self.freq = freq

        # Setup ROS node
        rospy.init_node("talker")
        self.pub = rospy.Publisher("conveyor_set", Bool, queue_size=10)

        # Read data
        self.read_file()

    def read_file(self):
        self.commands = []

        try:
            with open(self.input_file, 'r') as input:
                for line in input.readlines():
                    token = line.rstrip('\n')
                    if token == "True":
                        self.commands.append(True)
                    elif token == "False":
                        self.commands.append(False)
                    else:
                        rospy.loginfo("Unrecognised input : |" + line + "|")

            rospy.loginfo(str(len(self.commands)) + " commands read from file")

        except OSError as e:
            rospy.loginfo(e)
            rospy.signal_shutdown("IO Error")

    def talk(self):
        rate = rospy.Rate(self.freq)
        i = 0

        rospy.loginfo("Publishing commands at " + str(self.freq) + " Hz")
        for cmd in self.commands:
            if not rospy.is_shutdown():
                self.pub.publish(cmd)
                i += 1
                rate.sleep()
            else:
                print("Talker node interrupted")
                break

        rospy.loginfo("Publishing commands ended")
        rospy.loginfo(str(i) + " commands published")

# Run the talker node
if __name__ == '__main__':
    try:
        """
        Default file name and frequency are hard coded, but can be configured
        using ROS parameters.
        Published topic may be changed by topic remapping.
        """
        file_name = rospy.get_param("input_file", "input_sequence.txt")
        frequency = rospy.get_param("publish_freq", 10)

        talker = Talker(file_name, frequency)
        talker.talk()

    except rospy.ROSInterruptException:
        print("Talker is interrupted. Exiting now.")
        pass
