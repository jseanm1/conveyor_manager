#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool


class Listener:
    """Listen to the conveyor status and process diagnostics."""

    def __init__(self, input_file, timeout):
        self.input_file = input_file
        self.timeout = timeout

        self.input_buffer = []
        self.timestamps = []

        # Setup ROS node
        rospy.init_node("listener")
        rospy.Subscriber("conveyor_status", Bool, self.status_cb)

        # Start the timer
        self.timer = rospy.Timer(rospy.Duration(self.timeout),
                                 self.timeout_cb,
                                 oneshot=True)

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

    def status_cb(self, status):
        self.input_buffer.append(status.data)
        # Ideally we should use a custom msg with header information for time
        self.timestamps.append(rospy.Time.now())

        # Reset the timer as we have recieved a new msg
        self.timer.shutdown()
        self.timer = rospy.Timer(rospy.Duration(self.timeout),
                                 self.timeout_cb,
                                 oneshot=True)

    def timeout_cb(self, event):
        rospy.loginfo("Timeout after " + str(self.timeout) + " seconds")
        rospy.loginfo(str(len(self.input_buffer)) + " status msgs recieved")

        self.read_file()

        self.evaluate_conveyor_status()

    def evaluate_conveyor_status(self):
        if len(self.commands) == 0:
            rospy.loginfo("No input data recieved to evaluate")
            rospy.signal_shutdown("Done!")
            return
        elif len(self.input_buffer) == 0:
            rospy.loginfo("No conveyor status msgs recieved to evaluate")
            rospy.signal_shutdown("Done!")
            return

        # The following methods assume that we have at least some status data
        # and input sequence data
        self.print_is_subsequence()
        self.print_max_delta_T()
        self.print_max_percent_subsequence()
        self.print_is_last_cmd_last_input()

        rospy.signal_shutdown("Done!")

    def print_is_subsequence(self):
        """
        Is the recieved status sequence a subsequence of the test sequence?
        """
        n = len(self.commands)
        m = len(self.input_buffer)

        if m > n:
            # If we somehow recieve more status messages than the test sequence
            # then it is not a subset
            rospy.loginfo("Recieved statues sequence is not a subsequence of"
                          + " test sequence in " + self.input_file)
        else:
            for i in range(0, n-m+1):
                self.is_subsequence = True

                for j in range(m):
                    if self.input_buffer[j] != self.commands[i+j]:
                        self.is_subsequence = False
                        break

                if self.is_subsequence is True:
                    rospy.loginfo("Recieved status sequence is a subsequence"
                                  + " of test sequnce in " + self.input_file)
                    break

            if self.is_subsequence is not True:
                rospy.loginfo("Recieved statues sequence is not a subsequence"
                              + " of test sequence in " + self.input_file)

    def print_max_delta_T(self):
        """
        What is the largest potential time gap between matching status
        recieved?
        """
        max_delta_T = 0

        for i in range(len(self.input_buffer) - 1):
            for j in range(i+1, len(self.input_buffer)):
                if self.input_buffer[i] == self.input_buffer[j]:
                    delta_T = self.timestamps[j] - self.timestamps[i]

                    if delta_T.to_sec() > max_delta_T:
                        max_delta_T = delta_T.to_sec()
                    break

        max_delta_T = "{:.2f}".format(max_delta_T)
        rospy.loginfo("Maximum potential time gap between matching status"
                      + " received is " + max_delta_T + " s")


    def print_max_percent_subsequence(self):
        """
        What percentage match is the largest subsequence if one exists?
        """
        # This requirement is slightly ambiguous. Does it mean in pseudocode
        # if is_subsequence then percentage = len(status) / len(commands)
        # else 0
        # or does it mean
        # percentage = max(len(subset of status) / len(commands))
        # or does it mean
        # percentage = max(len(subset of status) / len(status))
        # Printing all three cases

        if self.is_subsequence is True:
            percent = len(self.input_buffer) * 100 / len(self.commands)
            percent = "{:.2f}".format(percent)
            rospy.loginfo("Recieved status is a " + percent
                          + "% subsequence of test sequence")
        else:
            n = len(self.commands)
            m = len(self.input_buffer)
            max_match_count = 0

            for i in range(m):
                for j in range(n):
                    match_count = 0

                    for k in range(min(m-i, n-j)):
                        if self.input_buffer[i+k] == self.commands[j+k]:
                            match_count += 1
                        else:
                            break

                    if match_count > max_match_count:
                        max_match_count = match_count

            percent = max_match_count * 100 / m
            percent = "{:.2f}".format(percent)
            rospy.loginfo(percent + "% of the received status sequence is the"
                          + " largest subsequence of the test sequence found"
                          + " on received status sequence")

            percent = max_match_count * 100 / n
            percent = "{:.2f}".format(percent)
            rospy.loginfo(percent + "% of the test sequence is  largest"
                          + " subsequence of test sequence found on received"
                          + " status sequence")

    def print_is_last_cmd_last_input(self):
        """
        Does the final status message recieved match the final value in the
        inptu sequence?
        """
        if self.input_buffer[-1] == self.commands[-1]:
            rospy.loginfo("Final status message recieved matches the final"
                          + " value in the input data")
        else:
            rospy.loginfo("Final status message recieved does not match the "
                          + " final value in the input data")


if __name__ == "__main__":
    try:
        """
        Default file name and the timeout value are hard coded, but can be
        configured using ROS parameters.
        Subscribed topic may be changed by topic remapping.
        """
        file_name = rospy.get_param("input_file", "input_sequence.txt")
        timeout = rospy.get_param("timeout", 10)

        listener = Listener(file_name, timeout)
        # listener.listen()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("Listener is interrupted. Exiting now.")
        pass
