#!/usr/bin/env python
import rospy
from random_pub_sub_interfaces.msg import RandomRange
from std_msgs.msg import Int32

class RandomSubscriber:
    def __init__(self):
        rospy.init_node('random_subscriber', anonymous=True)

        self.min_interest = rospy.get_param('~min_interest', -1)
        self.max_interest = rospy.get_param('~max_interest', 3)

        rospy.Subscriber('random_number', RandomRange, self.listener_callback)

    def listener_callback(self, msg):
        if self.min_interest <= msg.data <= self.max_interest:
            output_msg = "Received {} that is in range <{}, {}>".format(msg.data, self.min_interest, self.max_interest)
            rospy.loginfo(output_msg)

if __name__ == '__main__':
    random_subscriber = RandomSubscriber()
    rospy.spin()
