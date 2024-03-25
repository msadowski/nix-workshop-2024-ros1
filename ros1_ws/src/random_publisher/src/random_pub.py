#!/usr/bin/env python
import rospy
from random_pub_sub_interfaces.srv import SetInt32, SetInt32Response
from random_pub_sub_interfaces.msg import RandomRange
from std_msgs.msg import Int32
import random

class RandomPublisher:
    def __init__(self):
        rospy.init_node('random_publisher')

        self.publisher_ = rospy.Publisher('random_number', RandomRange, queue_size=1)

        self.start_number = rospy.get_param('~start_number', -5)
        self.stop_number = rospy.get_param('~stop_number', 10)
        timer_period_s = rospy.get_param('~timer_period_s', 0.5)

        self.start_number_srv = rospy.Service('~/set_start_number', SetInt32, self.set_start_number_cb)
        self.stop_number_srv = rospy.Service('~/set_stop_number', SetInt32, self.set_stop_number_cb)

        self.timer = rospy.Timer(rospy.Duration(timer_period_s), self.timer_callback)

    def set_start_number_cb(self, request):
        response = SetInt32Response()
        if request.data > self.stop_number:
            response.success = False
            response.message = "Can't set start number that is higher than stop number"
        else:
            self.start_number = request.data
            response.success = True
            response.message = "Start number successfully adjusted"
        return response

    def set_stop_number_cb(self, request):
        response = SetInt32Response()
        if request.data < self.start_number:
            response.success = False
            response.message = "Can't set stop number that is lower than start number"
        else:
            self.stop_number = request.data
            response.success = True
            response.message = "Stop number successfully adjusted"
        return response

    def timer_callback(self, event):
        msg = RandomRange()
        msg.data = random.randint(self.start_number, self.stop_number)
        msg.start_number = self.start_number
        msg.stop_number = self.stop_number
        self.publisher_.publish(msg)

if __name__ == '__main__':
    try:
        random_publisher = RandomPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
