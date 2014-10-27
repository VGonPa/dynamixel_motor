#!/usr/bin/env python
PKG = 'dynamixel_driver'
NNAME = 'test_dynamixel_serial_proxy'

import roslib
roslib.load_manifest(PKG)

import rospy
import unittest
from mock import (MagicMock, patch)
from mock import call as mcall

from dynamixel_driver import dynamixel_io as dxio
from dynamixel_driver import dynamixel_serial_proxy as dxsp
from dynamixel_msgs.msg import MotorError


_FAKE_SERVO = 0
_FAKE_PORT = 0
_FAKE_BAUDRATE = 0
_FAKE_COMMAND_FAILED = 32
_except_msg = '[Fatal Error: servo #{} on {}@{}bps]: %s failed'.format(
            _FAKE_SERVO, _FAKE_PORT, _FAKE_BAUDRATE, _FAKE_COMMAND_FAILED)


def get_dynamixel_io_mock():
    mock_dyn = MagicMock(spec=dxio.DynamixelIO)
    mock_dyn.get_feedback.side_effect = dxio.FatalErrorCodeError(_except_msg, 32)
    return  mock_dyn


def init_dynamixel_proxy():
    proxy = dxsp.SerialProxy()
    proxy.running = True
    proxy.motors = [1,2,3,4]
    return proxy


class TestDynamixelSerialProxy(unittest.TestCase):
    """Tests"""
    def __init__(self, *args):
        super(TestDynamixelSerialProxy, self).__init__(*args)
        rospy.init_node(NNAME)

    def setUp(self):
        self.proxy = init_dynamixel_proxy()
        

    def tearDown(self):
        pass

    def test_dynamixel_fatal_error_publishes_error_msg(self):
        self.proxy.dxl_io = get_dynamixel_io_mock()
        self.proxy.error_pub.publish = MagicMock(return_value=1)
       
        self.proxy._SerialProxy__process_motor_feedback(1)
        self.assertTrue(self.proxy.error_pub.publish.called)

    def test_process_motor_feedback_doesnt_publish_error_when_no_error(self):
        self.proxy.dxl_io = MagicMock(spec=dxio.DynamixelIO)
        self.proxy.error_pub.publish = MagicMock(return_value=1)
       
        self.proxy._SerialProxy__process_motor_feedback(1)
        self.assertTrue(not self.proxy.error_pub.publish.called)        


             
 
if __name__ == '__main__':

    import rostest
    rostest.rosrun(PKG, 'test_dynamixel_serial_proxy', TestDynamixelSerialProxy)