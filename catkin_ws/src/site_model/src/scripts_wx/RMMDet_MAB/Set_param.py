#! /usr/bin/env python
import rospy

if __name__=='__main__':
    rospy.init_node('Set_param')
    rospy.set_param('p_strict_1', 0.7)
    rospy.set_param('p_strict_2', 0.8)
    rospy.set_param('p_strict_3', 0.6)
    rospy.set_param('strict_selected', 0.)
    rospy.set_param('use_mab', 1)