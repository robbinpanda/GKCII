#!/usr/bin/python3
import rospy
from gazebo_msgs.msg import ModelStates

model_name = 'car1'
model_index = None


def get_model_index(msg):
    global model_name
    for i,name in enumerate(msg.name):
        if model_name==name:
            return i


def callback(msg):
    global model_index
    if not model_index:
        model_index = get_model_index(msg)
    model_pos = msg.pose[model_index].position
    print('------------------------')
    print('Model_Name: ', model_name)
    print('x: ', model_pos.x)
    print('y: ', model_pos.y)
    print('z: ', model_pos.z)


if __name__=='__main__':
    rospy.init_node("model_state_monitor", anonymous=True)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)

    rospy.spin()
