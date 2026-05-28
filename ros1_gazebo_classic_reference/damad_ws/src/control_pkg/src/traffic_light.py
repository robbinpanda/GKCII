#! /usr/bin/env python3
import rospy
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import SetLinkState, GetLinkState
from std_msgs.msg import String, Header
import time

#改模型红绿灯位置
#切换信号时先把之前信号模型调换复原再修改新的信号灯
def traffic_light_change(x, y, signal):
    #红绿灯太多影响程序运行，暂时只留cantilevered_light_0_0
    if x > 0 or y > 0:
        return
    
    
    # 根据当前信号选择需要修改的link名
    if signal == 0:
        link_name0 = 'cantilevered_light_%d_%d::traffic_light::left_off'%(x, y)
        link_name1 = 'cantilevered_light_%d_%d::traffic_light::left_on'%(x, y)
    elif signal == 1:
        link_name0 = 'cantilevered_light_%d_%d::traffic_light::middle_off'%(x, y)
        link_name1 = 'cantilevered_light_%d_%d::traffic_light::middle_on'%(x, y)
    elif signal == 2:
        link_name0 = 'cantilevered_light_%d_%d::traffic_light::right_off'%(x, y)
        link_name1 = 'cantilevered_light_%d_%d::traffic_light::right_on'%(x, y)
    
    # 获取左/中/右的红绿灯模型link state
    current_state0 = get_link_state_proxy(link_name0, reference_frame)
    link_state0 = LinkState()
    link_state0.link_name = link_name0
    current_state1 = get_link_state_proxy(link_name1, reference_frame)
    link_state1 = LinkState()
    link_state1.link_name = link_name1
    
    # 调换左/中/右的红绿灯模型
    link_state0.pose = current_state1.link_state.pose
    link_state0.twist = current_state0.link_state.twist
    link_state0.reference_frame = reference_frame
    set_link_state_proxy(link_state0)
    
    link_state1.pose = current_state0.link_state.pose
    link_state1.twist = current_state1.link_state.twist
    link_state1.reference_frame = reference_frame
    set_link_state_proxy(link_state1)

#定时切换左中右信号
def traffic_light_signal():
    global currentSignal, lastTime, signals
    if time.time() - lastTime >= changeRate:
        for i in range(len(signals)):
            for j in range(len(signals[0])):
                traffic_light_change(i, j, signals[i][j])
        currentSignal = (currentSignal + 1) % 3
        for i in range(len(signals)):
            for j in range(len(signals[0])):
                traffic_light_change(i, j, currentSignal)
                signals[i][j] = currentSignal
        lastTime = time.time()
        pub.publish(str(signals))
        rospy.loginfo("Current Traffic Light is " + str(signals))
        """traffic_light_change(0, 0, currentSignal)
        currentSignal = (currentSignal + 1) % 3
        traffic_light_change(0, 0, currentSignal)
        lastTime = time.time()
    pub.publish("%d_%d_%d"%(0, 0, currentSignal))
    rospy.loginfo("Current Traffic Light is " + "%d_%d_%d"%(0, 0, currentSignal))"""

if __name__ == '__main__':
    # 保存每个信号灯目前信号
    signals = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
    changeRate = 5 #5秒变信号
    currentSignal = 0 #0左 1中 2右
    lastTime = time.time()

    rospy.init_node("traffic_light")
    pub = rospy.Publisher("traffic_light", String,queue_size=1)

    # 等待并调用gazebo中get_link_state和set_link_state服务
    rospy.wait_for_service('/gazebo/get_link_state')
    rospy.wait_for_service('/gazebo/set_link_state')
    get_link_state_proxy= rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    set_link_state_proxy = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
    reference_frame = 'world'
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        traffic_light_signal()
        #signal = "1"
        #rospy.loginfo("Current Traffic Light is "+signal)
        #pub.publish(signal)
        rate.sleep()
    #rospy.spin()
    #pass
