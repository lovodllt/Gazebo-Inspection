#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def auto_inspect():
    rospy.init_node('auto_inspection', anonymous=True)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(3)  # 等待导航初始化

    # ########## 关键：修改为你的Gazebo场景中的实际坐标（用RViz看地图坐标） ##########
    inspect_points = [
        (4.2, 0.0, 0.0),
        (3.5, 0.8, 3.14),
        (2.5, 1.5, 2.70),
        (1.5, 0.75, 3.14),
        (0.6, 1.0, 3.14),
        (0.6, 1.0, 1.57),
        (1.2, 0.75, 0.0),
        (2.2, 1.5, 1.57),
        (3.2, 0.8, 0.0),
        (3.8, 1.3, 0.0),
        (3.8, 1.3, 1.57)
    ]

    # 循环发布巡检点
    while not rospy.is_shutdown():
        for (x, y, yaw) in inspect_points:
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()
            # 位置
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0
            # 朝向（四元数）
            goal.pose.orientation.z = yaw / 2.0
            goal.pose.orientation.w = 1.0 - (yaw**2)/8.0

            rospy.loginfo(f"前往巡检点：x={x}, y={y}")
            goal_pub.publish(goal)
            rospy.sleep(10)  # 等待15秒（根据机器人速度调整）

if __name__ == '__main__':
    try:
        auto_inspect()
    except rospy.ROSInterruptException:
        rospy.loginfo("巡检结束")
