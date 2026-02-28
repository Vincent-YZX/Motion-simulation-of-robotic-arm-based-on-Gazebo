#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped

def publish_end_pose():
    rospy.init_node('end_pose_publisher')
    pub = rospy.Publisher('/arm_end_effector_pose', PoseStamped, queue_size=10)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(10)  # 10Hz发布

    # 替换为你的末端链接名（比如Empty_Link6）
    END_LINK = "Empty_Link6"

    while not rospy.is_shutdown():
        try:
            # 监听末端链接相对于world的位姿
            trans: TransformStamped = tf_buffer.lookup_transform(
                "world", END_LINK, rospy.Time(0), rospy.Duration(1.0)
            )
            # 转换成PoseStamped话题
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation
            pub.publish(pose)
        except Exception as e:
            rospy.logwarn("获取末端位姿失败：{}".format(e))
        rate.sleep()

if __name__ == "__main__":
    try:
        publish_end_pose()
    except rospy.ROSInterruptException:
        pass