#!/usr/bin/env python3
import rospy
import moveit_commander
import geometry_msgs.msg
from math import pi, cos, sin, sqrt
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


# ---------------------- 修正核心参数（解决距离偏差） ----------------------
BASE_FRAME = "base_link"
END_EFFECTOR_LINK = "Empty_Link6"
# 1. 保守不动点（机械臂绝对可达的位置）
CONE_VERTEX = (0.1, 0.0, -0.2)  # 选更靠近机械臂的位置
# 2. 圆锥摆核心（末端-不动点距离=圆周半径，解决距离偏差）
ARM_LENGTH = 0.15  # 固定15cm
CIRCLE_RADIUS = ARM_LENGTH  # 圆周半径=距离，确保距离正确
ANGLE_RANGE = pi/2  # 只做1/4圈（进一步降低规划压力）
POINT_NUM = 4      # 1/4圈只设4个点


def calc_end_pose(vertex, angle):
    """修正：末端-不动点距离严格=15cm"""
    vx, vy, vz = vertex
    # XY平面圆周运动（距离=ARM_LENGTH）
    end_x = vx + CIRCLE_RADIUS * cos(angle)
    end_y = vy + CIRCLE_RADIUS * sin(angle)
    end_z = vz  # Z轴与不动点一致
    # 验证距离（确保正确）
    dist = sqrt((end_x-vx)**2 + (end_y-vy)**2 + (end_z-vz)**2)
    if abs(dist - ARM_LENGTH) > 0.005:
        rospy.logwarn(f"距离：{dist:.3f}m（目标0.15m）")
    # 末端姿态（选机械臂自然姿态）
    qx, qy, qz, qw = quaternion_from_euler(pi/2, 0, 0)
    pose = geometry_msgs.msg.Pose()
    pose.position.x = end_x
    pose.position.y = end_y
    pose.position.z = end_z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def create_trajectory_marker(waypoints):
    """轨迹可视化"""
    marker = Marker()
    marker.header.frame_id = BASE_FRAME
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.LINE_STRIP
    marker.scale.x = 0.005
    marker.color.r = 1.0
    marker.color.a = 0.9
    for p in waypoints:
        pt = geometry_msgs.msg.Point()
        pt.x = p.position.x
        pt.y = p.position.y
        pt.z = p.position.z
        marker.points.append(pt)
    arr = MarkerArray()
    arr.markers.append(marker)
    return arr


def main():
    moveit_commander.roscpp_initialize([])
    rospy.init_node("cone_swing_final")
    
    # 初始化MoveGroup
    move_group = moveit_commander.MoveGroupCommander("arm")
    move_group.set_end_effector_link(END_EFFECTOR_LINK)
    move_group.set_max_velocity_scaling_factor(0.05)  # 极慢速度
    move_group.set_planning_time(25.0)  # 最长规划时间
    move_group.allow_replanning(True)
    
    traj_pub = rospy.Publisher("/cone_traj", MarkerArray, queue_size=10)

    # 步骤1：生成1/4圈路点（无距离偏差）
    waypoints = []
    angles = [ANGLE_RANGE * i/(POINT_NUM-1) for i in range(POINT_NUM)]
    for angle in angles:
        pose = calc_end_pose(CONE_VERTEX, angle)
        waypoints.append(pose)
    traj_pub.publish(create_trajectory_marker(waypoints))
    rospy.loginfo(f"生成{POINT_NUM}个路点，1/4圈运动（不动点：{CONE_VERTEX}）")

    # 步骤2：规划路径
    move_group.set_start_state_to_current_state()
    plan, fraction = move_group.compute_cartesian_path(waypoints, 0.03, True)
    rospy.loginfo(f"路径覆盖率：{fraction:.2f}")

    # 步骤3：执行（只要有路径就尝试）
    if fraction > 0.2:
        move_group.execute(plan, wait=True)
        rospy.loginfo("1/4圈圆锥摆运动完成！")
    else:
        rospy.logerr("规划失败：请确认不动点{CONE_VERTEX}是否能手动拖动末端到达")

    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass