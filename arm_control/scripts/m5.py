#!/usr/bin/env python3
import rospy
import moveit_commander
import geometry_msgs.msg
from math import pi, cos, sin, radians
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


# ---------------------- 核心配置（阿基米德开放螺旋） ----------------------
BASE_FRAME = "base_link"
END_EFFECTOR_LINK = "Empty_Link6"
SAFE_X = 0.0
SAFE_Y = 0.0
SAFE_Z = -0.5
SPIRAL_CYCLES_PER_PLAN = 3  # 规划圈数
SPIRAL_POINTS_PER_CYCLE = 12  # 每圈12个点，保证螺旋平滑
SPIRAL_GROWTH_RATE = 0.001  # 螺旋增长率（每弧度增加0.001m）
TARGET_MAX_R = 0.04  # 最大半径（避免超出关节冗余）
ORI_TOLERANCE = radians(8)  # 保留8°姿态公差
POSE_TOLERANCE = 0.1
VEL = 0.1
CALIBRATE_CYCLE = 1  # 每规划1次校准姿态


def get_vertical_quaternion():
    """垂直向上的基准四元数"""
    return quaternion_from_euler(0, 0, 0)


def create_thin_white_line_trajectory(waypoints, marker_id=0):
    """生成细白色线轨迹（线宽0.005m）"""
    line_marker = Marker()
    line_marker.header.frame_id = BASE_FRAME
    line_marker.header.stamp = rospy.Time.now()
    line_marker.id = marker_id
    line_marker.type = Marker.LINE_STRIP  # 连续线
    line_marker.action = Marker.ADD
    line_marker.scale.x = 0.005  # 线宽调细到5mm
    line_marker.color.r = 1.0
    line_marker.color.g = 1.0
    line_marker.color.b = 1.0
    line_marker.color.a = 0.9
    line_marker.lifetime = rospy.Duration()  # 永久显示
    # 填充螺旋线的点
    for pose in waypoints:
        point = geometry_msgs.msg.Point()
        point.x = pose.position.x
        point.y = pose.position.y
        point.z = pose.position.z
        line_marker.points.append(point)
    # 封装为MarkerArray
    marker_array = MarkerArray()
    marker_array.markers.append(line_marker)
    return marker_array


def publish_end_effector_pose(move_group, pub):
    """发布末端实时位姿"""
    end_effector_pose = move_group.get_current_pose()
    pub.publish(end_effector_pose)


def calibrate_vertical_pose(move_group):
    """姿态校准：回归垂直基准"""
    rospy.loginfo("开始姿态校准...")
    target_pose = geometry_msgs.msg.Pose()
    qx, qy, qz, qw = get_vertical_quaternion()
    target_pose.orientation.x = qx
    target_pose.orientation.y = qy
    target_pose.orientation.z = qz
    target_pose.orientation.w = qw
    target_pose.position.x = SAFE_X
    target_pose.position.y = SAFE_Y
    target_pose.position.z = SAFE_Z
    move_group.set_pose_target(target_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    rospy.loginfo(" 姿态校准完成")


def main():
    moveit_commander.roscpp_initialize([])
    rospy.init_node("arm_archimedean_spiral")
    
    # RViz发布器
    traj_pub = rospy.Publisher("/spiral_trajectory", MarkerArray, queue_size=10)
    end_effector_pub = rospy.Publisher("/end_effector_pose", geometry_msgs.msg.PoseStamped, queue_size=10)

    # MoveGroup初始化
    move_group = moveit_commander.MoveGroupCommander("arm")
    move_group.set_end_effector_link(END_EFFECTOR_LINK)
    move_group.set_pose_reference_frame(BASE_FRAME)
    move_group.set_max_velocity_scaling_factor(VEL)
    move_group.set_goal_position_tolerance(POSE_TOLERANCE)
    move_group.set_goal_orientation_tolerance(ORI_TOLERANCE)
    move_group.allow_replanning(True)
    move_group.set_planning_time(15.0)

    # 初始位姿校准
    calibrate_vertical_pose(move_group)
    current_initial_r = 0.01  # 每轮螺旋的初始半径
    plan_count = 0
    line_marker_id = 0
    rate = rospy.Rate(1)

    rospy.loginfo(f"开始执行平面螺旋运动（目标最大半径{TARGET_MAX_R}m）...")
    while not rospy.is_shutdown() and current_initial_r + SPIRAL_GROWTH_RATE * (SPIRAL_CYCLES_PER_PLAN*2*pi) <= TARGET_MAX_R:
        # 每规划1次校准姿态
        if plan_count % CALIBRATE_CYCLE == 0 and plan_count != 0:
            calibrate_vertical_pose(move_group)
            line_marker_id += 1

        # 1. 生成阿基米德开放螺旋路点
        current_pose = move_group.get_current_pose().pose
        spiral_center_x = current_pose.position.x
        spiral_center_y = current_pose.position.y
        waypoints = []
        total_points = SPIRAL_CYCLES_PER_PLAN * SPIRAL_POINTS_PER_CYCLE
        # 角度范围：
        theta_max = SPIRAL_CYCLES_PER_PLAN * 2 * pi
        theta_step = theta_max / total_points
        qx, qy, qz, qw = get_vertical_quaternion()
        
        for i in range(total_points):
            theta = i * theta_step
            # 阿基米德螺旋公式：r = 初始半径 + 增长率·θ（半径随角度递增）
            current_r = current_initial_r + SPIRAL_GROWTH_RATE * theta
            # 计算XY位置
            spiral_pose = geometry_msgs.msg.Pose()
            spiral_pose.orientation.x = qx
            spiral_pose.orientation.y = qy
            spiral_pose.orientation.z = qz
            spiral_pose.orientation.w = qw
            spiral_pose.position.x = spiral_center_x + current_r * cos(theta)
            spiral_pose.position.y = spiral_center_y + current_r * sin(theta)
            spiral_pose.position.z = SAFE_Z  # Z固定，保证平面
            waypoints.append(spiral_pose)

        # 2. 发布细白线轨迹
        thin_white_line = create_thin_white_line_trajectory(waypoints, line_marker_id)
        traj_pub.publish(thin_white_line)
        rospy.loginfo(f"RViz细白线轨迹已显示（3圈开放螺旋，ID={line_marker_id}）")

        # 3. 规划+执行螺旋
        move_group.set_start_state_to_current_state()
        plan, fraction = move_group.compute_cartesian_path(waypoints, 0.02, True, None)
        rospy.loginfo(f"当前初始半径：{current_initial_r:.3f}m | 路径覆盖率：{fraction:.2f}")

        if fraction >= 0.8:
            move_group.execute(plan, wait=True)
            move_group.stop()
            rospy.loginfo(f"完成{SPIRAL_CYCLES_PER_PLAN}圈开放螺旋")
            publish_end_effector_pose(move_group, end_effector_pub)
            # 更新下一轮的初始半径（衔接当前螺旋的终点半径）
            current_initial_r = current_initial_r + SPIRAL_GROWTH_RATE * theta_max
            plan_count += 1
            line_marker_id += 1
        else:
            rospy.logwarn("覆盖率不足，保持当前初始半径")
        
        rate.sleep()




if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()