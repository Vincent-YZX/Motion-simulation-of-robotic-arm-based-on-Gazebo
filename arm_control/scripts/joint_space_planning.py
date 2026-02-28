#!/usr/bin/env python3
import sys
import rospy
import moveit_commander

def joint_space_simple():
    # 初始化
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('joint_simple')
    arm = moveit_commander.MoveGroupCommander("arm")  # 规划组名

    # 设置关节目标（joint1~joint6，弧度）
    arm.set_joint_value_target([1, 2, -0.2, 0.0, 0.1, 0.0])
    
    # 直接规划+执行（避免轨迹状态偏差）
    arm.go(wait=True)
    arm.stop()  # 确保执行后停止

    # 清理
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    try:
        joint_space_simple()
    except Exception as e:
        print(f"Error: {e}")