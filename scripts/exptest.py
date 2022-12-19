#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2018 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
import math
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node("pose_groupstate_example")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.5)
    arm.set_max_acceleration_scaling_factor(0.15)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())

    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)
    # アーム稼働
    gripper.set_joint_value_target([1, 1])
    gripper.go()
    # 手動で姿勢を指定するには以下のように指定
    # 0.624がアームの距離 
    # 一回目の移動
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0
    target_pose.position.y = -0.3
    target_pose.position.z = 0.3
    q = quaternion_from_euler( 3.14/2, 0, 0 )
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target( target_pose )  # 目標ポーズ設定
    arm.go()

    # 一回目の移動
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.15
    target_pose.position.y = -0.259
    target_pose.position.z = 0.3
    q = quaternion_from_euler( 3.14/2, 0, 0 )
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()

    # 一回目の移動
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.259
    target_pose.position.y = -0.15
    target_pose.position.z = 0.3
    q = quaternion_from_euler( 3.14/2, 0, 0 )
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target( target_pose )  # 目標ポーズ設定
    arm.go()

    # 一回目の移動
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.3
    target_pose.position.y = 0
    target_pose.position.z = 0.3
    q = quaternion_from_euler( 3.14/2, 0, 0 )
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target( target_pose )  # 目標ポーズ設定
    arm.go()

    # 一回目の移動
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.259
    target_pose.position.y = 0.15
    target_pose.position.z = 0.3
    q = quaternion_from_euler( 3.14/2, 0, 0 )
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target( target_pose )  # 目標ポーズ設定
    arm.go()

    # 一回目の移動
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.15
    target_pose.position.y = 0.259
    target_pose.position.z = 0.3
    q = quaternion_from_euler( 3.14/2, 0, 0 )
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target( target_pose )  # 目標ポーズ設定
    arm.go()

    # 一回目の移動
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0
    target_pose.position.y = 0.3
    target_pose.position.z = 0.3
    q = quaternion_from_euler( 3.14/2, 0, 0 )
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target( target_pose )  # 目標ポーズ設定
    arm.go()

    # 移動後の手先ポーズを表示
    arm_goal_pose = arm.get_current_pose().pose
    print("Arm goal pose:")
    print(arm_goal_pose)
    print("done")


if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass

