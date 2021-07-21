#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


from sensor_msgs.msg import JointState

# from deepbots.robots.controllers.robot_emitter_receiver_csv import RobotEmitterReceiverCSV


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class PandaControl(object):
# class PandaControl(RobotEmitterReceiverCSV):
    def __init__(self):
        super().__init__()
        print("Moveit")
        moveit_commander.roscpp_initialize(sys.argv)
        print("Moveit running")
        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        arm_group = "panda_arm"
        hand_group = "hand"

        move_group = moveit_commander.MoveGroupCommander(arm_group)
        hand = moveit_commander.MoveGroupCommander(hand_group)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()

        group_names = robot.get_group_names()
        print("group_names")
        print(group_names)
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.hand = hand
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        self.joint_state_sub = rospy.Subscriber('joint_states', JointState, self.joint_state_update)

        self.current_joint_position = [0.000,-0.785, 0.0,-1.570, 0.0, 1.047, 0.0, 0.0]

    def joint_state_update(self, msg):
        for i in range(len(msg.position)):
            self.current_joint_position[i] = msg.position[i]

    def open_hand(self, wait=True):
        joint_goal = self.hand.get_current_joint_values()
        joint_goal[0] = 0.0399

        self.hand.set_goal_joint_tolerance(0.001)
        self.hand.go(joint_goal, wait=wait)

        self.hand.stop()

        current_joints = self.hand.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


    def close_hand(self, wait=True):
        joint_goal = self.hand.get_current_joint_values()
        joint_goal[0] = 0.0001

        self.hand.set_goal_joint_tolerance(0.001)
        self.hand.go(joint_goal, wait=wait)

        if (not wait ):
          return

        self.hand.stop()

        current_joints = self.hand.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


    def go_to_joint_state(self, wait=True, pos=[0, -pi/4, 0, -pi/2, 0, pi/3, 0]):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = pos[0]
        joint_goal[1] = pos[1]
        joint_goal[2] = pos[2]
        joint_goal[3] = pos[3]
        joint_goal[4] = pos[4]
        joint_goal[5] = pos[5]
        joint_goal[6] = pos[6]

        self.move_group.go(joint_goal, wait=wait)

        self.move_group.stop()

        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


    def go_to_pose_goal(self, pos, ori):
        pose_goal = geometry_msgs.msg.Pose()
        q = quaternion_from_euler(ori[0], ori[1], ori[2])
        pose_goal.orientation = geometry_msgs.msg.Quaternion(*q)
        pose_goal.position.x = pos[0]
        pose_goal.position.y = pos[1]
        pose_goal.position.z = pos[2]

        self.move_group.set_goal_orientation_tolerance(0.0001)
        self.move_group.set_goal_position_tolerance(0.0001)

        self.move_group.set_pose_target(pose_goal)

        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)


    def plan_cartesian_path(self, scale=1):
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
                                             waypoints,   # waypoints to follow
                                             0.01,        # eef_step
                                             0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction


    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

    def go_to(self, msg):
        self.go_to_joint_state(pos=msg.position)

    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)
        

def main():
  rospy.init_node('robot_control', anonymous=True)
  try:
    robot = PandaControl()
    # sub = rospy.Subscriber('go_to', JointState, robot.go_to)
    
    # robot.run()
    # Init
    robot.go_to_joint_state()
    rpose_goal = geometry_msgs.msg.Pose()
    robot.open_hand()

    robot.go_to_joint_state(pos=[0.2258382248357322, 0.048306280026470125, -0.08938837137344713, -2.604623674588572, 0.008579893902162744, 2.652744625267876, -0.6025170443616208])
    robot.close_hand()
    robot.go_to_pose_goal([0.25, 0.0, 1.0], [0, 0, 0])
    robot.open_hand()
    
    rospy.spin()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
