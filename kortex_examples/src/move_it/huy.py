#!/usr/bin/env python3
import sys
import time
import rospy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
from medbot_api.interface import add_callback

class ExampleMoveItTrajectories(object):
  def __init__(self):

    # Initialize the node
    super(ExampleMoveItTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('huy')
    add_callback("abc", self.callback_from_api)

    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        self.gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True

  def callback_from_api(self, args, kwargs: dict, output):
    # {1:2}
    pose = output["pose"]
    self.reach_joint_angles_custom(pose)

  def reach_named_position(self, target):
    arm_group = self.arm_group
    
    arm_group.set_max_velocity_scaling_factor(0.9)

    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(trajectory_message, wait=True)

  def reach_joint_angles(self, tolerance):
    arm_group = self.arm_group
    success = True

    arm_group.set_max_velocity_scaling_factor(0.9)

    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions before movement :")
    for p in joint_positions: rospy.loginfo(p)

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # Set the joint target configuration
    if self.degrees_of_freedom == 7:
      joint_positions[0] = pi/2
      joint_positions[1] = 0
      joint_positions[2] = pi/4
      joint_positions[3] = -pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
      joint_positions[6] = 0.2
    elif self.degrees_of_freedom == 6:
      joint_positions[0] = 0
      joint_positions[1] = 0
      joint_positions[2] = pi/2
      joint_positions[3] = pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
    arm_group.set_joint_value_target(joint_positions)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success

  def reach_joint_angles_custom(self, joint_angles_degrees, tolerance=0.01):
    arm_group = self.arm_group
    success = True

    arm_group.set_max_velocity_scaling_factor(0.9)

    if self.degrees_of_freedom != 6:
      rospy.logerr("This function is designed for robots with exactly 6 degrees of freedom.")
      return False
    
    joint_angles_radians = []
    for angle in joint_angles_degrees:
        if angle > 180:
            angle = angle - 360
        joint_angles_radians.append(math.radians(angle))

    joint_limits = [
        (math.radians(-155), math.radians(155)),
        (math.radians(-155), math.radians(155)),  
        (math.radians(-155), math.radians(155)),  
        (math.radians(-155), math.radians(155)),  
        (math.radians(-155), math.radians(155)), 
        (math.radians(-155), math.radians(155))  
    ]
      
    for i in range(6):
        if not (joint_limits[i][0] <= joint_angles_radians[i] <= joint_limits[i][1]):
          rospy.logerr(f"Joint angle {i+1} out of bounds: {joint_angles_radians[i]}")
          return False

    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions before movement :")
    for p in joint_positions: rospy.loginfo(p)
      
    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)
   
    # Set the joint target configuration
    for i in range(6):
      joint_positions[i] = joint_angles_radians[i]
    
    arm_group.set_joint_value_target(joint_positions)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success


  def get_cartesian_pose(self):
    arm_group = self.arm_group
    
    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    arm_group.set_max_velocity_scaling_factor(0.9)

    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False 

def main():
  example = ExampleMoveItTrajectories()

  # For testing purposes
  success = example.is_init_success
  try:
      rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  except:
      pass
  """
  #Pst1
  if success:
    rospy.loginfo("Reaching Named Target Vertical...")
    success &= example.reach_named_position("vertical")
    print (success)
  """
  for _ in range(1):
    #Pst2
    if success:
      rospy.loginfo("Reaching Named Target Home...")
      success &= example.reach_named_position("home")
      print (success)

    #Pst3
    if example.is_gripper_present and success:
      rospy.loginfo("Opening the gripper...")
      success &= example.reach_gripper_position(1.0)
      print (success)

    #Pst4
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [50.47,290.03,81.16,44.97,291.58,287.6]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

    #Pst5
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [53.64,270.57,61.12,48.07,293.61,287.01]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)
    
    #Pst6
    if example.is_gripper_present and success:
      rospy.loginfo("Opening the gripper...")
      success &= example.reach_gripper_position(0.3)
      print (success)
    
    #Pst7
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [53.8,286.39,73.8,47.14,295.61,289.14]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

    #Pst8
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [17.86,292.59,77.2,13.56,278.24,119.99]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

    #Pst9
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [17.79,271.53,61.51,14.12,276.98,114.88]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)
    
    #Pst10
    if example.is_gripper_present and success:
      rospy.loginfo("Closing the gripper 50%...")
      success &= example.reach_gripper_position(0.9)
      print (success)

    #Pst11
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [13.06,289.9,78.11,9.93,274.81,116.96]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

    #Pst12
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [62.74,319.93,137.57,60.62,270.9,270.25]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

    #Pst13 
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [74.89,294.49,105.17,72.56,277.88,271.92]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

    #Pst14 
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [77.28,282.08,85.07,74.58,285.45,273.63]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)
    
    #Pst15
    if example.is_gripper_present and success:
      rospy.loginfo("Opening the gripper...")
      success &= example.reach_gripper_position(0)
      print (success)

    #Pst16 
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [75.36,292.44,100.38,72.93,280.67,272.63]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

    #Pst17 
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [29.98,302.8,112.86,27.53,274.13,312.59]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

    #Pst18 
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [28.8,292.31,104.13,26.51,273.09,15.63]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

      rospy.sleep(5)

    #Pst19
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [26.23,305.6,100.03,22.15,279.77,290.5]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

    #Pst20 
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [75.36,292.44,100.38,72.93,280.67,272.63]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

    #Pst21 
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [77.28,282.08,85.07,74.58,285.45,273.63]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

    #Pst22
    if example.is_gripper_present and success:
      rospy.loginfo("Opening the gripper...")
      success &= example.reach_gripper_position(0.9)
      print (success)

    #Pst23
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [69.36,301.59,115.99,67.12,274.35,271.37]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

    #Pst24
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [12.8,290.48,76.96,9.29,275.24,121.1]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

    #Pst25
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [17.98,270.35,59.61,13.96,277.45,117.94]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

    #Pst26
    if example.is_gripper_present and success:
      rospy.loginfo("Opening the gripper...")
      success &= example.reach_gripper_position(0.3)
      print (success)

    #Pst27
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [14.14,296.25,79.7,10.01,276.55,124.07]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

    #Pst28
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [346.12,298.54,110.75,345.52,268.29,93.16]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

    #Pst29
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [338.52,291.49,121.13,336.54,273.3,332.19]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

      rospy.sleep(5)
  
    #Pst30
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [338.88,295.51,126.33,337.03,273.86,311.8]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

      rospy.sleep(3)

    #Pst31
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [338.52,291.49,121.13,336.54,273.3,332.19]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

      rospy.sleep(1)

    #Pst32
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [2.49,318.89,117.1,0.45,269.6,108.9]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

    #Pst33
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [54.87,288.29,74.34,47.6,296.03,288.19]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

    #Pst34
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [54.17,267.66,57.35,47.26,289.5,289.52]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)

    #Pst35
    if example.is_gripper_present and success:
      rospy.loginfo("Closing the gripper 50%...")
      success &= example.reach_gripper_position(0.9)
      print (success)

    #Pst36
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [48.79,289.2,82.69,43.59,288.46,284.92]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)
    
    #Pst37
    if example.degrees_of_freedom == 6 and success:
      rospy.loginfo("Reaching custom joint angles...")
      custom_joint_angles = [357.01,21.02,150.11,272.03,320,273.01]
      success &= example.reach_joint_angles_custom(custom_joint_angles)
      print(success)
      
    if example.is_gripper_present and success:
      rospy.loginfo("Opening the gripper...")
      success &= example.reach_gripper_position(0)
      print (success)
    
  # For testing purposes
  rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

  if not success:
      rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
  main()
