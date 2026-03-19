import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  # Set the path to this package.
  pkg_share = FindPackageShare(package='md_controller').find('md_controller')

  # Set the path to the RViz configuration settings
  default_rviz_config_path = os.path.join(pkg_share, 'rviz/md.rviz')

  ########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############  
  # Launch configuration variables specific to simulation
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  use_rviz = LaunchConfiguration('use_rviz')
  publish_camera_static_tf = LaunchConfiguration('publish_camera_static_tf')
  camera_tf_parent = LaunchConfiguration('camera_tf_parent')
  camera_tf_child = LaunchConfiguration('camera_tf_child')
  camera_tf_x = LaunchConfiguration('camera_tf_x')
  camera_tf_y = LaunchConfiguration('camera_tf_y')
  camera_tf_z = LaunchConfiguration('camera_tf_z')
  camera_tf_yaw = LaunchConfiguration('camera_tf_yaw')
  camera_tf_pitch = LaunchConfiguration('camera_tf_pitch')
  camera_tf_roll = LaunchConfiguration('camera_tf_roll')
    
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='False',
    description='Whether to start RVIZ')

  declare_publish_camera_static_tf_cmd = DeclareLaunchArgument(
    name='publish_camera_static_tf',
    default_value='True',
    description='Whether to publish static TF from base_link to camera_link')

  declare_camera_tf_parent_cmd = DeclareLaunchArgument(
    name='camera_tf_parent',
    default_value='base_link',
    description='Parent frame for camera static TF')

  declare_camera_tf_child_cmd = DeclareLaunchArgument(
    name='camera_tf_child',
    default_value='camera_link',
    description='Child frame for camera static TF')

  declare_camera_tf_x_cmd = DeclareLaunchArgument(
    name='camera_tf_x',
    default_value='0.29',
    description='Static TF X (m): base_link -> camera_link')

  declare_camera_tf_y_cmd = DeclareLaunchArgument(
    name='camera_tf_y',
    default_value='0.0',
    description='Static TF Y (m): base_link -> camera_link')

  declare_camera_tf_z_cmd = DeclareLaunchArgument(
    name='camera_tf_z',
    default_value='0.53',
    description='Static TF Z (m): base_link -> camera_link')

  declare_camera_tf_yaw_cmd = DeclareLaunchArgument(
    name='camera_tf_yaw',
    default_value='0.0',
    description='Static TF yaw (rad): base_link -> camera_link')

  declare_camera_tf_pitch_cmd = DeclareLaunchArgument(
    name='camera_tf_pitch',
    default_value='0.0',
    description='Static TF pitch (rad): base_link -> camera_link')

  declare_camera_tf_roll_cmd = DeclareLaunchArgument(
    name='camera_tf_roll',
    default_value='0.0',
    description='Static TF roll (rad): base_link -> camera_link')
  
  # Launch motor driver controller
  md_controller_cmd = Node(
    package='md_controller',
    executable='md_controller',
    parameters=[{
      "MDUI":184,
      "MDT":183,
      "Port":"/dev/ttyMotorLeft",
      "Baudrate":57600,
      "ID":1,                  # Left (A) driver ID
      "GearRatio":4.33,
      "poles":20,
      "left_sign":1,
      "left_cmd_gain":1.0,      # Left command gain (speed asymmetry compensation)
      "left_odom_gain":1.0,     # Left odom gain (distance/yaw calibration)
      "left_front_cmd_gain":1.0,  # Left front channel gain
      "left_rear_cmd_gain":1.0,   # Left rear channel gain
      "right_enabled":True,
      "RightID":1,             # Right (B) driver ID (separate port, same ID allowed)
      "RightMDT":183,
      "RightGearRatio":4.33,
      "right_sign":1,
      "right_cmd_gain":1.0,     # Right command gain (speed asymmetry compensation)
      "right_odom_gain":1.0,    # Right odom gain (distance/yaw calibration)
      "right_front_cmd_gain":1.0, # Right front channel gain
      "right_rear_cmd_gain":1.0,  # Right rear channel gain
      "RightUseSeparatePort":True,
      "RightPort":"/dev/ttyMotorRight",
      "RightBaudrate":57600,
      "cmd_timeout_ms":300,
      "max_driver_rpm":700,
      "wheel_radius": 0.100,
      "wheel_base": 0.440,
      "use_imu_yaw_correction": True,
      "imu_topic": "/camera/camera/imu",
      "imu_yaw_weight": 0.7,
      "imu_yaw_rate_gain": 1.0,
      "imu_lowpass_alpha": 0.2,
      "imu_timeout_sec": 0.2,
      "publish_odom_tf": True,
      "odom_frame_id": "odom",
      "base_frame_id": "base_link"
    }],
    output='screen'
  )

  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])

  # Publish base_link -> camera_link static transform
  camera_static_tf_cmd = Node(
    condition=IfCondition(publish_camera_static_tf),
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_to_camera_static_tf',
    output='screen',
    arguments=[
      camera_tf_x,
      camera_tf_y,
      camera_tf_z,
      camera_tf_yaw,
      camera_tf_pitch,
      camera_tf_roll,
      camera_tf_parent,
      camera_tf_child
    ])
  
  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_use_rviz_cmd)
  ld.add_action(declare_publish_camera_static_tf_cmd)
  ld.add_action(declare_camera_tf_parent_cmd)
  ld.add_action(declare_camera_tf_child_cmd)
  ld.add_action(declare_camera_tf_x_cmd)
  ld.add_action(declare_camera_tf_y_cmd)
  ld.add_action(declare_camera_tf_z_cmd)
  ld.add_action(declare_camera_tf_yaw_cmd)
  ld.add_action(declare_camera_tf_pitch_cmd)
  ld.add_action(declare_camera_tf_roll_cmd)

  # Add any actions
  ld.add_action(md_controller_cmd)
  ld.add_action(camera_static_tf_cmd)
  ld.add_action(start_rviz_cmd)

  return ld
