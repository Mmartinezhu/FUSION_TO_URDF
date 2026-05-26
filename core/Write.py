# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:46:26 2019

@author: mmartinezhu
"""

import adsk, os
from xml.etree.ElementTree import Element, SubElement
from . import Link, Joint
from ..utils import utils


def _mkdir(path):
    try:
        os.mkdir(path)
    except:
        pass


def _remove_if_exists(path):
    try:
        if os.path.exists(path):
            os.remove(path)
    except:
        pass


def _movable_joint_names(joints_dict):
    return [j for j in joints_dict if joints_dict[j]['type'] != 'fixed']


def _write_material_xml(f):
    f.write('<material name="silver">\n')
    f.write('  <color rgba="0.700 0.700 0.700 1.000"/>\n')
    f.write('</material>\n')
    f.write('\n')


def _write_indented_xml(f, xml, indent):
    prefix = ' ' * indent
    for line in xml.splitlines():
        if line:
            f.write(prefix + line + '\n')


def write_link_urdf(joints_dict, repo, links_xyz_dict, file_name, inertial_dict):
    """
    Write links information into urdf "repo/file_name"


    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    repo: str
        the name of the repository to save the xml file
    links_xyz_dict: vacant dict
        xyz information of the each link
    file_name: str
        urdf full path
    inertial_dict:
        information of the each inertial

    Note
    ----------
    In this function, links_xyz_dict is set for write_joint_tran_urdf.
    The origin of the coordinate of center_of_mass is the coordinate of the link
    """
    with open(file_name, mode='a') as f:
        # for base_link
        center_of_mass = inertial_dict['base_link']['center_of_mass']
        link = Link.Link(name='base_link', xyz=[0, 0, 0],
            center_of_mass=center_of_mass, repo=repo,
            mass=inertial_dict['base_link']['mass'],
            inertia_tensor=inertial_dict['base_link']['inertia'])
        links_xyz_dict[link.name] = link.xyz
        link.make_link_xml()
        f.write(link.link_xml)
        f.write('\n')

        # others
        for joint in joints_dict:
            name = joints_dict[joint]['child']
            center_of_mass = \
                [i-j for i, j in zip(inertial_dict[name]['center_of_mass'], joints_dict[joint]['xyz'])]
            link = Link.Link(name=name, xyz=joints_dict[joint]['xyz'],
                center_of_mass=center_of_mass,
                repo=repo, mass=inertial_dict[name]['mass'],
                inertia_tensor=inertial_dict[name]['inertia'])
            links_xyz_dict[link.name] = link.xyz
            link.make_link_xml()
            f.write(link.link_xml)
            f.write('\n')


def write_joint_urdf(joints_dict, repo, links_xyz_dict, file_name):
    """
    Write joints information into urdf "repo/file_name"


    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    repo: str
        the name of the repository to save the xml file
    links_xyz_dict: dict
        xyz information of the each link
    file_name: str
        urdf full path
    """

    with open(file_name, mode='a') as f:
        for j in joints_dict:
            parent = joints_dict[j]['parent']
            child = joints_dict[j]['child']
            joint_type = joints_dict[j]['type']
            upper_limit = joints_dict[j]['upper_limit']
            lower_limit = joints_dict[j]['lower_limit']
            try:
                xyz = [round(p-c, 6) for p, c in
                    zip(links_xyz_dict[parent], links_xyz_dict[child])]  # xyz = parent - child
            except KeyError:
                app = adsk.core.Application.get()
                ui = app.userInterface
                ui.messageBox("There seems to be an error with the connection between\n\n%s\nand\n%s\n\nCheck \
whether the connections\nparent=component2=%s\nchild=component1=%s\nare correct or if you need \
to swap component1<=>component2"
                % (parent, child, parent, child), "Error!")
                quit()

            joint = Joint.Joint(name=j, joint_type=joint_type, xyz=xyz,
            axis=joints_dict[j]['axis'], parent=parent, child=child,
            upper_limit=upper_limit, lower_limit=lower_limit)
            joint.make_joint_xml()
            f.write(joint.joint_xml)
            f.write('\n')


def write_gazebo_endtag(file_name):
    """
    Write the </robot> tag at the end of the urdf


    Parameters
    ----------
    file_name: str
        urdf full path
    """
    with open(file_name, mode='a') as f:
        f.write('</robot>\n')


def write_urdf(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    _mkdir(save_dir + '/urdf')
    _remove_if_exists(save_dir + '/urdf/{}.trans'.format(robot_name))
    _remove_if_exists(save_dir + '/urdf/{}.gazebo'.format(robot_name))

    repo = package_name + '/meshes/'

    xacro_file_name = save_dir + '/urdf/' + robot_name + '.xacro'
    with open(xacro_file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'.format(robot_name))
        f.write('\n')
        f.write('<xacro:include filename="materials.xacro" />\n')
        f.write('<xacro:include filename="{}.ros2_control.xacro" />\n'.format(robot_name))
        f.write('<xacro:include filename="{}.gazebo.xacro" />\n'.format(robot_name))
        f.write('\n')

    write_link_urdf(joints_dict, repo, links_xyz_dict, xacro_file_name, inertial_dict)
    write_joint_urdf(joints_dict, repo, links_xyz_dict, xacro_file_name)
    write_gazebo_endtag(xacro_file_name)

    urdf_file_name = save_dir + '/urdf/' + robot_name + '.urdf'
    pure_links_xyz_dict = {}
    with open(urdf_file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}">\n'.format(robot_name))
        f.write('\n')
        _write_material_xml(f)

    write_link_urdf(joints_dict, repo, pure_links_xyz_dict, urdf_file_name, inertial_dict)
    write_joint_urdf(joints_dict, repo, pure_links_xyz_dict, urdf_file_name)
    write_gazebo_endtag(urdf_file_name)


def write_materials_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    _mkdir(save_dir + '/urdf')

    file_name = save_dir + '/urdf/materials.xacro'
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(robot_name))
        f.write('\n')
        _write_material_xml(f)
        f.write('</robot>\n')


def write_transmissions_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    """
    Write ROS 2 control information into a xacro fragment.
    """

    file_name = save_dir + '/urdf/{}.ros2_control.xacro'.format(robot_name)
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(robot_name))
        f.write('\n')

        movable_joints = _movable_joint_names(joints_dict)
        if movable_joints:
            f.write('<ros2_control name="{}_system" type="system">\n'.format(robot_name))
            f.write('  <hardware>\n')
            f.write('    <plugin>gz_ros2_control/GazeboSimSystem</plugin>\n')
            f.write('  </hardware>\n')

            for j in movable_joints:
                joint = Joint.Joint(name=j, joint_type=joints_dict[j]['type'], xyz=[0, 0, 0],
                axis=joints_dict[j]['axis'], parent=joints_dict[j]['parent'],
                child=joints_dict[j]['child'], upper_limit=joints_dict[j]['upper_limit'],
                lower_limit=joints_dict[j]['lower_limit'])
                joint.make_ros2_control_xml()
                _write_indented_xml(f, joint.ros2_control_xml, 2)

            f.write('</ros2_control>\n')
            f.write('\n')

        f.write('</robot>\n')


def write_gazebo_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    _mkdir(save_dir + '/urdf')

    file_name = save_dir + '/urdf/' + robot_name + '.gazebo.xacro'
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(robot_name))
        f.write('\n')
        f.write('<xacro:arg name="controller_config" default="../config/controller.yaml" />\n')
        f.write('<xacro:property name="body_color" value="Gazebo/Silver" />\n')
        f.write('\n')

        if _movable_joint_names(joints_dict):
            gazebo = Element('gazebo')
            plugin = SubElement(gazebo, 'plugin')
            plugin.attrib = {
                'name': 'gz_ros2_control::GazeboSimROS2ControlPlugin',
                'filename': 'libgz_ros2_control-system'
            }
            parameters = SubElement(plugin, 'parameters')
            parameters.text = '$(arg controller_config)'
            gazebo_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
            f.write(gazebo_xml)
            f.write('\n')

        # for base_link
        f.write('<gazebo reference="base_link">\n')
        f.write('  <material>${body_color}</material>\n')
        f.write('  <mu1>0.2</mu1>\n')
        f.write('  <mu2>0.2</mu2>\n')
        f.write('  <selfCollide>true</selfCollide>\n')
        f.write('  <gravity>true</gravity>\n')
        f.write('</gazebo>\n')
        f.write('\n')

        # others
        for joint in joints_dict:
            name = joints_dict[joint]['child']
            f.write('<gazebo reference="{}">\n'.format(name))
            f.write('  <material>${body_color}</material>\n')
            f.write('  <mu1>0.2</mu1>\n')
            f.write('  <mu2>0.2</mu2>\n')
            f.write('  <selfCollide>true</selfCollide>\n')
            f.write('</gazebo>\n')
            f.write('\n')

        f.write('</robot>\n')


def write_display_launch(package_name, robot_name, save_dir):
    """
    write ROS 2 display launch file "save_dir/launch/display.launch.py"
    """
    _mkdir(save_dir + '/launch')
    _remove_if_exists(save_dir + '/launch/display.launch')

    file_name = save_dir + '/launch/display.launch.py'
    with open(file_name, mode='w') as f:
        f.write("""from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import FileContent, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('{package_name}')
    model = LaunchConfiguration('model')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    rvizconfig = LaunchConfiguration('rvizconfig')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description = FileContent(model)

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=PathJoinSubstitution([pkg_share, 'urdf', '{robot_name}.urdf'])
        ),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=PathJoinSubstitution([pkg_share, 'launch', 'urdf.rviz'])
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{{'use_sim_time': use_sim_time, 'robot_description': robot_description}}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(gui),
            parameters=[{{'robot_description': robot_description}}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(gui),
            parameters=[{{'robot_description': robot_description}}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rvizconfig],
            condition=IfCondition(rviz),
        ),
    ])
""".format(package_name=package_name, robot_name=robot_name))


def write_gazebo_launch(package_name, robot_name, save_dir, joints_dict=None):
    """
    write ROS 2 Gazebo launch file "save_dir/launch/gazebo.launch.py"
    """

    _mkdir(save_dir + '/launch')
    _remove_if_exists(save_dir + '/launch/gazebo.launch')
    has_movable_joints = bool(_movable_joint_names(joints_dict or {}))

    controller_spawners = ""
    if has_movable_joints:
        controller_spawners = """        Node(
            package='controller_manager',
            executable='spawner',
            name='spawner_joint_state_broadcaster',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name='spawner_{robot_name}_position_controller',
            arguments=['{robot_name}_position_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        ),""".format(robot_name=robot_name)

    file_name = save_dir + '/launch/gazebo.launch.py'
    with open(file_name, mode='w') as f:
        f.write("""from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('{package_name}')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', '{robot_name}.xacro'])
    controller_config = PathJoinSubstitution([pkg_share, 'config', 'controller.yaml'])
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' controller_config:=', controller_config]),
        value_type=str
    )
    gz_args = LaunchConfiguration('gz_args')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('gz_args', default_value='-r empty.sdf'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
            ),
            launch_arguments={{'gz_args': gz_args}}.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{{'use_sim_time': use_sim_time, 'robot_description': robot_description}}],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_{robot_name}',
            output='screen',
            arguments=['-name', '{robot_name}', '-topic', 'robot_description', '-allow_renaming', 'true'],
        ),
{controller_spawners}
    ])
""".format(package_name=package_name, robot_name=robot_name, controller_spawners=controller_spawners))


def write_control_launch(package_name, robot_name, save_dir, joints_dict):
    """
    write ROS 2 control launch file "save_dir/launch/controller.launch.py"
    """

    _mkdir(save_dir + '/launch')
    _remove_if_exists(save_dir + '/launch/controller.launch')
    has_movable_joints = bool(_movable_joint_names(joints_dict))

    controller_spawners = ""
    if has_movable_joints:
        controller_spawners = """        Node(
            package='controller_manager',
            executable='spawner',
            name='spawner_joint_state_broadcaster',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name='spawner_{robot_name}_position_controller',
            arguments=['{robot_name}_position_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        ),""".format(robot_name=robot_name)

    file_name = save_dir + '/launch/controller.launch.py'
    with open(file_name, mode='w') as f:
        f.write("""from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
{controller_spawners}
    ])
""".format(controller_spawners=controller_spawners))


def write_yaml(package_name, robot_name, save_dir, joints_dict):
    """
    write ROS 2 controller yaml file "save_dir/config/controller.yaml"
    """
    _mkdir(save_dir + '/config')
    _remove_if_exists(save_dir + '/launch/controller.yaml')

    controller_name = robot_name + '_position_controller'
    movable_joints = _movable_joint_names(joints_dict)
    file_name = save_dir + '/config/controller.yaml'

    with open(file_name, 'w') as f:
        f.write('controller_manager:\n')
        f.write('  ros__parameters:\n')
        f.write('    update_rate: 100\n')
        f.write('\n')
        f.write('    joint_state_broadcaster:\n')
        f.write('      type: joint_state_broadcaster/JointStateBroadcaster\n')

        if movable_joints:
            f.write('\n')
            f.write('    ' + controller_name + ':\n')
            f.write('      type: position_controllers/JointGroupPositionController\n')
            f.write('\n')
            f.write(controller_name + ':\n')
            f.write('  ros__parameters:\n')
            f.write('    joints:\n')
            for joint in movable_joints:
                f.write('      - ' + joint + '\n')
