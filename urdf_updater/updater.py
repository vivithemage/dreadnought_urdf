#
# Dynamic URDF Publisher
#
# This script monitors a URDF file and when changed will publish to /robot_description
#
# Requirements:
#   Besides the standard Ros2 modules, Install watchdog module for listening 
#   to filesystem events:
#      `pip3 install watchdog`
#
# Features:
#    * can process xacro files
#    * can specify URDF file by package:file or absolute filename.
#        (If find by package is used, --symlink-install is recommended otherwise you'd
#        have to modify the share filename to detect a change.)
#
# usage: dynamic-urdf-publisher.py [-h] --urdf <urdf-file> [--package <package-name>] [--xacro]
# 
# Monitor a URDF file and publish on any changes
# 
# optional arguments:
#   -h, --help            show this help message and exit
#   --urdf <urdf-file>    The URDF file to monitor and publish
#   --package <package-name>
#                         If specified, load the URDF relative to given package installation
#   --xacro               use xacro to transform the URDF file before publishing
#
#

import argparse
import os
import sys
from watchdog.observers import Observer
from watchdog.events import FileSystemEvent, FileSystemEventHandler

# Ros2 node imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

import xacro


class URDFChangeEventHander(FileSystemEventHandler):
    node: object

    def __init__(self, node):
        self.node = node

    def publish(self, filepath):
        basepath, filename = os.path.split(filepath)
        if os.path.isdir(filepath):
            # dont update just for a dir update
            return
        elif filename.startswith('.'):
            # ignore dot files (like .swp files)
            return
        self.node.publish()

    def on_any_event(self, event):
        pass

    def on_modified(self, event: FileSystemEvent):
        self.publish(event.src_path)


    def on_deleted(self, event):
        pass

    def on_moved(self, event):
        pass

    def on_created(self, event):
        self.publish(event.src_path)

    def on_closed(self, event):
        pass


class URDFPublisher(Node):
    urdf_filename = None
    package_name = None
    use_xacro = False

    full_urdf_filename = None

    # called when URDF should be published again
    # we start with True so we always publish on startup
    updated = True

    def __init__(self, args):
        super().__init__('dynamic_urdf_publisher')

        parser = argparse.ArgumentParser(
            description='Monitor a URDF file and publish on any changes')
        parser.add_argument('--urdf', required=True, type=str, dest='urdf_filename', metavar='<urdf-file>',
                            help='The URDF file to monitor and publish')
        parser.add_argument('--package', required=False, type=str, dest='package_name', metavar='<package-name>',
                            help='If specified, load the URDF relative to given package installation')
        parser.add_argument('--xacro', required=False, action='store_true', dest='use_xacro',
                            help='use xacro to transform the URDF file before publishing')
        args = parser.parse_args(args[1:])
        self.urdf_filename = args.urdf_filename
        self.package_name = args.package_name
        self.use_xacro = args.use_xacro

        reliable_transient_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)

        # create the TF publisher for base
        self.robot_description_pub = self.create_publisher(
            String,
            "/robot_description",
            reliable_transient_profile)

        # we set a 'publish' flag on file changes and then monitor
        # this flag every second before sending a publish msg.
        # This solves a few issues:
        #    (1) Some editors delete and recreate the file, so file
        #    may not exist if we publish immediately
        #    (2) If we get a flood of events we dont publish a flood,
        #    but instead only once a second maximum.
        self.timer = self.create_timer(1.0, self._transmit_robot_description)

    def publish(self):
        self.updated = True

    def find_urdf(self):
        full_urdf_file = self.urdf_filename
        if self.package_name:
            package_path = get_package_share_directory(self.package_name)
            full_urdf_file = os.path.join(package_path, self.urdf_filename)
            if not os.path.exists(full_urdf_file):
                full_urdf_file = os.path.join(package_path, 'urdf', self.urdf_filename)
                if not os.path.exists(full_urdf_file):
                    raise FileNotFoundError(f'cannot find URDF in {package_path}')
        else:
            if not os.path.exists(full_urdf_file):
                raise FileNotFoundError(f'URDF file {full_urdf_file} does not exist')

        if os.path.islink(full_urdf_file):
            full_urdf_file = os.path.realpath(full_urdf_file)
        return full_urdf_file

    def _transmit_robot_description(self):
        if not self.updated:
            return # nothing to do
        if not os.path.exists(self.full_urdf_filename):
            print("fine not found")
            return

        if self.use_xacro:
            try:
                robot_description_config = xacro.process_file(self.full_urdf_filename)
                robot_description = robot_description_config.toprettyxml()
            except Exception as e:
                print('Refusing to publish due to exception: ', e)
                return
        else:
            with open(self.full_urdf_filename) as f:
                robot_description = f.read()

        # publish the string
        urdf_str = String()
        urdf_str.data = robot_description
        self.robot_description_pub.publish(urdf_str)
        self.updated = False
        print("published URDF using xacro" if self.use_xacro else "published URDF")

    def run(self):
        # determine the path to the URDF
        self.full_urdf_filename = self.find_urdf()

        # break filname into directory and file
        basepath, filename = os.path.split(self.full_urdf_filename)

        # monitor the whole directory for changes
        observer = Observer()
        handler = URDFChangeEventHander(self)
        observer.schedule(handler, basepath, recursive=True)
        observer.start()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1)

        observer.stop()
        observer.join()

def main(args=sys.argv):
    rclpy.init(args=args)
    args_without_ros = rclpy.utilities.remove_ros_args(args)
    urdf_pub_node = URDFPublisher(args_without_ros)
    urdf_pub_node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
