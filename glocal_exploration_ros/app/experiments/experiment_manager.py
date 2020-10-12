#!/usr/bin/env python

# Python
import sys
import time
import csv
import datetime
import os
import re
import subprocess
import math
from xmlrpc.client import ServerProxy
import numpy as np
import psutil

# ROS
import rospy
import tf
import rosnode
import rosgraph

# msgs
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger
from voxblox_msgs.srv import FilePath


class ResourceMonitor(object):
    def __init__(self, ros_node_name, verbose=False):
        self.verbose = verbose

        self.cpu_frequency = 0
        self.total_wall_time = 0
        self.total_cpu_time = 0
        self.total_cpu_percent = 0
        self.total_memory_percent = 0

        self.node_name = ros_node_name
        self.node_process = None
        self.node_wall_time = 0
        self.node_cpu_time = 0
        self.node_cpu_percent = 0
        self.node_memory_percent = 0

    def get_process_for_ros_node(self, node_name):
        target_node = None
        caller_id = '/experiment_manager'
        master = rosgraph.Master(caller_id)
        running_nodes = rosnode.get_node_names()
        for node in running_nodes:
            if node_name in node:
                if target_node is None:
                    target_node = node
                else:
                    rospy.logwarn("Multiple nodes found whose name "
                                  "contains " + node_name)
                    return None
        if target_node is None:
            rospy.logwarn("Could not find a node whose name contains " +
                          node_name)
            return None
        target_node_api = rosnode.get_api_uri(master, target_node)
        if not target_node_api:
            rospy.logwarn("Could not connect to " + target_node + "'s API")
            return None

        target_node = ServerProxy(target_node_api)
        target_node_pid = rosnode._succeed(
            target_node.getPid(caller_id))  # NOLINT
        rospy.loginfo('Registered %s node PID %i for resource '
                      'usage tracking' % (node_name, target_node_pid))
        return psutil.Process(target_node_pid)

    def update_stats(self):
        if self.node_process is None:
            self.node_process = self.get_process_for_ros_node(self.node_name)
            if self.node_process is None:
                rospy.logwarn(
                    'Could not get PID for node named %s for resource '
                    'usage tracking.' % self.node_name)
            else:
                rospy.loginfo('Resource usage tracking set up for %s, '
                              'under PID %i' %
                              (self.node_name, self.node_process.pid))

        # NOTE: By default psutil evaluates its percentages over the interval
        #       since they were last queried
        self.cpu_frequency = psutil.cpu_freq().current
        self.total_wall_time = psutil._timer() - psutil.boot_time()
        self.total_cpu_time = psutil.cpu_times().system + psutil.cpu_times(
        ).user
        self.total_cpu_percent = psutil.cpu_percent()
        self.total_memory_percent = psutil.virtual_memory().percent

        try:
            self.node_wall_time = psutil._timer(
            ) - self.node_process.create_time()
            self.node_cpu_time = \
                self.node_process.cpu_times().system + \
                self.node_process.cpu_times().children_system + \
                self.node_process.cpu_times().user + \
                self.node_process.cpu_times().children_user
            self.node_cpu_percent = self.node_process.cpu_percent()
            self.node_memory_percent = self.node_process.memory_percent()

            if self.verbose:
                rospy.loginfo(
                    'Resource usage of node %s with PID: %i\n'
                    '--CPU frequency: %iMHz\n'
                    '--Total wall time: %is\n'
                    '--Total CPU time: %is\n'
                    '--Total CPU percent: %0.2f%%\n'
                    '--Total memory percent: %0.2f%%\n'
                    '--Node wall time: %is\n'
                    '--Node CPU time: %is\n'
                    '--Node CPU percent: %0.2f%%\n'
                    '--Node memory percent: %0.2f%%\n' %
                    (self.node_name, self.node_process.pid, self.cpu_frequency,
                     self.total_wall_time, self.total_cpu_time,
                     self.total_cpu_percent, self.total_memory_percent,
                     self.node_wall_time, self.node_cpu_time,
                     self.node_cpu_percent, self.node_memory_percent))

        except psutil.NoSuchProcess:
            rospy.logwarn("Process for node %s no longer exists" %
                          self.node_name)
            self.cpu_frequency = 0
            self.total_wall_time = 0
            self.total_cpu_time = 0
            self.total_cpu_percent = 0
            self.total_memory_percent = 0
            self.node_wall_time = 0
            self.node_cpu_time = 0
            self.node_cpu_percent = 0
            self.node_memory_percent = 0


class EvalData(object):
    def __init__(self):
        '''  Initialize ros node and read params '''
        # Parse parameters
        self.ns_planner = rospy.get_param(
            '~ns_planner', "/glocal/glocal_system/toggle_running")
        self.planner_delay = rospy.get_param(
            '~delay', 0.0)  # Waiting time until the planner is launched
        self.startup_timeout = rospy.get_param(
            '~startup_timeout', 0.0)  # Max allowed time for startup, 0 for inf

        self.evaluate = rospy.get_param(
            '~evaluate', False)  # Periodically save the voxblox state
        self.eval_frequency = rospy.get_param('~eval_frequency',
                                              30.0)  # Save rate in seconds
        self.time_limit = rospy.get_param(
            '~time_limit', 0.0)  # Maximum sim duration in minutes, 0 for inf

        # Name of the node whose resource usage we measure
        self.planner_node_name = rospy.get_param('~planner_node_name',
                                                 '/glocal_system')
        self.glocal_planning_cpu_time_topic = rospy.get_param(
            '~total_planning_cpu_time', '/glocal/total_planning_cpu_time')
        self.run_planner_srv_type = rospy.get_param('~planner_start_srv_type',
                                                    'SetBool')

        self.eval_walltime_0 = None
        self.eval_rostime_0 = None
        self.shutdown_reason_known = False

        if self.evaluate:
            # Setup parameters
            self.eval_directory = rospy.get_param(
                '~eval_directory',
                'DirParamNotSet')  # Periodically save voxblox map
            if not os.path.isdir(self.eval_directory):
                rospy.logfatal("Invalid target directory '%s'.",
                               self.eval_directory)
                sys.exit(-1)

            self.ns_voxblox = rospy.get_param('~ns_voxblox',
                                              "/voxblox/voxblox_node")

            # Statistics
            self.eval_n_maps = 0
            self.distance_traveled = 0
            self.previous_position = None
            self.collided = False
            self.run_planner_srv = None

            # placeholders
            self.eval_timer = None
            self.dist_timer = None
            self.initial_point_offset = None
            self.planner_resource_monitor = ResourceMonitor(
                self.planner_node_name, verbose=False)
            self.glocal_planning_cpu_time_s = 0.0

            # Setup data directory
            if not os.path.isdir(os.path.join(self.eval_directory,
                                              "tmp_bags")):
                os.mkdir(os.path.join(self.eval_directory, "tmp_bags"))
            self.eval_directory = os.path.join(
                self.eval_directory,
                datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
            os.mkdir(self.eval_directory)
            os.mkdir(os.path.join(self.eval_directory, "voxblox_maps"))
            self.eval_data_file = open(
                os.path.join(self.eval_directory, "voxblox_data.csv"), 'wb')
            self.eval_writer = csv.writer(self.eval_data_file,
                                          delimiter=',',
                                          quotechar='|',
                                          quoting=csv.QUOTE_MINIMAL,
                                          lineterminator='\n')
            self.eval_writer.writerow([
                'MapName', 'RosTime', 'WallTime', 'PositionDrift',
                'RotationDrift', 'PositionDriftEstimated',
                'RotationDriftEstimated', 'DistanceTraveled', 'CpuFrequency',
                'TotalWallTime', 'TotalCpuTime', 'TotalCpuPercent',
                'TotalMemoryPercent', 'PlannerWallTime', 'PlannerCpuTime',
                'PlannerCpuPercent', 'PlannerMemoryPercent',
                'GlocalPlanningCpuTime'
            ])
            self.eval_writer.writerow([
                'Unit', 's', 's', 'm', 'deg', 'm', 'deg', 'm', 'MHz', 's', 's',
                'Percent', 'Percent', 's', 's', 'Percent', 'Percent', 's'
            ])
            self.eval_log_file = open(
                os.path.join(self.eval_directory, "data_log.txt"), 'a')

            # Subscribers, Services
            self.collision_sub = rospy.Subscriber("collision",
                                                  Bool,
                                                  self.collision_callback,
                                                  queue_size=10)
            self.glocal_planning_cpu_time_sub = rospy.Subscriber(
                self.glocal_planning_cpu_time_topic,
                Float32,
                self.glocal_planning_cpu_time_callback,
                queue_size=1)
            self.tf_listener = tf.TransformListener()

            # Finish
            self.writelog("Data folder created at '%s'." % self.eval_directory)
            rospy.loginfo("[ExperimentManager]: Data folder created at '%s'." %
                          self.eval_directory)
            self.eval_voxblox_service = rospy.ServiceProxy(
                self.ns_voxblox + "/save_map", FilePath)
            rospy.on_shutdown(self.eval_finish)

        self.launch_simulation()

    def launch_simulation(self):
        rospy.loginfo(
            "[ExperimentManager]: Waiting for unreal MAV simulation to setup..."
        )
        # Wait for unreal simulation to setup
        if self.startup_timeout > 0.0:
            try:
                rospy.wait_for_message("/simulation_is_ready", Bool,
                                       self.startup_timeout)
            except rospy.ROSException:
                self.stop_experiment(
                    "Simulation startup failed (timeout after " +
                    str(self.startup_timeout) + "s).")
                return
        else:
            rospy.wait_for_message("/simulation_is_ready", Bool)
        rospy.loginfo(
            "[ExperimentManager]: Waiting for unreal MAV simulation to setup..."
            " done.")

        # Launch planner
        # (every planner needs to advertise this service when ready)
        rospy.loginfo(
            "[ExperimentManager]: Waiting for planner to be ready...")
        if self.startup_timeout > 0.0:
            try:
                rospy.wait_for_service(self.ns_planner, self.startup_timeout)
            except rospy.ROSException:
                self.stop_experiment("Planner startup failed (timeout after " +
                                     str(self.startup_timeout) + "s).")
                return
        else:
            rospy.wait_for_service(self.ns_planner)

        if self.planner_delay > 0:
            rospy.loginfo(
                "[ExperimentManager]: Waiting for planner to be ready... done. "
                "Launch in %d seconds.", self.planner_delay)
            rospy.sleep(self.planner_delay)
        else:
            rospy.loginfo(
                "[ExperimentManager]: Waiting for planner to be ready... done."
            )
        if 'SetBool' in self.run_planner_srv_type:
            self.run_planner_srv = rospy.ServiceProxy(self.ns_planner, SetBool)
            self.run_planner_srv(True)
        elif 'Trigger' in self.run_planner_srv_type:
            self.run_planner_srv = rospy.ServiceProxy(self.ns_planner, Trigger)
            self.run_planner_srv()
        else:
            rospy.logfatal(
                "Planner service type should be SetBool or Trigger, "
                "but is: %s" % self.run_planner_srv_type)

        # Setup first measurements
        self.eval_walltime_0 = time.time()
        self.eval_rostime_0 = rospy.get_time()
        # Evaluation init
        if self.evaluate:
            self.writelog("Succesfully started the simulation.")

            # Dump complete rosparams for reference
            subprocess.check_call([
                "rosparam", "dump",
                os.path.join(self.eval_directory, "rosparams.yaml"), "/"
            ])
            self.writelog("Dumped the parameter server into 'rosparams.yaml'.")

            self.eval_n_maps = 0

            # Keep track of the (most recent) rosbag
            bag_expr = re.compile(
                r'tmp_bag_\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}\.bag.'
            )  # Default names
            bags = [
                b for b in os.listdir(
                    os.path.join(os.path.dirname(self.eval_directory),
                                 "tmp_bags")) if bag_expr.match(b)
            ]
            bags.sort(reverse=True)
            if bags:
                self.writelog("Registered '%s' as bag for this experiment." %
                              bags[0])
                self.eval_log_file.write("[FLAG] Rosbag: %s\n" %
                                         bags[0].split('.')[0])
            else:
                rospy.logwarn(
                    "[ExperimentManager]: No tmpbag found. Is rosbag recording?"
                )
                self.writelog("No rosbag found to register.")

            # Periodic evaluation (call once for initial measurement)
            self.eval_callback(None)
            self.eval_timer = rospy.Timer(rospy.Duration(self.eval_frequency),
                                          self.eval_callback)
            self.dist_timer = rospy.Timer(rospy.Duration(0.1),
                                          self.distance_callback)

        # Finish
        rospy.loginfo("\n" + "*" * 40 +
                      "\n* Successfully started the experiment! *\n" +
                      "*" * 40)

    def eval_callback(self, _):
        # Check whether the planner is still alive
        try:
            # If planner is running calling this service again does nothing
            if 'SetBool' in self.run_planner_srv_type:
                self.run_planner_srv(True)
            elif 'Trigger' in self.run_planner_srv_type:
                self.run_planner_srv()
        except:
            # Usually this means the planner died
            self.stop_experiment("Planner Node died.")
            return

        # Produce a data point
        time_real = time.time() - self.eval_walltime_0
        time_ros = rospy.get_time() - self.eval_rostime_0
        map_name = "{0:05d}".format(self.eval_n_maps)

        # Compute transform errors
        drift_pos = None
        drift_rot = 0
        try:
            (t, r) = self.tf_listener.lookupTransform(
                'airsim_drone/Lidar', 'airsim_drone/Lidar_ground_truth',
                rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            drift_pos = 0
        if drift_pos is None:
            r = np.array(r)
            r = r / np.linalg.norm(r)
            drift_pos = (t[0]**2 + t[1]**2 + t[2]**2)**0.5
            drift_rot = 2 * math.acos(r[3]) * 180.0 / math.pi

        drift_estimated_pos = None
        drift_estimated_rot = 0

        try:
            (t, r) = self.tf_listener.lookupTransform('odom', 'initial_pose',
                                                      rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            drift_estimated_pos = 0
        if drift_estimated_pos is None:
            if self.initial_point_offset is None:
                drift_estimated_pos = 0
            else:
                trans = np.dot(tf.transformations.translation_matrix(t),
                               tf.transformations.quaternion_matrix(r))

                trans = tf.transformations.concatenate_matrices(
                    trans, self.initial_point_offset)
                t = tf.transformations.translation_from_matrix(trans)
                r = tf.transformations.quaternion_from_matrix(trans)
                drift_estimated_pos = (t[0]**2 + t[1]**2 + t[2]**2)**0.5
                r = np.array(r)
                r = r / np.linalg.norm(r)
                drift_estimated_rot = 2 * math.acos(r[3]) * 180.0 / math.pi

        # Gather system resource usage stats
        self.planner_resource_monitor.update_stats()

        self.eval_writer.writerow([
            map_name, time_ros, time_real, drift_pos, drift_rot,
            drift_estimated_pos, drift_estimated_rot, self.distance_traveled,
            self.planner_resource_monitor.cpu_frequency,
            self.planner_resource_monitor.total_wall_time,
            self.planner_resource_monitor.total_cpu_time,
            self.planner_resource_monitor.total_cpu_percent,
            self.planner_resource_monitor.total_memory_percent,
            self.planner_resource_monitor.node_wall_time,
            self.planner_resource_monitor.node_cpu_time,
            self.planner_resource_monitor.node_cpu_percent,
            self.planner_resource_monitor.node_memory_percent,
            self.glocal_planning_cpu_time_s
        ])
        # Immediately write the data to disk to avoid losing anything if the
        # experiment gets interupted
        self.eval_data_file.flush()
        self.eval_voxblox_service(
            os.path.join(self.eval_directory, "voxblox_maps",
                         map_name + ".vxblx"))
        self.eval_n_maps += 1

        # If the time limit is reached stop the simulation
        if self.time_limit > 0.0:
            if rospy.get_time(
            ) - self.eval_rostime_0 >= self.time_limit * 60.0:
                self.stop_experiment("Time limit reached.")

    def distance_callback(self, _):
        """ Periodically query tf to see how much distance was covered """
        # Test whether the origin point is already published
        if self.initial_point_offset is None:
            try:
                (t1, r1) = self.tf_listener.lookupTransform(
                    'initial_pose', 'odom', rospy.Time(0))
                (t2, r2) = self.tf_listener.lookupTransform(
                    'airsim_drone/Lidar', 'airsim_drone/Lidar_ground_truth',
                    rospy.Time(0))
                trans1 = np.dot(tf.transformations.translation_matrix(t1),
                                tf.transformations.quaternion_matrix(r1))
                trans2 = np.dot(tf.transformations.translation_matrix(t2),
                                tf.transformations.quaternion_matrix(r2))
                self.initial_point_offset = \
                    tf.transformations.concatenate_matrices(trans1, trans2)

            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                pass

        # Query position
        t, _ = self.tf_listener.lookupTransform('odom',
                                                'airsim_drone_ground_truth',
                                                rospy.Time(0))
        t = np.array(t)
        if self.previous_position is None:
            self.previous_position = t
            return

        self.distance_traveled = self.distance_traveled + np.linalg.norm(
            self.previous_position - t)
        self.previous_position = t

    def writelog(self, text):
        # In case of simulation data being stored, maintain a log file
        if not self.evaluate:
            return
        self.eval_log_file.write(
            datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S] ") + text +
            "\n")

    def stop_experiment(self, reason):
        # Shutdown the node with proper logging, only required when experiment
        # is being performed
        reason = "Stopping the experiment: " + reason
        self.shutdown_reason_known = True
        if self.evaluate:
            self.writelog(reason)
        width = len(reason) + 4
        rospy.loginfo("\n" + "*" * width + "\n* " + reason + " *\n" +
                      "*" * width)
        rospy.signal_shutdown(reason)

    def eval_finish(self):
        self.eval_data_file.close()
        map_path = os.path.join(self.eval_directory, "voxblox_maps")
        n_maps = len([
            f for f in os.listdir(map_path)
            if os.path.isfile(os.path.join(map_path, f))
        ])
        if not self.shutdown_reason_known:
            self.writelog("Stopping the experiment: External Interrupt.")
        self.writelog("Finished the simulation, %d/%d maps created." %
                      (n_maps, self.eval_n_maps))
        self.eval_log_file.close()
        rospy.loginfo(
            "[ExperimentManager]: On eval_data_node shutdown: closing data "
            "files.")

    def collision_callback(self, _):
        if not self.collided:
            self.collided = True
            self.stop_experiment("Collision detected!")

    def glocal_planning_cpu_time_callback(self, msg):
        self.glocal_planning_cpu_time_s = msg.data


if __name__ == '__main__':
    rospy.init_node('experiment_manager', anonymous=False)
    ed = EvalData()
    rospy.spin()
