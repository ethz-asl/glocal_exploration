#!/usr/bin/env python

# Python
import csv
import datetime
import os
import re
import shutil
import sys

# ros
import rospy
from std_srvs.srv import Empty

import numpy as np
from matplotlib import pyplot as plt


class EvaluationManager(object):
    """
    This is the main evaluation node to read the data created by the experiment
    manager. The voxblox_evaluation_node is used to parse voxblox maps.
    """
    def __init__(self):
        # Parse parameters
        target_dir = rospy.get_param('~target_directory')
        self.method = rospy.get_param('~method', 'single')
        self.ns_voxblox = rospy.get_param('~ns_eval_voxblox_node',
                                          '/eval_voxblox_node')
        self.evaluate = rospy.get_param('~evaluate', True)
        self.create_plots = rospy.get_param('~create_plots', True)
        self.series = rospy.get_param(
            '~series', False)  # True: skip single evaluation and create
        # series evaluation data and plots for all runs in the target directory
        self.clear_voxblox_maps = rospy.get_param(
            '~clear_voxblox_maps',
            False)  # rm all maps after eval (disk space!)

        # Check for valid params
        methods = {
            'single': 'single',
            'recent': 'recent',
            'all': 'all'
        }  # Dictionary of implemented models
        selected = methods.get(self.method, 'NotFound')
        if selected == 'NotFound':
            warning = "Unknown method '" + self.method + \
                      "'. Implemented are: " + \
                      "".join(["'" + m + "', " for m in methods])
            rospy.logfatal(warning[:-2])
            sys.exit(-1)
        else:
            self.method = selected

        # Setup
        self.eval_log_file = None
        rospy.wait_for_service(self.ns_voxblox + "/evaluate")
        self.eval_voxblox_srv = rospy.ServiceProxy(
            self.ns_voxblox + "/evaluate", Empty)

        # Evaluate
        if self.series:
            self.evaluate_series(target_dir)
        elif self.method == 'single':
            self.run_single_evaluation(target_dir)
        elif self.method == 'recent':
            dir_expression = re.compile(
                r'\d{8}_\d{6}')  # Only check the default names
            subdirs = [
                o for o in os.listdir(target_dir)
                if os.path.isdir(os.path.join(target_dir, o))
                and dir_expression.match(o)
            ]
            subdirs.sort(reverse=True)
            if not subdirs:
                rospy.loginfo(
                    "No recent directories in target dir '%s' to evaluate.",
                    target_dir)
                sys.exit(-1)

            self.run_single_evaluation(os.path.join(target_dir, subdirs[0]))
        elif self.method == 'all':
            subdirs = [
                o for o in os.listdir(target_dir)
                if os.path.isdir(os.path.join(target_dir, o))
            ]
            for subdir in subdirs:
                self.run_single_evaluation(os.path.join(target_dir, subdir))

        rospy.loginfo(
            "\n" + "*" * 53 +
            "\n* Evaluation completed successfully, shutting down. *\n" +
            "*" * 53)

    def run_single_evaluation(self, target_dir):
        rospy.loginfo("Starting evaluation on target '%s'.", target_dir)
        # Check target dir is valid (approximately)
        if not os.path.isfile(os.path.join(target_dir, "data_log.txt")):
            rospy.logerr(
                "Invalid target directory: Could not find a 'data_log.txt' "
                "file.")
            return

        # Check for rosbag renaming.
        self.eval_log_file = open(os.path.join(target_dir, "data_log.txt"),
                                  'a+')
        lines = [line.rstrip('\n') for line in self.eval_log_file]
        if not "[FLAG] Rosbag renamed" in lines:
            for line in lines:
                if line[:14] == "[FLAG] Rosbag:":
                    file_name = os.path.join(os.path.dirname(target_dir),
                                             "tmp_bags", line[15:] + ".bag")
                    if os.path.isfile(file_name):
                        os.rename(
                            file_name,
                            os.path.join(target_dir, "visualization.bag"))
                        self.writelog(
                            "Moved the tmp rosbag into 'visualization.bag'")
                        self.eval_log_file.write("[FLAG] Rosbag renamed\n")
                    else:
                        self.writelog("Error: unable to locate '" + file_name +
                                      "'.")
                        rospy.logwarn("Error: unable to locate '" + file_name +
                                      "'.")

        self.eval_log_file.close()  # Make it available for voxblox node.

        # Call voxblox evaluation.
        if self.evaluate:
            # Set params and call the voxblox evaluator.
            rospy.set_param(self.ns_voxblox + "/target_directory", target_dir)
            try:
                self.eval_voxblox_srv()
            except:
                rospy.logerr(
                    "eval_voxblox service call failed. Shutting down.")
                sys.exit(-1)

        # Reopen logfile.
        self.eval_log_file = open(os.path.join(target_dir, "data_log.txt"),
                                  'a+')

        if self.create_plots:
            # Create dirs.
            if not os.path.isdir(os.path.join(target_dir, "graphs")):
                os.mkdir(os.path.join(target_dir, "graphs"))

            if os.path.isfile(
                    os.path.join(target_dir, "voxblox_data_evaluated.csv")):
                # Read voxblox data file.
                data_voxblox = self.read_voxblox_data(
                    os.path.join(target_dir, "voxblox_data_evaluated.csv"))
                if len(data_voxblox['RosTime']) > 1:
                    self.plot_sim_overview(data_voxblox, target_dir)
                else:
                    rospy.loginfo(
                        "Too few entries in 'voxblox_data_evaluated.csv', "
                        "skipping dependent graphs.")
            else:
                rospy.loginfo(
                    "No 'voxblox_data_evaluated.csv' found, skipping "
                    "dependent graphs.")

            # Finish
            if self.clear_voxblox_maps:
                # Remove all voxblox maps to free up disk space.
                shutil.rmtree(os.path.join(target_dir, 'voxblox_maps'),
                              ignore_errors=True)
            self.eval_log_file.close()

    def evaluate_series(self, target_dir):
        rospy.loginfo("Evaluating experiment series at '%s'", target_dir)

        # Setup a directory for data, plots, ...
        folder_name = "series_evaluation"
        if not os.path.isdir(os.path.join(target_dir, folder_name)):
            os.mkdir(os.path.join(target_dir, folder_name))
        self.eval_log_file = open(
            os.path.join(target_dir, folder_name, "eval_log.txt"), 'a')

        # Read all the data.
        dir_expression = re.compile(r'\d{8}_\d{6}')
        subdirs = [
            o for o in os.listdir(target_dir)
            if os.path.isdir(os.path.join(target_dir, o))
            and dir_expression.match(o)
        ]
        self.writelog("Evaluating '%s' (%i subdirs)." %
                      (target_dir, len(subdirs)))
        voxblox_data = []
        max_data_length = 0
        names = []
        for o in subdirs:
            if os.path.isfile((os.path.join(target_dir, o,
                                            "voxblox_data_evaluated.csv"))):
                # Valid evaluated directory.
                data = self.read_voxblox_data(
                    os.path.join(target_dir, o, "voxblox_data_evaluated.csv"))
                max_data_length = max(max_data_length, len(data["RosTime"]))
                voxblox_data.append(data)
                names.append(o)
            else:
                rospy.logwarn("Experiment at '%s' not properly evaluated!", o)
                self.writelog("Experiment at '%s' not properly evaluated!" % o)

        if max_data_length < 2:
            rospy.loginfo(
                "No valid experiments found, stopping series evaluation.")
            self.writelog(
                "No valid experiments found, stopping series evaluation.")
            self.eval_log_file.close()
            return

        # Create common data timeline by averaging measurement times (these
        # should be similar).
        data_file = open(
            os.path.join(target_dir, folder_name, "series_data.csv"), 'wb')
        data_writer = csv.writer(data_file,
                                 delimiter=',',
                                 quotechar='|',
                                 quoting=csv.QUOTE_MINIMAL,
                                 lineterminator='\n')
        means = {}
        std_devs = {}
        keys = voxblox_data[0].keys()
        keys.remove('RosTime')
        keys.remove('MapName')
        keys = ['RosTime'] + keys  # RosTime is expected as the first argument.
        for key in keys:
            means[key] = np.array([])
            std_devs[key] = np.array([])
        for i in range(max_data_length):
            line = []
            if i == 0:
                header_line = []
                for key in keys:
                    header_line.extend((key, ''))
                    for name in names:
                        line.append(name)
                        header_line.append('')
                    line.extend(("Mean", "StdDev"))
                data_writer.writerow(header_line)
                data_writer.writerow(line)
                line = []
            for key in keys:
                values = []
                for dataset in voxblox_data:
                    if i < len(dataset[key]):
                        line.append(dataset[key][i])
                        values.append(dataset[key][i])
                    else:
                        line.append("")
                values = np.array(values, dtype=float)
                mean = np.mean(values)
                std = np.std(values)
                means[key] = np.append(means[key], mean)
                std_devs[key] = np.append(std_devs[key], std)
                line.extend((mean, std))
            data_writer.writerow(line)
        data_file.close()

        # Create plot.
        rospy.loginfo("Creating graph 'SeriesOverview'")
        x = means['RosTime']
        unit = "s"
        if x[-1] >= 300:
            unit = "min"
            x = x / 60

        # Plot ends of data series for unequal lengths
        early_stops = []
        for i in range(len(voxblox_data)):
            dataset = voxblox_data[i]
            length = len(dataset['RosTime']) - 1
            if length < max_data_length - 1:
                time = 0
                if length >= 0:
                    time = float(dataset['RosTime'][length])
                early_stops.append(length)
                self.writelog("Early stop detected for '%s' at %.2fs." %
                              (names[i], time))

        # create all plots.
        fig, axes = plt.subplots(2, 2)
        ax = axes[0, 0]
        y = np.array(means['ObservedVolume'], dtype=float)
        y = y / (40 * 40 * 3) * 100  # Compensate for total volume
        y_std = np.array(std_devs['ObservedVolume'], dtype=float)
        y_std = y_std / (40 * 40 * 3) * 100  # Compensate for total volume
        ax.plot(x, y, 'b-')
        ax.fill_between(x, y - y_std, y + y_std, facecolor='b', alpha=.2)
        ax.plot([x[i] for i in early_stops], [y[i] for i in early_stops],
                'kx',
                markersize=9,
                markeredgewidth=2)
        ax.set_ylabel('Observed Volume [%]')
        ax.set_ylim(bottom=0, top=100)
        ax.set_xlim(left=0, right=x[-1])
        ax.set_xlabel('Simulated Time [%s]' % unit)

        ax = axes[1, 0]
        y = np.array(means['DistanceTraveled'], dtype=float)
        y_std = np.array(std_devs['DistanceTraveled'], dtype=float)
        ax.plot(x, y, 'g-')
        ax.fill_between(x, y - y_std, y + y_std, facecolor='g', alpha=.2)
        ax.set_ylabel('Distance Traveled [m]')
        ax.set_ylim(bottom=0)
        ax.set_xlim(left=0, right=x[-1])
        ax.set_xlabel("Simulated Time [%s]" % unit)

        ax = axes[0, 1]
        y = np.array(means['PositionDrift'], dtype=float)
        y_std = np.array(std_devs['PositionDrift'], dtype=float)
        ax.plot(x, y, 'k-')
        ax.fill_between(x, y - y_std, y + y_std, facecolor='k', alpha=.2)
        y = np.array(means['PositionDriftEstimated'], dtype=float)
        y_std = np.array(std_devs['PositionDriftEstimated'], dtype=float)
        ax.plot(x, y, 'r-')
        ax.fill_between(x, y - y_std, y + y_std, facecolor='r', alpha=.2)
        ax.set_ylabel('Position Drift [m]')
        ax.set_ylim(bottom=0)
        ax.set_xlim(left=0, right=x[-1])
        ax.set_xlabel("Simulated Time [%s]" % unit)
        ax.legend(['Simulated', 'Estimated'])

        ax = axes[1, 1]
        y = np.array(means['RotationDrift'], dtype=float)
        y_std = np.array(std_devs['RotationDrift'], dtype=float)
        ax.plot(x, y, 'k-')
        ax.fill_between(x, y - y_std, y + y_std, facecolor='k', alpha=.2)
        y = np.array(means['RotationDriftEstimated'], dtype=float)
        y_std = np.array(std_devs['RotationDriftEstimated'], dtype=float)
        ax.plot(x, y, 'r-')
        ax.fill_between(x, y - y_std, y + y_std, facecolor='r', alpha=.2)
        ax.set_ylabel('Rotation Drift [deg]')
        ax.set_ylim(bottom=0)
        ax.set_xlim(left=0, right=x[-1])
        ax.set_xlabel("Simulated Time [%s]" % unit)
        ax.legend(['Simulated', 'Estimated'])

        # Finish.
        plt.suptitle("Experiment Series Overview (" + str(len(voxblox_data)) +
                     " experiments)\nMeans + Std. Deviations (shaded)")
        fig.set_size_inches(15, 10, forward=True)

        save_name = os.path.join(target_dir, folder_name, "SeriesOverview.png")
        plt.savefig(save_name, dpi=300, format='png', bbox_inches='tight')
        self.writelog("Created graph 'SeriesOverview'.")
        self.eval_log_file.close()

    @staticmethod
    def read_voxblox_data(file_name):
        # Read voxblox data file
        data_voxblox = {}
        headers = None
        with open(file_name) as infile:
            reader = csv.reader(infile,
                                delimiter=',',
                                quotechar='|',
                                quoting=csv.QUOTE_MINIMAL)
            for row in reader:
                if row[0] == 'MapName':
                    headers = row
                    for header in headers:
                        data_voxblox[header] = []
                    continue
                if row[0] != 'Unit':
                    for i in range(len(row)):
                        data_voxblox[headers[i]].append(row[i])
        return data_voxblox

    def plot_sim_overview(self, data, target_dir):
        rospy.loginfo("Creating Graphs: SimulationOverview")
        unit = "s"

        x = np.array(data['RosTime'], dtype=float)
        if x[-1] >= 300:
            unit = "min"
            x = x / 60

        # Create all plots.
        fig, axes = plt.subplots(2, 2)

        ax = axes[0, 0]
        mean = np.array(data['ObservedVolume'], dtype=float)
        mean = mean / (40 * 40 * 3) * 100  # Compensate for total volume
        ax.plot(x, mean, 'b-')
        ax.set_ylabel('Observed Volume [%]')
        ax.set_ylim(bottom=0, top=100)
        ax.set_xlim(left=0, right=x[-1])
        ax.set_xlabel("Simulated Time [%s]" % unit)

        ax = axes[1, 0]
        y = np.array(data['DistanceTraveled'], dtype=float)
        ax.plot(x, y, 'g-')
        ax.set_ylabel('Distance Traveled [m]')
        ax.set_ylim(bottom=0)
        ax.set_xlim(left=0, right=x[-1])
        ax.set_xlabel("Simulated Time [%s]" % unit)

        ax = axes[0, 1]
        y = np.array(data['PositionDrift'], dtype=float)
        ax.plot(x, y, 'k-')
        y = np.array(data['PositionDriftEstimated'], dtype=float)
        ax.plot(x, y, 'r-')
        ax.set_ylabel('Position Drift [m]')
        ax.set_ylim(bottom=0)
        ax.set_xlim(left=0, right=x[-1])
        ax.set_xlabel("Simulated Time [%s]" % unit)
        ax.legend(['Simulated', 'Estimated'])

        ax = axes[1, 1]
        y = np.array(data['RotationDrift'], dtype=float)
        ax.plot(x, y, 'k-')
        y = np.array(data['RotationDriftEstimated'], dtype=float)
        ax.plot(x, y, 'r-')
        ax.set_ylabel('Rotation Drift [deg]')
        ax.set_ylim(bottom=0)
        ax.set_xlim(left=0, right=x[-1])
        ax.set_xlabel("Simulated Time [%s]" % unit)
        ax.legend(['Simulated', 'Estimated'])

        # Finish.
        save_name = os.path.join(target_dir, "graphs",
                                 "SimulationOverview.png")
        fig.set_size_inches(15, 10, forward=True)
        plt.savefig(save_name, dpi=300, format='png', bbox_inches='tight')
        self.writelog("Created graph 'SimulationOverview'.")

    def writelog(self, text):
        if self.eval_log_file is not None:
            self.eval_log_file.write(
                datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S] ") +
                text + "\n")


if __name__ == '__main__':
    rospy.init_node('eval_plotting_node', anonymous=True)
    em = EvaluationManager()
