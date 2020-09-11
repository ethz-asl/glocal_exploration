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
            np.divide(x, 60)

        # Plot ends of data series for unequal lengths
        early_stops = []
        x_early = []
        for i in range(len(voxblox_data)):
            dataset = voxblox_data[i]
            length = len(dataset['RosTime']) - 1
            if length < max_data_length - 1:
                early_stops.append(length)
                x_early.append(float(dataset['RosTime'][length]))
                self.writelog("Early stop detected for '%s' at %.2fs." %
                              (names[i], float(dataset['RosTime'][length])))

        # create all plots.
        fig, axes = plt.subplots(1, 1)
        axes[0, 0].plot(x, means['ObservedVolume'], 'b-')
        axes[0, 0].fill_between(
            x,
            means['ObservedVolume'] - std_devs['ObservedVolume'],
            means['ObservedVolume'] + std_devs['ObservedVolume'],
            facecolor='b',
            alpha=.2)
        axes[0, 0].plot([x[i] for i in early_stops],
                        [means['ObservedVolume'][i] for i in early_stops],
                        'kx',
                        markersize=9,
                        markeredgewidth=2)
        axes[0, 0].set_ylabel('Observed Volume [m]')
        axes[0, 0].set_ylim(bottom=0)
        axes[0, 0].set_xlim(left=0, right=x[-1])
        axes[0, 0].set_xlabel('Simulated Time [%s]' % unit)

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
            np.divide(x, 60)

        mean = np.array(data['ObservedVolume'])

        # Create all plots.
        _, axes = plt.subplots(1, 1)
        axes[0, 0].plot(x, mean, 'b-')
        axes[0, 0].set_ylabel('Observed Volume [m]')
        axes[0, 0].set_ylim(bottom=0)
        axes[0, 0].set_xlim(left=0, right=x[-1])
        axes[0, 0].set_xlabel("Simulated Time [%s]" % unit)

        # Finish.
        save_name = os.path.join(target_dir, "graphs",
                                 "SimulationOverview.png")
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
