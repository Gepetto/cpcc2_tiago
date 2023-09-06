#!/usr/bin/env python3

"""
This script is used to evaluate the performance of the cpcc2 controller.
It sends a sequence of targets to the controller and records the error and the 
time it takes to reach each target. To evalate if the task is done we compute the stantard deviation
based on the last 1000 error samples. If the standard deviation is below a certain threshold and the
current error is below 0.1, we consider the task done and move to the next target.
It also log the full history of the error, the target, the ricatti and crocoddyl command, the real torque and the time.
The command to run the performance_evaluator.py script is:
ros2 run cpcc2_tiago performance_evaluator /path/to/log/dir/ suffix
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import sys
import csv
import os


class PerformanceEvaluator(Node):
    def __init__(self, target_sequence):
        super().__init__("performance_evaluator")
        self.target_publisher = self.create_publisher(
            Float64MultiArray, "crocoddyl_controller/target", 10
        )
        self.error_subscriber = self.create_subscription(
            Float64MultiArray,
            "crocoddyl_controller/end_effect_pos_error",
            self.error_callback,
            10,
        )
        self.ric_effort_subscriber = self.create_subscription(
            Float64MultiArray,
            "pveg_chained_controller/ricatti_command",
            self.ric_effort_callback,
            10,
        )

        self.croc_effort_subscriber = self.create_subscription(
            Float64MultiArray,
            "crocoddyl_controller/effort_command",
            self.croc_effort_callback,
            10,
        )

        self.real_effort_subscriber = self.create_subscription(
            Float64MultiArray,
            "crocoddyl_controller/real_effort",
            self.real_effort_callback,
            10,
        )

        self.ddq_subscriber = self.create_subscription(
            Float64MultiArray, "pveg_chained_controller/ddq", self.ddq_callback, 10
        )

        self.error_subscriber  # To avoid the subscriber from being garbage collected
        self.target_sequence = target_sequence
        self.target_index = 0
        self.current_target = None
        self.previous_errors = np.array([1e3, 1e3, 1e3])
        self.started_evaluation = False
        self.max_distance_variation = 0.0005
        self.num_error_samples = 1000  # Number of error samples to consider for std
        self.error_history = []
        self.full_error_history = []
        self.full_ric_effort_history = []
        self.full_croc_effort_history = []
        self.full_real_effort_history = []
        self.full_ddq_history = []
        self.full_time_history = []
        self.full_target_history = []

        self.start_time = None
        self.start_evaluation_time = self.get_clock().now()
        self.path_to_log_file = sys.argv[1]
        self.suffix = sys.argv[2]

    def send_target(self, target):
        msg = Float64MultiArray(data=target)
        self.target_publisher.publish(msg)
        self.current_target = target
        self.previous_errors = np.array([1e3, 1e3, 1e3])
        self.started_evaluation = True
        self.start_time = self.get_clock().now()

    def euclidean_distance(self, error):
        if error.ndim == 1:
            return np.linalg.norm(error, ord=2)
        else:
            return np.linalg.norm(error, ord=2, axis=1)

    def ric_effort_callback(self, msg):
        self.full_ric_effort_history.append(np.array(msg.data))

    def croc_effort_callback(self, msg):
        self.full_croc_effort_history.append(np.array(msg.data))

    def real_effort_callback(self, msg):
        self.full_real_effort_history.append(np.array(msg.data))

    def ddq_callback(self, msg):
        self.full_ddq_history.append(np.array(msg.data))

    def error_callback(self, msg):
        if self.started_evaluation and self.current_target is not None:
            current_error = np.array(msg.data)
            elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            total_elapsed_time = (
                self.get_clock().now() - self.start_evaluation_time
            ).nanoseconds / 1e9

            self.previous_errors = np.vstack((self.previous_errors, current_error))
            current_distance = self.euclidean_distance(current_error)
            previous_distance = self.euclidean_distance(
                self.previous_errors[-self.num_error_samples : :]
            )

            self.full_error_history.append(current_error)
            self.full_time_history.append(total_elapsed_time)
            self.full_target_history.append(np.array(self.current_target))

            if len(self.previous_errors) < self.num_error_samples:
                self.previous_errors = np.vstack((self.previous_errors, current_error))

            else:
                if (
                    np.std(previous_distance) < self.max_distance_variation
                    and current_distance < 0.1
                ):
                    self.error_history.append(
                        (self.current_target, current_distance, elapsed_time)
                    )
                    print(
                        f"Target: {self.current_target}, Error: {round(1e3 * current_distance, 2)} mm, Elapsed Time: {round(elapsed_time,2)} s"
                    )

                    if self.target_index < len(self.target_sequence) - 1:
                        self.target_index += 1
                        self.send_target(self.target_sequence[self.target_index])
                    else:
                        self.save_history()
                        sys.exit()

                elif elapsed_time > 30:
                    print("Timeout reached. Ending performance evaluation...")
                    self.save_history()
                    self.destroy_node()
                    sys.exit()

    def save_history(self):
        try:
            os.makedirs(os.path.dirname(self.path_to_log_file), exist_ok=True)
        except OSError:
            pass

        with open(self.path_to_log_file + "error_history_" + self.suffix + ".txt", "w") as f:
            for target, error, elapsed_time in self.error_history:
                f.write(f"Target: {target}, Error: {error}, Elapsed Time: {elapsed_time}\n")
        with open(self.path_to_log_file + "full_history_" + self.suffix + ".csv", "w") as f:
            writer = csv.writer(f)
            for target, error, ric_effort, croc_effort, real_effort, ddq, time in zip(
                self.full_target_history,
                self.full_error_history,
                self.full_ric_effort_history,
                self.full_croc_effort_history,
                self.full_real_effort_history,
                self.full_ddq_history,
                self.full_time_history,
            ):
                writer.writerow(
                    np.hstack(
                        (
                            time,
                            target,
                            error.T,
                            croc_effort.T,
                            ric_effort.T,
                            real_effort.T,
                            ddq.T,
                        )
                    )
                )


def main(args=None):
    rclpy.init(args=args)
    print("Starting performance evaluation...")
    target_sequence = [
        [0.5, -0.35, 0.5],
        [0.7, -0.35, 0.5],
        [0.7, -0.35, 0.8],
        [0.5, -0.35, 0.8],
        [0.5, 0.35, 0.8],
        [0.7, 0.35, 0.8],
        [0.7, 0.35, 0.5],
        [0.5, 0.35, 0.5],
    ]
    performance_evaluator = PerformanceEvaluator(target_sequence)
    rclpy.spin_once(performance_evaluator)  # Wait for the first target to be sent
    performance_evaluator.send_target(target_sequence[0])  # Start the evaluation
    rclpy.spin(performance_evaluator)


if __name__ == "__main__":
    main()
