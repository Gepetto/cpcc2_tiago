# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
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

from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_pal.include_utils import include_launch_py_description
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pveg_chained_controller_launch = include_launch_py_description(
        "cpcc2_tiago", ["launch", "pveg_chained_controller.launch.py"]
    )

    crocoddyl_controller_launch = include_launch_py_description(
        "cpcc2_tiago", ["launch", "crocoddyl_controller.launch.py"]
    )

    package_bin_dir = os.path.join(get_package_share_directory("cpcc2_tiago"), "bin")
    executable_path = os.path.join(package_bin_dir, "parallel_croc_solver")

    threaded_solver_launch = ExecuteProcess(cmd=[executable_path], output="screen")

    ld = LaunchDescription()

    ld.add_action(pveg_chained_controller_launch)

    ld.add_action(
        TimerAction(period=1.0, actions=[crocoddyl_controller_launch])
    )  # We wait for the pveg_chained_controller to fully load,then launch the crocoddyl one

    ld.add_action(
        TimerAction(period=3.0, actions=[threaded_solver_launch])
    )  # We wait for the pveg_chained_controller to fully load,then launch the crocoddyl one

    return ld
