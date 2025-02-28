#!/usr/bin/env python3

import copy
import math
from math import pi as PI
from typing import Dict, List, Literal, Union

import numpy as np
import rclpy
from arm_msgs.msg import ArmCommand, ArmStatus, JointCommand, JointStatus
from dynamixel_sdk import GroupBulkRead, GroupBulkWrite, PacketHandler, PortHandler
from rclpy.node import Node
from std_msgs.msg import Empty, Float32, Float64


class DynamixelConfig:
    def __init__(self, id, name, current_limit, pwm_limit):
        self.id = id
        self.name = name
        self.current_limit = current_limit
        self.pwm_limit = pwm_limit


class DynamixelState:
    def __init__(self, current_limit, pwm_limit, op_mode, value):
        self.current_limit = current_limit
        self.pwm_limit = pwm_limit
        self.op_mode = op_mode
        self.halted = False
        self.value = value


class DynamixelController:
    """Controller for a chain of Protocol 2.0 MX64s Dynamixels."""

    class ControlTable:
        ADDR_TORQUE_ENABLE = 64
        ADDR_GOAL_POSITION = 116
        ADDR_GOAL_CURRENT = 102
        ADDR_GOAL_VELOCITY = 104
        ADDR_PRESENT_POSITION = 132
        ADDR_PRESENT_VELOCITY = 128
        ADDR_PRESENT_CURRENT = 126
        ADDR_OPERATING_MODE = 11
        ADDR_CURRENT_LIMIT = 38
        ADDR_PWM_LIMIT = 36
        ADDR_HOMING_OFFSET = 20

    class OperatingMode:
        CURRENT_CONTROL_MODE = 0

        VELOCITY_CONTROL_MODE = 1
        POSITION_CONTROL_MODE = 3
        EXT_POSITION_CONTROL_MODE = 4
        CURRENT_BASED_POSITION_CONTROL_MODE = 5
        PWM_CONTROL_MODE = 16

    def __init__(
        self,
        node: Node,
        configs: List[DynamixelConfig],
        device_name: str = "/dev/ttyUSB0",
        baud_rate: int = 57600,
    ):
        self.node = node  # the ros2 node for pubs and subs and stuff

        # config related information
        self.typing = False
        self.configs: List[DynamixelConfig] = configs
        self.ids = [config.id for config in configs]
        # set the left, right, and elbow roll IDs
        for DC in configs:
            # set the left ID
            if DC.name == "diff2":
                self.LEFT_DYNA_ID = DC.id
            elif DC.name == "diff1":
                self.RIGHT_DYNA_ID = DC.id
            elif DC.name == "gripper":
                self.GRIPPER_DYNA_ID = DC.id
            elif DC.name == "typing":
                self.TYPING_DYNA_ID = DC.id
                self.typing = True
                self.typing_sub = node.create_subscription(
                    Float32, "/arm/command/typing", self.handle_typing, 1
                )
                self.typing_pub = node.create_publisher(Float32, "/arm/status/typing", 1)

        self.baud_rate = baud_rate
        self.device_name = device_name
        self.DEGREES_PER_PULSE = 0.088 / 2
        self.goal_roll = 0.0
        self.goal_pitch = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0

        self.is_moving = False
        self.calibrate_home = False

        self.portHandler = PortHandler(device_name)
        self.packetHandler = PacketHandler(2.0)  # No support for 1.0!

        self.groupread_num = GroupBulkRead(self.portHandler, self.packetHandler)
        self.groupwrite_num = GroupBulkWrite(self.portHandler, self.packetHandler)

        self.carousel_dyna = DynamixelConfig(3, "carousel", 300, 400)

        self.posn_sub = node.create_subscription(
            Float64, "/life_detection/goal_carousel_posn", self.move_carousel, 1
        )
        self.posn_pub = node.create_publisher(Float64, "/life_detection/curr_carousel_posn", 1)

        # Open port
        self.portHandler.openPort()
        node.get_logger().info("Succeeded to open the port")

        baudRateRet = self.portHandler.setBaudRate(baud_rate)

        node.get_logger().info(f"Change the baudrate result: {baudRateRet}")

        # offsets
        self.offsets: Dict[int, float] = {}
        self.calibrate_home_position()

        # subscriber
        self.status_sub = node.create_subscription(
            ArmCommand, "/arm/command/current", self.handle_status, 1
        )

        # publisher
        self.status_pub = node.create_publisher(ArmStatus, "/arm/status/dynamixel", 1)
        self.arm_status_msg = ArmStatus()

        # new subscriber for switching between lie group/not lie group stuff
        self.rotation_mode_lie = True
        self.rotation_mode_sub = node.create_subscription(
            Empty, "/arm/dynamixel/rotation_mode", self.update_rotation_mode, 1
        )

        self.previous_states = {}
        self.next_states = {}

        self.operating_modes = {
            id: DynamixelController.OperatingMode.CURRENT_CONTROL_MODE for id in self.ids
        }

        torque_enable_bulkWrite_data = {}
        current_limit_bulkWrite_data = {}
        pwm_limit_bulkWrite_data = {}
        opmode_bulkWrite_data = {}

        for config in configs:
            torque_enable_bulkWrite_data[config.id] = False

            current_limit_bulkWrite_data[config.id] = config.current_limit
            pwm_limit_bulkWrite_data[config.id] = config.pwm_limit

            opmode_bulkWrite_data[config.id] = (
                DynamixelController.OperatingMode.CURRENT_CONTROL_MODE
            )

            self.previous_states[config.id] = DynamixelState(
                config.current_limit,
                config.pwm_limit,
                DynamixelController.OperatingMode.CURRENT_CONTROL_MODE,
                0,
            )
            self.next_states[config.id] = DynamixelState(
                config.current_limit,
                config.pwm_limit,
                DynamixelController.OperatingMode.CURRENT_CONTROL_MODE,
                0,
            )

        # set torque enable to false to do stuff
        self.set_torque_enable(torque_enable_bulkWrite_data)
        # do stuff
        self.set_current_limit(current_limit_bulkWrite_data)
        self.set_pwm_limit(pwm_limit_bulkWrite_data)
        self.set_operating_mode(opmode_bulkWrite_data)
        # set torque enable to true
        self.set_torque_enable({x: True for x in torque_enable_bulkWrite_data})

        # starting dynamixels with 0 current so they don't move at the start
        self.set_goal_current({dynamixel_id: 0 for dynamixel_id in self.ids})

        self.torque_enabled: Dict[int, bool] = {}
        self.last_torque_check = 0
        # disable torque
        self._write(self.ControlTable.ADDR_TORQUE_ENABLE, False, 1)
        # set the operating mode
        self._write(
            self.ControlTable.ADDR_OPERATING_MODE,
            self.OperatingMode.CURRENT_BASED_POSITION_CONTROL_MODE,
            1,
        )
        # re-enable torque
        self._write(self.ControlTable.ADDR_TORQUE_ENABLE, True, 1)

    def _degrees_to_pulses(self, degrees):
        return int(degrees / self.DEGREES_PER_PULSE)

    def _pulses_to_degrees(self, pulses):
        return int(pulses) * self.DEGREES_PER_PULSE

    def _write(self, addr, data, len: int) -> None:
        if len == 1:
            self.packetHandler.write1ByteTxRx(self.portHandler, self.carousel_dyna.id, addr, data)
        elif len == 4:
            self.packetHandler.write4ByteTxRx(self.portHandler, self.carousel_dyna.id, addr, data)

    def _read(self, addr, len):
        if len == 1:
            data, result, error = self.packetHandler.read1ByteTxRx(
                self.portHandler, self.carousel_dyna.id, addr
            )
        if len == 4:
            data, result, error = self.packetHandler.read4ByteTxRx(
                self.portHandler, self.carousel_dyna.id, addr
            )
        else:
            raise ValueError("Unsupported message length!")
        return data, result, error

    def read_position(self):
        posn_data, result, error = self._read(self.ControlTable.ADDR_PRESENT_POSITION, 4)
        return self._pulses_to_degrees(posn_data), result, error

    def write_position(self, posn: float) -> None:
        raw_posn = self._degrees_to_pulses(posn)
        self._write(self.ControlTable.ADDR_GOAL_POSITION, raw_posn, 4)

    def publish_posn(self):
        data, result, error = self.read_position()
        if result == 0:
            self.posn_pub.publish(Float64(data=data))

    def move_carousel(self, msg: Float64):
        self.write_position(msg.data)

    def queue_state_value_change(self, id, op_mode, value):
        """Queues a next state with the given op mode and value for the given dynamixel id"""
        self.next_states[id].op_mode = op_mode
        self.next_states[id].value = value

    def update_rotation_mode(self, msg: Empty):
        self.rotation_mode_lie = not self.rotation_mode_lie
        self.node.get_logger().info(
            f"Updated rotation mode - using lie groups={self.rotation_mode_lie}"
        )

    def lie_group_pitch_roll(self, goal_pitch: float, goal_roll: float) -> List[float]:
        """Returns the pitch/roll increments needed to reach the goal pitch and goal roll"""

        # 1. Convert goal pitch/roll to SO2
        goal_pitch_so2 = np.array(
            [
                [math.cos(goal_pitch), -1 * math.sin(goal_pitch)],
                [math.sin(goal_pitch), math.cos(goal_pitch)],
            ],
        )
        goal_roll_so2 = np.array(
            [
                [math.cos(goal_roll), -1 * math.sin(goal_roll)],
                [math.sin(goal_roll), math.cos(goal_roll)],
            ],
        )
        # 2. Convert current pitch/roll to SO2
        curr_pitch_so2 = np.array(
            [
                [math.cos(self.current_pitch), -1 * math.sin(self.current_pitch)],
                [math.sin(self.current_pitch), math.cos(self.current_pitch)],
            ],
        )
        curr_roll_so2 = np.array(
            [
                [math.cos(self.current_roll), -1 * math.sin(self.current_roll)],
                [math.sin(self.current_roll), math.cos(self.current_roll)],
            ],
        )

        # 3. Calculate the transformation required to get from current to goal
        curr_to_goal_pitch = np.linalg.inv(curr_pitch_so2) @ goal_pitch_so2
        curr_to_goal_roll = np.linalg.inv(curr_roll_so2) @ goal_roll_so2

        # 4. Extract the angle incr/decr from the transformation matrix
        pitch_incr = 0
        roll_incr = 0

        arccos_pitch = math.acos(curr_to_goal_pitch[0][0])
        arccos_roll = math.acos(curr_to_goal_roll[0][0])

        if curr_to_goal_pitch[1][0] == 0:
            pitch_incr = 0
        else:
            pitch_incr = arccos_pitch * (curr_to_goal_pitch[1][0] / abs(curr_to_goal_pitch[1][0]))

        if curr_to_goal_roll[1][0] == 0:
            roll_incr = 0
        else:
            roll_incr = arccos_roll * (curr_to_goal_roll[1][0] / abs(curr_to_goal_roll[1][0]))

        return [pitch_incr, roll_incr]

    def pitch_roll_dirs_to_diff_vels(self, goal_pitch: float, goal_roll: float):
        pitch_incr, roll_incr = self.lie_group_pitch_roll(goal_pitch, goal_roll)

        self.goal_pitch = self.current_pitch + pitch_incr
        self.goal_roll = self.current_roll + roll_incr

    def queue_pitch_roll(
        self, to_change: Literal["pitch"] | Literal["roll"], op_mode: int, value: float
    ):
        """Queues a change in either pitch or roll"""
        value = math.radians(value)
        if to_change == "roll":
            self.goal_roll = value
        if to_change == "pitch":
            self.goal_pitch = value

        self.next_states[self.LEFT_DYNA_ID].op_mode = op_mode
        self.next_states[self.RIGHT_DYNA_ID].op_mode = op_mode

        # call new function! will udpate goal pitch/roll based on new math
        if self.rotation_mode_lie:
            self.pitch_roll_dirs_to_diff_vels(self.goal_pitch, self.goal_roll)

        self.next_states[self.LEFT_DYNA_ID].value = self.goal_roll - self.goal_pitch
        self.next_states[self.RIGHT_DYNA_ID].value = self.goal_pitch + self.goal_roll

    def halt_handler(self, id, msg):
        self.next_states[id].halted = True

    def calibrate_home_position(self, msg=None):
        """Sets the offsets for all of the dynamixels to be their current raw positions"""
        self.offsets = self.get_raw_position()
        for dynamixel_id, position in self.offsets.items():
            print(
                f"Starting position for ID: {dynamixel_id} is {position}. Set offset to {position}"
            )

    def radians_to_pulses(self, radians):
        """Converts a given radian value to corresponding dynamixel value"""
        if math.isnan(radians):
            return 0

        return int(radians * 180 / (self.DEGREES_PER_PULSE * PI))

    def pulses_to_radians(self, pulses):
        """Converts dynamixel pulses to radian values"""
        return int(pulses) * self.DEGREES_PER_PULSE / 180 * PI

    def handle_typing(self, cmd: Float32) -> None:
        """Handles a typing command"""
        if self.typing:
            self.queue_state_value_change(
                self.TYPING_DYNA_ID,
                DynamixelController.OperatingMode.CURRENT_CONTROL_MODE,
                cmd.data,
            )

    def handle_pitch_cmd(self, cmd: JointCommand) -> None:
        """Handles a wrist pitch JointCommand"""
        if cmd.command_type == JointCommand.COMMAND_TYPE_POSITION:
            self.queue_pitch_roll(
                "pitch",
                DynamixelController.OperatingMode.CURRENT_BASED_POSITION_CONTROL_MODE,
                cmd.value,
            )
        elif cmd.command_type == JointCommand.COMMAND_TYPE_STOP:
            self.halt_handler(self.LEFT_DYNA_ID, None)
            self.halt_handler(self.RIGHT_DYNA_ID, None)

    def handle_roll_cmd(self, cmd: JointCommand) -> None:
        """Handles an wrist roll JointCommand for the elbow roll Dynamixel"""
        if cmd.command_type == JointCommand.COMMAND_TYPE_POSITION:
            self.queue_pitch_roll(
                "roll",
                DynamixelController.OperatingMode.CURRENT_BASED_POSITION_CONTROL_MODE,
                cmd.value,
            )
        elif cmd.command_type == JointCommand.COMMAND_TYPE_STOP:
            self.halt_handler(self.LEFT_DYNA_ID, None)
            self.halt_handler(self.RIGHT_DYNA_ID, None)

    def handle_gripper_cmd(self, cmd: JointCommand) -> None:
        """Handles a JointCommand for the gripper Dynamixel"""
        if cmd.command_type == JointCommand.COMMAND_TYPE_VELOCITY:
            self.queue_state_value_change(
                self.GRIPPER_DYNA_ID,
                DynamixelController.OperatingMode.CURRENT_CONTROL_MODE,
                cmd.value,
            )
        elif cmd.command_type == JointCommand.COMMAND_TYPE_STOP:
            self.halt_handler(self.GRIPPER_DYNA_ID, None)

    def handle_status(self, msg: ArmCommand) -> None:
        """Queues states for the Dynamixels based on the given ArmCommand message"""

        # handle relevant joints
        self.handle_pitch_cmd(msg.wrist_pitch)
        self.handle_roll_cmd(msg.wrist_roll)
        self.handle_gripper_cmd(msg.gripper)

    def update(self):
        """Publish state to ROS."""
        current_limit_bulkWrite_dict = {}
        pwm_limit_bulkWrite_dict = {}
        opmode_bulkWrite_dict = {}

        current_bulkWrite_dict = {}
        currentBasedPosition_bulkWrite_dict = {}

        halt_motor_list = []  # to halt motors

        for dynamixel_id in self.ids:
            prev_state = self.previous_states[dynamixel_id]
            next_state = self.next_states[dynamixel_id]

            # Check if current changed
            if prev_state.current_limit != next_state.current_limit:
                current_limit_bulkWrite_dict[dynamixel_id] = next_state.current_limit

            # Check if pwm changed
            if prev_state.pwm_limit != next_state.pwm_limit:
                pwm_limit_bulkWrite_dict[dynamixel_id] = next_state.pwm_limit

            # If op mode changed
            if prev_state.op_mode != next_state.op_mode:
                opmode_bulkWrite_dict[dynamixel_id] = next_state.op_mode

            if next_state.halted:
                # FIXME: Commented out halted for now
                halt_motor_list.append(dynamixel_id)
                next_state.halted = False  # unset the halted flag
                # set current position as goal position?

            if next_state.op_mode == DynamixelController.OperatingMode.CURRENT_CONTROL_MODE:
                current_bulkWrite_dict[dynamixel_id] = int(next_state.value)
            elif (
                next_state.op_mode
                == DynamixelController.OperatingMode.CURRENT_BASED_POSITION_CONTROL_MODE
            ):
                currentBasedPosition_bulkWrite_dict[dynamixel_id] = next_state.value
            else:
                self.node.get_logger().warn(
                    "Dynamixel op mode is invalid/unsupported by this ros node!"
                )

            self.previous_states[dynamixel_id] = copy.deepcopy(self.next_states[dynamixel_id])

        # bulkWrites after for loop
        self.set_current_limit(current_limit_bulkWrite_dict)
        self.set_pwm_limit(pwm_limit_bulkWrite_dict)
        self.set_operating_mode(opmode_bulkWrite_dict)

        self.set_goal_current(current_bulkWrite_dict)

        self.halt_motor(halt_motor_list)

        self.set_goal_position_current_control(currentBasedPosition_bulkWrite_dict)
        posn_dict = self.get_present_position()

        # set arm status fields
        self.arm_status_msg.wrist_pitch = JointStatus(
            position=math.degrees(self.current_pitch),
            status=JointStatus.STATUS_OK,
            timestamp=self.node.get_clock().now().to_msg(),
        )
        self.arm_status_msg.wrist_roll = JointStatus(
            position=math.degrees(self.current_roll),
            status=JointStatus.STATUS_OK,
            timestamp=self.node.get_clock().now().to_msg(),
        )
        self.arm_status_msg.gripper = JointStatus(
            position=math.degrees(posn_dict[self.GRIPPER_DYNA_ID]),
            status=JointStatus.STATUS_OK,
            timestamp=self.node.get_clock().now().to_msg(),
        )

        # publish!
        self.status_pub.publish(self.arm_status_msg)
        if self.typing:
            self.typing_pub.publish(Float32(data=math.degrees(posn_dict[self.TYPING_DYNA_ID])))

        self.publish_posn()

        # check torques every 7-ish times update is called
        # (update called every 30 milliseconds I think)
        self.last_torque_check += 1
        if self.last_torque_check == 6:
            self.check_torque_enabled()
            self.last_torque_check = 0

    def get_torque_enabled(self):
        """
        Call bulkRead to check torque_enabled values for the Dynamixels.

        Stores the dict in self.torque_enabled dict (declared in __init__).
        """
        torque_enabled_dict = self._bulkRead(DynamixelController.ControlTable.ADDR_TORQUE_ENABLE, 1)
        self.torque_enabled = {
            dynamixel_id: torque_enabled_val
            for (dynamixel_id, torque_enabled_val) in torque_enabled_dict.items()
        }
        self.node.get_logger().debug("get_torque_enabled called by check_torque_enable")

    def check_torque_enabled(self):
        """
        Check the torque_enabled value of each Dynamixel.

        Creates a dict of Dynamixels with torque not enabled and
        sets torque_enabled to True.
        """
        torque_enabled_bulkWrite_dict = {
            dxl_id: True for dxl_id, enabled in self.torque_enabled.items() if not enabled
        }
        self.set_torque_enable(torque_enabled_bulkWrite_dict)

    def get_raw_position(self) -> Dict[int, float]:
        """Return the positions read from each Dynamixel in encoder counts (not radians)."""
        return self._bulkRead(DynamixelController.ControlTable.ADDR_PRESENT_POSITION, 4)

    def mod_pulses(self, pulse: int) -> int:
        return (pulse + 2**31) % 2**32 - 2**31

    def get_present_position(self) -> Dict[int, float]:
        """Return the present position of each dynamixel in radians."""
        res = self.get_raw_position()

        for dynamixel_id, position in res.items():
            res[dynamixel_id] = position - self.offsets[dynamixel_id]

        res_radians_dict = {
            dynamixel_id: self.pulses_to_radians(self.mod_pulses(res_pulse))
            for dynamixel_id, res_pulse in res.items()
        }

        self.current_roll = (
            res_radians_dict[self.LEFT_DYNA_ID] + res_radians_dict[self.RIGHT_DYNA_ID]
        ) / 2
        self.current_pitch = (
            res_radians_dict[self.RIGHT_DYNA_ID] - res_radians_dict[self.LEFT_DYNA_ID]
        ) / 2
        return res_radians_dict

    def halt_motor(self, id_halt_list):
        """
        Write the present positions to the Dynamixels so they hold their position.

        Works with position control modes.
        """
        present_positions = self.get_raw_position()
        halt_bulkWrite_dict = {}

        for dynamixel_id in id_halt_list:
            for dxl_id, posn in present_positions.items():
                if dynamixel_id == dxl_id:
                    halt_bulkWrite_dict[dynamixel_id] = posn

        self._bulkWrite(halt_bulkWrite_dict, DynamixelController.ControlTable.ADDR_GOAL_POSITION, 4)

    def set_goal_position(self, id_positions_dict: Dict[int, int]):
        """Set the goal position [0, 4095]."""
        id_positions_dict = {
            dxl_id: self.pulses_to_radians(pos) for dxl_id, pos in id_positions_dict.items()
        }

        if self.operating_modes[id] not in (
            DynamixelController.OperatingMode.POSITION_CONTROL_MODE,
            DynamixelController.OperatingMode.EXT_POSITION_CONTROL_MODE,
        ):
            self.node.get_logger().warn(
                f"Trying to set_goal_position when dynamixel {id} "
                "is not configured in a position-based mode! Changing mode."
            )
            self.set_operating_mode({id: DynamixelController.OperatingMode.POSITION_CONTROL_MODE})

        self._bulkWrite(id_positions_dict, DynamixelController.ControlTable.ADDR_GOAL_POSITION, 4)

    def set_goal_position_current_control(self, id_positions_dict: Dict[int, float]):
        """
        Set the goal position of the Dynamixel while obeying the current limit.

        The SDK accepts Goal Positions from the extended position range, i.e.,
        -1,048,575 through 1,048,575 or -256[rev] through 256[rev].
        Reference: https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/#goal-position116
        Therefore, given this function accepts radians,
        it accepts values -256 * 2 * PI through 256 * 2 * PI

        id_positions_dict: (Dict[int, float]):
            Dict mapping ids to given position commands in radians (angles)
        """
        # Converting commands from radians to "pulses"
        # (the unit that the dynamixel takes, defined in datasheet)
        id_positions_dict = {
            dynamixel_id: self.radians_to_pulses(position_rad) + self.offsets[dynamixel_id]
            for dynamixel_id, position_rad in id_positions_dict.items()
        }

        for dynamixel_id in id_positions_dict.keys():
            if (
                self.operating_modes[dynamixel_id]
                != DynamixelController.OperatingMode.CURRENT_BASED_POSITION_CONTROL_MODE
            ):
                self.node.get_logger().warn(
                    "Trying to set_goal_position_current_control "
                    f"when dynamixel {dynamixel_id} is not configured in a "
                    "CURRENT_BASED_POSITION_CONTROL_MODE mode!"
                )
                self.set_operating_mode(
                    {
                        dynamixel_id: (
                            DynamixelController.OperatingMode.CURRENT_BASED_POSITION_CONTROL_MODE
                        )
                        for dynamixel_id in id_positions_dict.keys()
                    }
                )
                break

        # According to tidscontrol table, ADDR_GOAL_POSITION is 4 bytes long
        self._bulkWrite(id_positions_dict, DynamixelController.ControlTable.ADDR_GOAL_POSITION, 4)

    def set_goal_current(self, current_data_dict: Dict[int, float]):
        """Take in a dict with Dynamixel IDs and bulkwrites their corresponding goal currents."""
        for dynamixel_id in current_data_dict.keys():
            if self.operating_modes[dynamixel_id] not in (
                DynamixelController.OperatingMode.CURRENT_CONTROL_MODE,
                DynamixelController.OperatingMode.CURRENT_BASED_POSITION_CONTROL_MODE,
            ):
                self.node.get_logger().warn(
                    f"Trying to set_goal_current when dynamixel {dynamixel_id} "
                    "is not configured in a current-based mode!"
                )
                self.set_operating_mode(
                    {dynamixel_id: DynamixelController.OperatingMode.CURRENT_CONTROL_MODE}
                )

        self._bulkWrite(current_data_dict, DynamixelController.ControlTable.ADDR_GOAL_CURRENT, 2)

    def set_torque_enable(self, id_enabled_dict: Dict[int, bool]):
        """Bulkwrites whether torque is on for each of the provided Dynamixels."""
        id_enabled: Dict[int, int] = {
            dynamixel_id: int(enabled) for dynamixel_id, enabled in id_enabled_dict.items()
        }
        self._bulkWrite(id_enabled, DynamixelController.ControlTable.ADDR_TORQUE_ENABLE, 1)

    def set_eeprom_val(self, id_val_dict: Dict[int, int], val_addr: int, size: int):
        # eeprom vals in dynamixels require torque enabled to be false
        # (cannot be changed when the dynamixel is in motion)
        self.set_torque_enable({dynamixel_id: False for dynamixel_id in id_val_dict.keys()})

        self._bulkWrite(id_val_dict, val_addr, size)

        self.set_torque_enable({dynamixel_id: True for dynamixel_id in id_val_dict.keys()})

    def set_operating_mode(self, id_opmode_dict):
        """
        Change the operating mode of the dynamixels.

        Sets the operating mode to one of:
        Current Control Mode: 0
        Velocity Control Mode: 1
        Position Control Mode: 3
        Extended Position Control Mode: 4
        Current Based Position Control Mode: 5
        PWM Control Mode: 16

        Link: https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/#operating-mode11
        """
        # check for valid operating mode - TODO: add more op modes if needed
        for dynamixel_id, op_mode in id_opmode_dict.items():
            if op_mode not in (
                DynamixelController.OperatingMode.CURRENT_CONTROL_MODE,
                DynamixelController.OperatingMode.CURRENT_BASED_POSITION_CONTROL_MODE,
            ):
                self.node.get_logger().error(
                    f"Trying to write an invalid op mode to {dynamixel_id}."
                )
                if dynamixel_id == self.LEFT_DYNA_ID or dynamixel_id == self.RIGHT_DYNA_ID:
                    id_opmode_dict[dynamixel_id] = (
                        DynamixelController.OperatingMode.CURRENT_BASED_POSITION_CONTROL_MODE
                    )
                    self.operating_modes[dynamixel_id] = id_opmode_dict[dynamixel_id]
                else:
                    id_opmode_dict[dynamixel_id] = (
                        DynamixelController.OperatingMode.CURRENT_CONTROL_MODE
                    )
                    self.operating_modes[dynamixel_id] = id_opmode_dict[dynamixel_id]
            else:
                self.operating_modes[dynamixel_id] = id_opmode_dict[dynamixel_id]

        self.set_eeprom_val(id_opmode_dict, DynamixelController.ControlTable.ADDR_OPERATING_MODE, 1)

    def set_current_limit(self, id_current_dict: Dict[int, int]):
        """
        Set the current limit [0-1941].

        Link: https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/current-limit38
        """
        self.set_eeprom_val(id_current_dict, DynamixelController.ControlTable.ADDR_CURRENT_LIMIT, 2)

    def set_pwm_limit(self, id_pwm_dict: Dict[int, int]):
        """
        Set the PWM limit [0-885].

        Link: https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/#pwm-limit
        """
        self.set_eeprom_val(id_pwm_dict, DynamixelController.ControlTable.ADDR_PWM_LIMIT, 2)

    def _bulkWrite(
        self,
        id_data_dict: Union[Dict[int, int], Dict[int, float]],
        data_address: int,
        data_length_bytes: int,
    ):
        """
        Use the Dynamixel SDK Bulk Write.

        id_data_dict: Maps dynamixel IDs (ints) to the values to update for each dynamixel
        data_address: Address in the dynamixel control table to write to
        data_length_bytes: Length of bytes of each spot in the control table
        """
        if not id_data_dict:
            return

        for dynamixel_id, data_value in id_data_dict.items():
            dxl_addParam_result = self.groupwrite_num.addParam(
                dynamixel_id, data_address, data_length_bytes, data_value
            )
            if dxl_addParam_result is not True:
                self.node.get_logger().warn("_bulkWrite: Error adding parameter")

        bulkwrite_result = self.groupwrite_num.txPacket()
        self.groupwrite_num.clearParam()

        return bulkwrite_result

    def _bulkRead(self, data_address: int, data_length_bytes: int):
        """
        Uses the Dynamixel SDK Bulk Read.

        data_address: Address in the Dynamixel control table to read from
        data_length_bytes: the length, in bytes, of the content being read per dynamixel
        """
        for dynamixel_id in self.ids:
            addparam_result = self.groupread_num.addParam(
                dynamixel_id, data_address, data_length_bytes
            )
            if addparam_result is not True:
                self.node.get_logger().warn("_bulkRead: Error adding parameter")
            else:  # added for debugging - 1/19/23
                pass

        # TODO - ACTUALLY USE BULKREAD RESULT INSTEAD OF JUST ASSUMING 0 = WRONG
        # SO WE CAN TELL IF DATA WAS ACTUALLY FOUND OR NOT
        # NOTE or change data result so that it is not returning 0 and returns something else
        # for now, return null
        bulkRead_result = self.groupread_num.txRxPacket()  # noqa: F841

        if bulkRead_result != 0:
            self.node.get_logger().error(
                f"Error occured during bulkread, result was: {bulkRead_result}"
            )
        data_dict = {}
        for dynamixel_id in self.ids:
            data_result = self.groupread_num.getData(dynamixel_id, data_address, data_length_bytes)
            data_dict[dynamixel_id] = data_result
            if data_result is None:
                self.node.get_logger().error("_bulkRead: data not found - returned None")

        self.groupread_num.clearParam()

        return data_dict


class DynamixelControl(Node):
    def __init__(self) -> None:
        super().__init__("dynamixel_controller")

        self.declare_parameter("typing_dynamixel", False)
        self.typing_dynamixel = self.get_parameter("typing_dynamixel").value

        # need more things here? TODO NOTE
        dynamixel_configs = [
            DynamixelConfig(31, "diff1", 500, 300),  # ID: 24
            DynamixelConfig(32, "diff2", 500, 300),  # ID: 23
            DynamixelConfig(33, "gripper", 375, 885),  # ID: 22
        ]
        # added in same order as power chain

        if self.typing_dynamixel:
            dynamixel_configs.append(DynamixelConfig(5, "typing", 400, 300))
        self.dyna_control = DynamixelController(
            self,
            dynamixel_configs,
            device_name="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT6RW5R9-if00-port0",
            baud_rate=1000000,
        )
        # TODO: do udev rules on tx2/orin so we can use "/dev/dynamixel-controller"
        # NOTE: the baudrate of the dynamixels on the real arm is 1000000

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.update)

    def update(self):
        self.dyna_control.update()


def main(args=None):
    rclpy.init(args=args)
    controller = DynamixelControl()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
