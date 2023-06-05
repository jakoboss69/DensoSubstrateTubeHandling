from dataclasses import dataclass, asdict

import numpy as np
import zmq
import yaml


class DensoCommands:
    CHECK_COM = "check_com"
    OPEN_SESSION = "open_session"
    CLOSE_SESSION = "close_session"
    SET_SERVO = "set_servo"
    SET_TOOLKIT = "set_toolkit"
    GET_CURRENT_POSITION = "get_current_position"
    MOVE_BY_COORDINATE_J = "move_by_coordinate_j"
    MOVE_BY_COORDINATE_C = "move_by_coordinate_c"
    MOVE_BY_COORDINATE_T = "move_by_coordinate_t"
    SET_INTERNAL_SPEED = "set_internal_speed"
    SET_EXTERNAL_SPEED = "set_external_speed"
    SET_TOOL_PARAMS = "set_tool_params"
    SET_ACTIVE_TOOL = "set_active_tool"


class Interpolation:
    Move_P = "move_p"
    Move_L = "move_l"


class PassMotion:
    O = "@O"
    P = "@P"
    E = "@E"


class MotionOption:
    Default = "default"
    Next = "next"


@dataclass
class PoseData:
    def save(self, file_path):
        with open(file_path, "w") as file:
            yaml.dump(asdict(self), file)

    def load(self, file_path):
        with open(file_path, "r") as file:
            return self.__class__(**yaml.load(file, Loader=yaml.FullLoader))


@dataclass
class TransPose(PoseData):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    ox: float = 0.0
    oy: float = 0.0
    oz: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    figure: int = 0

    def to_msg_str(self):
        return f"{self.x};{self.y};{self.z};{self.ox};{self.oy};{self.oz};{self.ax};{self.ay};{self.az};{self.figure};"


@dataclass
class CartesianPose(PoseData):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0
    figure: int = 0.0

    def to_msg_str(self):
        return f"{self.x};{self.y};{self.z};{self.rx};{self.ry};{self.rz};{self.figure};"


@dataclass
class JointPose(PoseData):
    j1: float = 0.0
    j2: float = 0.0
    j3: float = 0.0
    j4: float = 0.0
    j5: float = 0.0
    j6: float = 0.0

    def to_msg_str(self):
        return f"{self.j1};{self.j2};{self.j3};{self.j4};{self.j5};{self.j6};"


def trans_pose_to_transformation_matrix(trans_pose: TransPose):
    tmat = np.diag([1.0, 1.0, 1.0, 1.0])
    y_vec = np.asarray([trans_pose.ox, trans_pose.oy, trans_pose.oz])
    y_vec /= np.linalg.norm(y_vec)
    z_vec = np.asarray([trans_pose.ax, trans_pose.ay, trans_pose.az])
    z_vec /= np.linalg.norm(z_vec)
    x_vec = np.cross(y_vec, z_vec)
    r = np.stack([x_vec, y_vec, z_vec], axis=1)
    tmat[:3, :3] = r
    tmat[:3, 3] = np.asarray([trans_pose.x, trans_pose.y, trans_pose.z])
    return tmat, trans_pose.figure


def transformation_matrix_to_trans_pose(tmat: np.array, figure: int):
    trans_pose = TransPose(x=tmat[0, 3],
                           y=tmat[1, 3],
                           z=tmat[2, 3],
                           ox=tmat[0, 1],
                           oy=tmat[1, 1],
                           oz=tmat[2, 1],
                           ax=tmat[0, 2],
                           ay=tmat[1, 2],
                           az=tmat[2, 2],
                           figure=figure)
    return trans_pose


class DensoController:
    def __init__(self):
        self.ip = None
        self.port = None
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)

    def connect(self, communication_ip, communication_port):
        self.ip = communication_ip
        self.port = communication_port
        self.socket.connect(f"{self.ip}:{self.port}")
        return self.check_connection()

    def send_command(self, command, data):
        msg_str = f"{command}#{data}"
        self.socket.send_string(msg_str)
        reply_msg = self.socket.recv().decode()
        reply_command, reply_data = reply_msg.split("#")
        return reply_command, reply_data

    def check_connection(self):
        reply_command, reply_data = self.send_command(DensoCommands.CHECK_COM, "")
        return reply_command == DensoCommands.CHECK_COM and reply_data == "ok"

    def open_session(self, robot_ip: str = "192.168.1.210"):
        """
        Connects the communication pc with the robot. The ip is hardcoded on the robot controller.
        """
        reply_command, reply_data = self.send_command(DensoCommands.OPEN_SESSION, robot_ip)
        return reply_command == DensoCommands.OPEN_SESSION and reply_data == "ok"

    def close_session(self):
        reply_command, reply_data = self.send_command(DensoCommands.CLOSE_SESSION, "")
        return reply_command == DensoCommands.CLOSE_SESSION and reply_data == "ok"

    def set_servo(self, status: bool):
        """
        Turns on/off servos.
        Status: True = 'ON', False = 'OFF'
        """
        cmd = DensoCommands.SET_SERVO
        data = "ON" if status else "OFF"
        reply_command, reply_data = self.send_command(cmd, data)
        return reply_command == DensoCommands.SET_SERVO

    def set_toolkit(self, status: bool):
        """
        Activates robot toolkit (makes the robot able to receive commands from LabVIEW).
        Status: True = 'START', False = 'STOP'
        """
        cmd = DensoCommands.SET_TOOLKIT
        data = "START" if status else "STOP"
        reply_command, reply_data = self.send_command(cmd, data)
        return reply_command == DensoCommands.SET_TOOLKIT

    def move_to_trans(self,
                      pose: TransPose,
                      interpolation: Interpolation = Interpolation.Move_P,
                      pass_motion: PassMotion = PassMotion.O,
                      move_option: MotionOption = MotionOption.Default):
        cmd = DensoCommands.MOVE_BY_COORDINATE_T
        data = f"{pose.to_msg_str()}|{interpolation}|{pass_motion}|{move_option}"
        reply_command, reply_data = self.send_command(cmd, data)
        return reply_command == DensoCommands.MOVE_BY_COORDINATE_T

    def move_to_joint(self,
                      pose: JointPose,
                      interpolation: Interpolation = Interpolation.Move_P,
                      pass_motion: PassMotion = PassMotion.O,
                      move_option: MotionOption = MotionOption.Default):
        cmd = DensoCommands.MOVE_BY_COORDINATE_J
        data = f"{pose.to_msg_str()}|{interpolation}|{pass_motion}|{move_option}"
        reply_command, reply_data = self.send_command(cmd, data)
        return reply_command == DensoCommands.MOVE_BY_COORDINATE_J

    def move_to_cart(self,
                     pose: CartesianPose,
                     interpolation: Interpolation = Interpolation.Move_P,
                     pass_motion: PassMotion = PassMotion.O,
                     move_option: MotionOption = MotionOption.Default):
        cmd = DensoCommands.MOVE_BY_COORDINATE_C
        data = f"{pose.to_msg_str()}|{interpolation}|{pass_motion}|{move_option}"
        reply_command, reply_data = self.send_command(cmd, data)
        return reply_command == DensoCommands.MOVE_BY_COORDINATE_C

    def move_to_transformation_matrix(self,
                                      tmat: np.array,
                                      figure: int,
                                      interpolation: Interpolation = Interpolation.Move_P,
                                      pass_motion: PassMotion = PassMotion.O,
                                      move_option: MotionOption = MotionOption.Default):
        trans_pose = transformation_matrix_to_trans_pose(tmat, figure=figure)
        print(trans_pose)
        return self.move_to_trans(pose=trans_pose,
                                  interpolation=interpolation,
                                  pass_motion=pass_motion,
                                  move_option=move_option)

    def disconnect(self):
        pass

    def get_position_joint(self):
        cmd = DensoCommands.GET_CURRENT_POSITION
        data = "J"
        reply_command, reply_data = self.send_command(cmd, data)
        reply_data = np.asarray(reply_data.split(";")).astype(float)
        return JointPose(*reply_data)

    def get_position_cart(self):
        cmd = DensoCommands.GET_CURRENT_POSITION
        data = "C"
        reply_command, reply_data = self.send_command(cmd, data)
        reply_data = np.asarray(reply_data.split(";")).astype(float)
        return CartesianPose(*reply_data)

    def get_position_trans(self):
        cmd = DensoCommands.GET_CURRENT_POSITION
        data = "T"
        reply_command, reply_data = self.send_command(cmd, data)
        reply_data = np.asarray(reply_data.split(";")).astype(float)
        return TransPose(*reply_data)

    def get_position_transformation_matrix(self):
        trans_pose = self.get_position_trans()
        return trans_pose_to_transformation_matrix(trans_pose)

    def set_internal_speed(self, speed: float):
        """
        Sets the internal speed of the robot. Speed should be in the range 0 - 100 (%).
        Actual robot speed is internal_speed * external_speed (0 - 100 %).
        """
        cmd = DensoCommands.SET_INTERNAL_SPEED
        data = str(speed)
        reply_command, reply_data = self.send_command(cmd, data)
        return reply_command == DensoCommands.SET_INTERNAL_SPEED

    def set_external_speed(self, speed: float):
        """
        Sets the external speed of the robot. Speed should be in the range 0 - 100 (%).
        Actual robot speed is internal_speed * external_speed (0 - 100 %).
        """
        cmd = DensoCommands.SET_EXTERNAL_SPEED
        data = str(speed)
        reply_command, reply_data = self.send_command(cmd, data)
        return reply_command == DensoCommands.SET_EXTERNAL_SPEED

    def set_tool_params(self, x: float, y: float, z: float, rx: float, ry: float, rz: float, tool_number: int):
        """
        Adds a tool offset to the controller memory. Tool number between 1 - 20.
        """
        if tool_number != 0:
            cmd = DensoCommands.SET_TOOL_PARAMS
            data = f"{x};{y};{z};{rx};{ry};{rz};{tool_number}|"
            reply_command, reply_data = self.send_command(cmd, data)
            return reply_command == DensoCommands.SET_TOOL_PARAMS
        else:
            print("ERROR: Cannot set tool number 0 params")
            return False

    def set_active_tool(self, tool_number: int):
        """
        Sets the active tool from controller memory. Tool number = 0 gives zero offset.
        """
        cmd = DensoCommands.SET_ACTIVE_TOOL
        data = f"{tool_number}"
        reply_command, reply_data = self.send_command(cmd, data)
        return reply_command == DensoCommands.SET_ACTIVE_TOOL




def debug():
    trans_pose = TransPose(1.0, 2.0, 3.0, 0.5, 0.5, 0.0, 0.0, 0.0, 0.5)
    tmat = trans_pose_to_transformation_matrix(trans_pose)
    print(tmat)
    print(np.linalg.det(tmat[:3, :3]))


if __name__ == '__main__':
    debug()
