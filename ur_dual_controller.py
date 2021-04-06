import os
import struct
import socket
import program_builder as pb
import numpy as np
import ur_robot
import trajectory as traj


class URDualController(object):

    def __init__(self, master_robot_ip='10.2.0.50', slave_robot_ip='10.2.0.51', control_pc_ip='10.2.0.100'):
        # left arm

        self._lft_arm = ur_robot.URRobot(master_robot_ip)
        self._rgt_arm = ur_robot.URRobot(slave_robot_ip)
        # setup control pc server
        self._pc_server_socket_addr = (control_pc_ip, 0)  # 0: the system finds an available port
        self._pc_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._pc_server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._pc_server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._pc_server_socket.bind(self._pc_server_socket_addr)
        self._pc_server_socket.listen(5)
        self._jnts_scaler = 1e6
        self._pb = pb.ProgramBuilder()
        self._script_dir = os.path.dirname(__file__)
        self._pb.load_prog(os.path.join(self._script_dir, "urscripts/moderndriver_master.script"))
        self._master_modern_driver_urscript = self._pb.get_program_to_run()
        self._master_modern_driver_urscript = self._master_modern_driver_urscript.replace("parameter_pc_ip",
                                                                                          self._pc_server_socket.getsockname()[
                                                                                              0])
        self._master_modern_driver_urscript = self._master_modern_driver_urscript.replace("parameter_pc_port",
                                                                                          str(
                                                                                              self._pc_server_socket.getsockname()[
                                                                                                  1]))
        self._master_modern_driver_urscript = self._master_modern_driver_urscript.replace("parameter_slave_ip",
                                                                                          slave_robot_ip)
        self._master_modern_driver_urscript = self._master_modern_driver_urscript.replace("parameter_jnts_scaler",
                                                                                          str(self._jnts_scaler))
        self._pb.load_prog(os.path.join(self._script_dir, "urscripts_cbseries/moderndriver_cbseries_slave.script"))
        self._slave_modern_driver_urscript = self._pb.get_program_to_run()
        self._slave_modern_driver_urscript = self._slave_modern_driver_urscript.replace("parameter_master_ip",
                                                                                        master_robot_ip)
        self._slave_modern_driver_urscript = self._slave_modern_driver_urscript.replace("parameter_jnts_scaler",
                                                                                        str(self._jnts_scaler))
        self._trajt = traj.Trajectory(method='quintic')

    def move_jntspace_path(self, path, control_frequency=.008, interval_time=1.0, interpolation_method=None):
        """
        :param path: a list of 1x12 arrays
        :param control_frequency: the program will sample interval_time/control_frequency confs, see motion.trajectory
        :param interval_time: equals to expandis/speed, speed = degree/second
                              by default, the value is 1.0 and the speed is expandis/second
        :param interpolation_method
        :return:
        author: weiwei
        date: 20210404
        """
        self._trajt.set_interpolation_method(interpolation_method)
        interpolated_confs, interpolated_spds = self._trajt.piecewise_interpolation(path, control_frequency,
                                                                                     interval_time)
        # upload a urscript to connect to the pc server started by this class
        self._lft_arm.send_program(self._master_modern_driver_urscript)
        self._rgt_arm.send_program(self._slave_modern_driver_urscript)
        # accept arm socket
        pc_server_socket, pc_server_socket_addr = self._pc_server_socket.accept()
        print("PC server connected by ", pc_server_socket_addr)
        # send trajectory
        keepalive = 1
        buf = bytes()
        for id, conf in enumerate(interpolated_confs):
            if id == len(interpolated_confs) - 1:
                keepalive = 0
            jointsradint = [int(jnt_value * self._lft_arm_hnd.jnts_scaler) for jnt_value in conf]
            buf += struct.pack('!iiiiiiiiiiiii', jointsradint[0], jointsradint[1], jointsradint[2],
                               jointsradint[3], jointsradint[4], jointsradint[5], jointsradint[6],
                               jointsradint[7], jointsradint[8], jointsradint[9], jointsradint[10],
                               jointsradint[11], keepalive)
        pc_server_socket.send(buf)
        pc_server_socket.close()

    def get_jnt_values(self):
        """
        get the joint angles of both arms
        :return: 1x12 array
        author: ochi, revised by weiwei
        date: 20180410, 20210404
        """
        return np.array(self._lft_arm.get_jnt_values() + self._rgt_arm.get_jnt_values())
