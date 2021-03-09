import requests
import json
import time
import socket

from .type import *

class LebaiScene:
    def __init__(self, ip, scene_id=0, task_id=0):
        self.ip = ip
        self.scene_id = scene_id
        self.task_id = task_id
        self.robot = LebaiRobot(ip)

    def start(self, loop=1, force=False):
        payload = {
            'execute_count': loop,
            'clear': 1 if force else 0
        }
        if self.task_id > 0:
            payload['task_id'] = self.task_id
        else:
            payload['scene_id'] = self.scene_id
        payload = json.dumps(payload)
        r = requests.post("http://{0}/public/task".format(self.ip), data=payload)
        r.raise_for_status()
        r = r.json()
        if r['code'] == 0:
            self.task_id = r['data']['id']
        else:
            raise RequestError(r)

    def pause(self):
        self.robot.action('pause_task', sleep=1)

    def resume(self):
        self.robot.action('resume_task', sleep=1)

    def stop(self):
        self.robot.action('stop_task', sleep=1)
    
    def result(self):
        r = requests.get("http://{0}/public/task".format(self.ip), params={'id': self.task_id})
        r.raise_for_status()
        r = r.json()
        if r['code'] == 0:
            return r['data']
        else:
            raise RequestError(r)

    def status(self):
        return TaskStatus(self.result()['status'])

    def done(self):
        status = self.status()
        if status == TaskStatus.SUCCESS or status == TaskStatus.STOPPED or status == TaskStatus.ABORTED:
            return True
        return False

    def run(self, loop=1):
        output = b''
        self.start()
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.ip, 5180))
            while True:
                output += s.recv(1024)
                if self.done():
                    break
        return output.decode('utf-8')

class IODevice:
    def __init__(self, robot, device_type=IODeviceType.HOST, id=0):
        self.__robot = robot
        self.__device_id = id
        if device_type == IODeviceType.FLANGE:
            self.__cmds = ['set_flange_do', 'set_flange_ao', 'get_flange_di', 'get_flange_ai']
        elif device_type == IODeviceType.MODBUS_TCP:
            self.__cmds = ['set_external_do', 'set_external_ao', 'get_external_di', 'get_external_ai']
        else:
            self.__cmds = ['set_do', 'set_ao', 'get_di', 'get_ai']

    def set_do(self, pin, value):
        return self.__robot.action(self.__cmds[0], {
            'id': self.__device_id,
            'pin': pin,
            'value': value
        })

    def set_ao(self, pin, value):
        return self.__robot.action(self.__cmds[1], {
            'id': self.__device_id,
            'pin': pin,
            'value': value
        })

    def get_di(self, pin):
        return self.__robot.action(self.__cmds[2], {
            'id': self.__device_id,
            'pin': pin
        })['value']

    def get_ai(self, pin):
        return self.__robot.action(self.__cmds[3], {
            'id': self.__device_id,
            'pin': pin
        })['value']

class LebaiRobot:
    '''
    :param ip: 机器人设备 IP

    :returns: 返回一个乐白机器人控制实例
    '''
    def __init__(self, ip):
        self.ip = ip
        self.host = IODevice(self, IODeviceType.HOST)
        self.flange = IODevice(self, IODeviceType.FLANGE)
        # self.gripper = ClawDevice(self)
        self.externals = {}

    def external(self, id):
        if not id in self.externals:
            self.externals[id] = IODevice(self, IODeviceType.MODBUS_TCP, id)
        return self.externals[id]

    def action(self, cmd, data=None, sleep=0):
        payload = json.dumps({
            'cmd': cmd,
            'data': data
        })
        r = requests.post("http://{0}/public/robot/action".format(self.ip), data=payload)
        r.raise_for_status()
        r = r.json()
        if r['code'] == 0:
            if sleep > 0:
                time.sleep(1)
            return r['data']
        else:
            raise RequestError(r)

    def start_sys(self):
        return self.action('start_sys', sleep=1)

    def stop_sys(self):
        return self.action('stop_sys', sleep=1)

    def powerdown(self):
        return self.action('powerdown', sleep=1)

    def stop(self):
        return self.action('stop', sleep=1)

    def estop(self):
        return self.action('estop', sleep=1)

    def teach_mode(self):
        return self.action('teach_mode', sleep=1)

    def end_teach_mode(self):
        return self.action('end_teach_mode', sleep=1)

    def resume(self):
        return self.action('resume', sleep=1)

    def pause(self):
        return self.action('pause', sleep=1)

    def set_gravity(self, x=0, y=0, z=-9.8):
        if type(x) is tuple or type(x) is list:
            y = x[1]
            z = x[2]
            x = x[0]
        return self.action('set_gravity', {
            'value': [x, y, z]
        }, sleep=1)

    def get_gravity(self):
        r = self.action('get_gravity')['gravity']
        return (r[0], r[1], r[2])

    def set_payload(self, x=0, y=0, z=-9.8, mass=0):
        if type(x) is tuple:
            if type(y) is not tuple and type(x[0]) is tuple:
                y = x[1]
                x = x[0]
            mass = y
            z = x[2]
            y = x[1]
            x = x[0]
        return self.action('set_payload', {
            'mass': mass,
            'cog': [x, y, z]
        }, sleep=1)

    def get_payload(self):
        r = self.action('get_payload')
        return ((r['cog'][0], r['cog'][1], r['cog'][2]), r['mass'])

    def set_tcp(self, x=0, y=0, z=0, rz=0, ry=0, rx=0):
        tcp = CartesianPose(x, y, z, rz, ry, rx)
        return self.action('set_tcp', {
            'value': tcp.pos
        }, sleep=1)

    def get_tcp(self):
        r = self.action('get_tcp')['value']
        return CartesianPose(r)

    def get_velocity_factor(self):
        return self.action('get_velocity_factor')['value']

    def set_velocity_factor(self, factor):
        return self.action('set_velocity_factor', {
            'value': factor
        }, sleep=1)

    def movej(self, p, a=0, v=0, t=0, r=0):
        req = {
            'pose_to': list(p.pos),
            'is_joint_angle': getattr(p, 'is_joint', True),
            'acceleration': a,
            'velocity': v,
            'time': t,
            'smooth_move_to_next':r
        }
        # TODO: base
        return self.action('movej', req)

    def movel(self, p, a=0, v=0, t=0, r=0):
        req = {
            'pose_to': list(p.pos),
            'is_joint_angle': getattr(p, 'is_joint', True),
            'acceleration': a,
            'velocity': v,
            'time': t,
            'smooth_move_to_next':r
        }
        # TODO: base
        return self.action('movel', req)

    def stop_move(self):
        return self.action('stop_move')

    def get_robot_mode(self):
        r = self.action('robot_data')
        return RobotState(r['robot_mode'])

    def get_actual_joint_positions(self):
        res = self.action('robot_data')
        return JointPose(*res['actual_joint'])

    def get_target_joint_positions(self):
        res = self.action('robot_data')
        return JointPose(*res['target_joint'])

    def get_actual_tcp_pose(self):
        res = self.action('robot_data')
        return CartesianPose(*res['actual_tcp_pose'])

    def get_target_tcp_pose(self):
        res = self.action('robot_data')
        return CartesianPose(*res['target_tcp_pose'])

    def __claw_type(self, pin):
        typ = 'Amplitude'
        pin = pin.lower()
        if pin == 'force':
            typ = 'Force'
        elif pin == 'weight':
            typ = 'Weight'
        return typ

    def get_claw_ai(self, pin):
        return self.action('get_claw_ai', {
            'type': self.__claw_type(pin)
        })['value']

    def set_claw_ao(self, pin, value=0):
        return self.action('set_claw_ao', {
            'type': self.__claw_type(pin),
            'value': value
        })
    
    def run_scene(self, scene_id, loop=1):
        task = LebaiScene(self.ip, scene_id=scene_id)
        return task.run(loop)

    def rerun_task(self, task_id):
        task = LebaiScene(self.ip, task_id=task_id)
        return task.run(loop)
