from enum import Enum, unique

class RobotState(Enum):
    '''机器人状态
    '''
    DISCONNECTED = 0  # 已断开连接
    ESTOP = 1         # 急停停止状态
    BOOTING = 2       # 启动中
    ROBOT_OFF = 3     # 电源关闭
    ROBOT_ON = 4      # 电源开启
    IDLE = 5          # 空闲中
    PAUSED = 6        # 暂停中
    RUNNING = 7       # 机器人运动运行中
    UPDATING = 8      # 更新固件中
    STARTING = 9      # 启动中
    STOPPING = 10     # 停止中
    TEACHING = 11     # 示教中
    STOP = 12         # 普通停止
    FINETUNING = 13   # 微调中

class TaskStatus(Enum):
    IDLE = 0 # 空闲
    RUNNING = 1 # 运行中
    PAUSED = 2 # 暂停
    SUCCESS = 3 # 运行完成
    STOPPED = 4 # 停止
    ABORTED = 5 # 异常终止

class IODeviceType(Enum):
    HOST = 0 # 主机箱
    FLANGE = 1 # 法兰
    GRIPPER = 2 # 手爪
    MODBUS_TCP = 3 # MODBUS/TCP
    # MODBUS_TCP6 = 4
    # MODBUS_RTU = 5

class CartesianPose:
    '''空间位置
    
    笛卡尔坐标描述的位姿
    '''
    def __init__(self, x=0, y=0, z=0, rz=0, ry=0, rx=0, base=None):
        if hasattr(x, '__iter__') and len(x) >= 6:
            self.pos = (x[0], x[1], x[2], x[3], x[4], x[5])
        elif type(x) is CartesianPose:
            self.pos = x.pos[:]
            base = x.base
        else:
            self.pos = (x, y, z, rz, ry, rx)
        self.is_joint = False
        # 要求 base 是元组或列表而不是 CartesianPose 对象
        self.base = getattr(base, 'pos', base) if base is not None else None

    def __str__(self):
        if self.base is None:
            return 'CartesianPose' + str(self.pos)
        else:
            return f'CartesianPose({self.pos[0]}, {self.pos[1]}, {self.pos[2]}, {self.pos[3]}, {self.pos[4]}, {self.pos[5]}, base={self.base})'

    def __getattr__(self, key):
        pos_idx_arr = ['x', 'y', 'z', 'rz', 'ry', 'rx']
        if key in pos_idx_arr:
            idx = pos_idx_arr.index(key)
            return getattr(self, 'pos')[idx]
        else:
            return getattr(self, key)

    def __eq__(self, other):
        return self.pos == other.pos and self.base == other.base

class JointPose:
    '''关节位置
    
    关节旋转角度描述的机器人姿态
    '''
    def __init__(self, *j):
        self.pos = j
        self.is_joint = True

    def __str__(self):
        return 'JointPose' + str(self.pos)

class Error(Exception):
    """Base class for exceptions in this module."""
    pass

class RequestError(Error):
    def __init__(self, res):
        print(res)
        self.code = getattr(res, 'code', -1)
        self.param = getattr(res, 'msg_params', [])
        self.data = getattr(res, 'data', None)
