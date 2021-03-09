import math
import time

from lebai_http import LebaiRobot, CartesianPose, JointPose, LebaiScene

def run():
    rb = LebaiRobot("192.168.3.218")

    rb.start_sys()
    print(rb.get_robot_mode())

    rb.movej(JointPose(0, -1.2, math.pi/6, 0, math.pi/4, 0), 0, 0, 1, 0)

    p2 = rb.get_actual_tcp_pose()
    rb.movel(p2, 0, 0, 1, 0)

    rb.stop_sys()
    print(rb.get_robot_mode())

    kfc = LebaiScene('192.168.3.218', 10001)
    kfc.start()
    time.sleep(1)
    kfc.stop()

    print(rb.run_scene(10001))

if __name__ == '__main__':
    run()
