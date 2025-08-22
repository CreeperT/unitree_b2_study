"""
需求：
    编写键盘控制节点，控制b2运动

    移动按键
        w: 向前移动
        s: 向后移动
        a: 向左转
        d: 向右转
        q: 向前并向左转移动
        e: 向前并向右转移动
        z: 向后并向左转移动
        c: 向后并向右转移动

    按下shift键进入时，机器人进入全向模式（Holonomic mode），允许它进行侧向移动（strafing）
        W: 向前移动
        S: 向后移动
        A: 向左移动
        D: 向右移动
        Q: 向前并向左移动
        E: 向前并向右移动
        Z: 向后并向左移动
        C: 向后并向右移动

    速度调整按键
        r: 增加最大速度和转向速度10%
        t: 减少最大速度和转向速度10%
        f: 仅增加线性速度10%
        g: 仅减少线性速度10%
        v: 仅增加转向速度10%
        b: 仅减少转向速度10%

    运动模式切换
        h: 趴下
        j: 恢复站立
        k: 停下

实现流程：
    1.引入读取【键盘录入的第三方库
    2.编写ros2节点
    3.编写案件映射逻辑

"""

import rclpy
from rclpy.node import Node

import termios
import sys
import tty
import threading
import json

from unitree_api.msg import Request

msg = """
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   q    w    e
   a    x    d
   z    s    c

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   Q    W    E
   A    X    D
   Z    S    C

   
anything else : stop

r/t : increase/decrease max speeds by 10%
f/g : increase/decrease only linear speed by 10%
v/b : increase/decrease only angular speed by 10%

h: stand down
j: recovery stand
k: stop move

CTRL-C to quit

"""

ROBOT_SPORT_API_IDS = {
    "DAMP": 1001,
    "BALANCESTAND": 1002,
    "STOPMOVE": 1003,
    "STANDUP": 1004,
    "STANDDOWN": 1005,
    "RECOVERYSTAND": 1006,
    "MOVE": 1008,
    "SWITCHGAIT": 1011    
}

sportModel = {
    "h": ROBOT_SPORT_API_IDS["STANDDOWN"],
    "j": ROBOT_SPORT_API_IDS["RECOVERYSTAND"],
    "k": ROBOT_SPORT_API_IDS["STOPMOVE"]
}

moveBindings = {
    "w": (1, 0, 0, 0),  # x*1, y*0, z*0, omega*0
    "s": (-1, 0, 0, 0),
    "a": (0, 0, 0, 1),
    "d": (0, 0, 0, -1),
    "q": (1, 0, 0, 1),
    "e": (1, 0, 0, -1),
    "z": (-1, 0, 0, 1),
    "c": (-1, 0, 0, -1),
    "W": (1, 0, 0, 0),
    "S": (-1, 0, 0, 0),
    "A": (0, 1, 0, 0),
    "D": (0, -1, 0, 0),
    "Q": (1, 1, 0, 0),
    "E": (1, -1, 0, 0),
    "Z": (-1, 1, 0, 0),
    "C": (-1, -1, 0, 0)
}

speedBindings = {
    "r": (1.1, 1.1),
    "t": (0.9, 0.9),
    "f": (1.1, 1.0),
    "g": (0.9, 1.0),
    "v": (1.0, 1.1),
    "b": (1.0, 0.9)
}

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_ctrl_keyboard')
        self.pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.declare_parameter("speed", 0.2)
        self.declare_parameter("angular", 0.5)
        self.speed = self.get_parameter("speed").value
        self.angular = self.get_parameter("angular").value

    def publish(self, api_id, x=0.0, y=0.0, omega=0.0):
        req = Request()
        req.header.identity.api_id = api_id
        js = {"x": x, "y": y, "z": omega}
        req.parameter = json.dumps(js)
        self.pub.publish(req)

# 读取按键
def getKey(settings):
    # 读取按键模式
    tty.setraw(sys.stdin.fileno())
    # 获取一个按键
    key = sys.stdin.read(1)
    # 恢复终端原始设置
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    # 返回按键值
    return key

def main():
    print(msg)
    # 读取键盘录入实现
    ## 1.获取标准输入流终端属性并返回
    settings = termios.tcgetattr(sys.stdin)

    ## 2.ros2 node实现，需要单独子线程处理
    rclpy.init()
    teleopNode = TeleopNode()
    spinner = threading.Thread(target=rclpy.spin, args=(teleopNode, ))
    spinner.start()

    ## 3.循环读取按键并映射
    try:
        while True:
            key = getKey(settings)
            # 1.结束终端判断 ctrl+c 狗子进入站立平衡状态
            if key == '\x03':
                break

            # 2.运动模式切换（设置api_id)
            elif key in sportModel.keys():
                teleopNode.publish(sportModel[key])

            # 3.运动控制（设置api_id和速度消息）
            elif key in moveBindings.keys():
                x_bind = moveBindings[key][0]
                y_bind = moveBindings[key][1]
                omega_bind = moveBindings[key][3]
                teleopNode.publish(ROBOT_SPORT_API_IDS["MOVE"],
                                   x = x_bind * teleopNode.speed,
                                   y = y_bind * teleopNode.speed,
                                   omega = omega_bind * teleopNode.angular)

            # 4.运动控制（设置api_id和速度消息）
            elif key in speedBindings.keys():
                s_bind = speedBindings[key][0]
                a_bind = speedBindings[key][1]
                teleopNode.speed = s_bind * teleopNode.speed
                teleopNode.angular = a_bind * teleopNode.angular
                print("current speed: %.5f, angular: %.5f" %(teleopNode.speed, teleopNode.angular))

            # 5.所有按键都不在这里面
            else:
                teleopNode.publish(ROBOT_SPORT_API_IDS["BALANCESTAND"])


    finally:        
        # 狗子进入站立平衡状态
        teleopNode.publish(ROBOT_SPORT_API_IDS["BALANCESTAND"])
        rclpy.shutdown()


    

if __name__ == '__main__':
    main()
