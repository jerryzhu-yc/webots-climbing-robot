#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
三足履带爬管机器人 —— 控制器主程序

这台机器人有三条履带腿, 像奔驰车标的三叉一样均匀撑在竖直钢管内壁上,
靠履带贴着管壁往上爬. 整个过程分三步:

    第一步 (开机 0~3 秒)  三条腿慢慢张开, 履带贴住管壁
    第二步 (3 秒之后)     机体沿管轴匀速上升, 132 块履带板同时滚动,
                          画面上就是"履带驱动着往上爬"
    第三步 (到达管顶)     停住, 自查一遍关节角度和最终高度, 写进日志

说明: 这是一段"运动学演示" —— 上升由 Webots 的 Supervisor 节点(相当于
导演视角)直接抬动机体完成, 不依赖真实的摩擦力和电机力矩, 所以每次运行
的结果都完全一致, 不会出现爬到一半滑下来的情况.

机器人模型从 URDF 原样导入, 外形没有做任何改动. 每条腿三个关节:
joint3 是主驱动(决定腿撑多开), joint4/joint5 跟随联动(保证履带面
始终竖直贴壁不歪斜), 名字后面的 _1/_2/_3 对应三条腿.
"""

import math
import os
import sys
import time
import traceback

# controller 是 Webots 自带的接口库(不是本项目的文件);
# gait_data.py 存放预先算好的动作数据(关节角度表、履带环几何), 相当于"动作剧本".
from controller import Supervisor
from gait_data import (SEG, S5, S4, Z_START, Z_END, CLIMB_VEL, TRACKS, TRK_NB)

# 三条腿的编号, 后面大量出现"对每条腿做同样的事"的循环.
LEGS = (1, 2, 3)

# 张腿动作的播放时长(秒). 改小张得快, 改大张得慢.
T_INIT = 3.0

# 每条腿 3 个关节的名字. S4/S5 是联动方向系数(三条腿装配朝向不同,
# 所以有 +1 有 -1), 具体数值在 gait_data.py 里.
JN = {i: ('joint3_%d' % i, 'joint4_%d' % i, 'joint5_%d' % i) for i in LEGS}

# 运行日志: 过程记录写进同目录的 controller_log.txt, 方便事后核对爬到了多高.
LOGF = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         'controller_log.txt'), 'w', encoding='utf-8')

def log(msg):
    """往屏幕和日志文件各写一行, 双保险."""
    print(msg, flush=True)
    LOGF.write(msg + '\n')
    LOGF.flush()


class Climber(Supervisor):
    """机器人总控.

    继承 Supervisor 意味着它除了是"机器人", 还兼任"导演":
    可以直接读写场景里任何物体的位置和角度. 本演示正是靠这一点
    抬升机体、摆放履带板的.
    """

    def __init__(self):
        super().__init__()
        self.tstep = int(self.getBasicTimeStep())   # 仿真步长(毫秒), 世界文件里定为 16
        self.dt = self.tstep / 1000.0               # 换算成秒, 算速度用

        # 9 个电机 + 9 个位置传感器各就各位: 电机负责"动", 传感器负责"回报转到了多少".
        self.motors = {}
        self.sensors = {}
        for i in LEGS:
            for name in JN[i]:
                m = self.getDevice(name)
                m.setVelocity(3.0)          # 关节限速 3 rad/s, 够用且安全
                self.motors[name] = m
                s = self.getDevice(name + '_sensor')
                s.enable(self.tstep)
                self.sensors[name] = s

        # 机体的位置和朝向字段. 为什么每帧都要管它? 因为演示里履带与管壁
        # 之间刻意留了 1~2 毫米的视觉间隙(没有物理接触), 撒手不管的话机体
        # 会在重力下东倒西歪. 锁住这两个字段, 机体就始终居中、竖直,
        # 只沿管轴上下移动.
        self.node = self.getFromDef('ROBOT')
        self.tfield = self.node.getField('translation')
        self.rfield = self.node.getField('rotation')

        # 132 块履带板(每条腿 44 块)的"把手". 世界文件里每块板都有名字
        # (BLT_腿号_序号), 这里把它们的位移/旋转字段全部抓在手里,
        # 之后每帧挪一下, 履带就"转"起来了.
        self.blt = {i: [] for i in LEGS}
        for i in LEGS:
            for k in range(TRK_NB):
                node = self.getFromDef('BLT_%d_%d' % (i, k))
                self.blt[i].append((node.getField('translation'),
                                    node.getField('rotation')))

    # ---------- 履带滚动的几何计算 ----------

    @staticmethod
    def _ring_point(tr, s):
        """把"沿履带环走了 s 米"换算成履带板该在的位置和朝向.

        履带环是跑道形: 两段直边 + 上下两个半圆(端轮). 机器人往上爬时,
        贴壁那条直边上的板相对机体往下走, 到底部端轮后翻到内侧回程向上,
        到顶部端轮再翻回外侧 —— 和真实履带的运转一模一样.

        返回的坐标是履带自己的局部坐标: x 朝管壁, y 沿履带长边(竖直),
        alpha 是板绕履带法线的转角.
        """
        h, r = tr['half'], tr['r']      # half=直段半长, r=端轮半径
        l1 = 2.0 * h                    # 走完外侧直边的路程
        l2 = l1 + math.pi * r           # 再加下端轮半圈
        l3 = l2 + 2.0 * h               # 再加内侧直边
        if s < l1:                      # 外侧直边(贴壁): 从上往下
            return r, h - s, math.pi
        if s < l2:                      # 下端轮半圆: 从外侧翻到内侧
            th = (s - l1) / r
            return r * math.cos(th), -h - r * math.sin(th), math.pi - th
        if s < l3:                      # 内侧直边(回程): 从下往上
            u = s - l2
            return -r, -h + u, 0.0
        th = (s - l3) / r               # 上端轮半圆: 从内侧翻回外侧
        return -r * math.cos(th), h + r * math.sin(th), -th

    def track_loop(self, dist):
        """让 132 块履带板各自沿环前移 dist 米(走过一圈自动从头循环).

        传入的 dist 是机体已爬升的高度, 于是贴壁侧的板相对机体下移
        同样的距离 —— 画面上: 机体不动时履带静止, 机体上升时履带
        正好贴着管壁滚动, 速度与爬升速度一致.
        """
        for i in LEGS:
            tr = TRACKS[i]
            p = tr['P']                     # 环的周长
            spacing = p / TRK_NB            # 相邻两块板的间距
            for k, (tf, rf) in enumerate(self.blt[i]):
                s = (k * spacing + dist) % p    # 取余 = 转过一圈从头再来
                x, y, alpha = self._ring_point(tr, s)
                tf.setSFVec3f([x, y, 0.0])
                rf.setSFRotation([0.0, 0.0, 1.0, alpha])

    # ---------- 基础动作 ----------

    def set_leg(self, i, th, de):
        """给第 i 条腿的三个关节下发目标角度.

        th 是主关节角(腿撑多开), de 是微调量(本演示恒为 0). 另两个关节
        按联动系数 S4/S5 跟随 —— 这是平行四边形机构的固定关系: 三个关节
        按这个比例同步动, 履带面才能平移着张开而不倾斜.
        """
        q3, q4, q5 = JN[i]
        self.motors[q3].setPosition(th)
        self.motors[q4].setPosition(S4[i] * th + de)
        self.motors[q5].setPosition(S5[i] * th)

    def hold_upright(self):
        """扶正机体: 回到管子正中央、保持竖直, 高度不变.

        张腿、等待这些"没有在爬"的阶段也要每帧扶一次,
        否则机体一受扰动就会慢慢歪掉.
        """
        z = self.tfield.getSFVec3f()[2]
        self.tfield.setSFVec3f([0.0, 0.0, z])
        self.rfield.setSFRotation([0.0, 0.0, 1.0, 0.0])

    def set_z(self, z):
        """把机体搬到高度 z(米), 同时保持居中、竖直 —— 这就是"爬升"."""
        self.tfield.setSFVec3f([0.0, 0.0, z])
        self.rfield.setSFRotation([0.0, 0.0, 1.0, 0.0])

    def label(self, text):
        """在画面左上角显示一行提示, 方便知道当前进行到哪一步."""
        self.setLabel(0, text, 0.02, 0.02, 0.09, 0x000000, 0, 'Arial')

    def play(self, legs, seg, duration):
        """在 duration 秒内, 把几条腿的关节平滑地过渡到目标角度.

        途经点预先存在 gait_data.py 的 SEG 表里, 这里只负责按时间
        均匀插值播放 —— 相当于按下工业机械臂"示教再现"的播放键.
        """
        paths = [SEG[i][seg] for i in legs]
        n = len(paths[0])
        steps = max(1, int(duration * 1000.0 / self.tstep))
        for k in range(steps + 1):
            f = k / float(steps)
            g = f * (n - 1)
            j = min(int(g), n - 2)
            t = g - j
            for li, i in enumerate(legs):
                p0, p1 = paths[li][j], paths[li][j + 1]
                self.set_leg(i, p0[0] + (p1[0] - p0[0]) * t,
                             p0[1] + (p1[1] - p0[1]) * t)
            self.hold_upright()
            if self.step(self.tstep) == -1:    # 返回 -1 表示仿真被关闭
                return False
        return True

    # ---------- 爬到顶之后的自查 ----------

    def verify(self):
        """核对最终高度到没到、9 个关节的到位误差大不大, 结果写进日志.

        PASS = 一切正常.
        """
        log('[verify] settling ...')
        for _ in range(int(1000 / self.tstep)):   # 先稳定 1 秒, 等关节完全停住
            self.hold_upright()
            if self.step(self.tstep) == -1:
                log('[verify] aborted: step=-1 (sim quitting)')
                return
        z_act = self.tfield.getSFVec3f()[2]
        err_max, parts = 0.0, []
        for i in LEGS:
            th, de = SEG[i]['INIT'][-1]
            want = {JN[i][0]: th, JN[i][1]: S4[i] * th + de, JN[i][2]: S5[i] * th}
            for name in JN[i]:
                e = abs(self.sensors[name].getValue() - want[name])
                err_max = max(err_max, e)
                parts.append('%s %.4f' % (name, e))
        log('[verify] joint err: ' + ' | '.join(parts))
        ok = abs(z_act - Z_END) < 0.005 and err_max < 0.02
        log('[verify] FINAL_Z=%.4f (expect %.4f)  max_joint_err=%.4f  RESULT=%s'
            % (z_act, Z_END, err_max, 'PASS' if ok else 'FAIL'))


def _climb(rob):
    """主流程: 张腿 -> 爬升 -> 到顶自查. 顺着读就是全过程."""
    # 先把履带板按初始相位铺满环, 张腿阶段履带保持静止.
    rob.track_loop(0.0)

    # --- 第一步: 3 秒内三条腿张开, 履带贴住管壁 ---
    rob.label('就位: 三足履带撑住管壁 ...')
    if not rob.play(LEGS, 'INIT', T_INIT):
        return
    qE = {i: SEG[i]['INIT'][-1] for i in LEGS}   # 记住最终撑紧的角度
    log('init done (3 tracks gripping wall), body move up %.2f -> %.2f m'
        % (Z_START, Z_END))

    # --- 第二步: 机体沿管轴匀速上移 ---
    z = Z_START
    last_report = 0.0
    while z < Z_END - 1e-9:
        z = min(z + CLIMB_VEL * rob.dt, Z_END)   # 按速度算出新的高度
        rob.set_z(z)                             # 抬机体
        for i in LEGS:                           # 三条腿锁死在撑紧角度
            rob.set_leg(i, qE[i][0], qE[i][1])
        rob.track_loop(z - Z_START)              # 履带板跟着滚起来
        if z - last_report >= 0.25:              # 每爬 25 厘米记一笔日志
            last_report = z
            log('moving up z=%.3f' % z)
            rob.label('履带贴壁上移中   z = %.2f / %.2f m' % (z, Z_END))
        if rob.step(rob.tstep) == -1:
            return

    # --- 第三步: 到顶 ---
    rob.label('已到管顶  z=%.2f m (上移 %.2f m)' % (z, Z_END - Z_START))
    log('DONE z=%.3f' % z)
    # 存一张到顶时的截图, 就在本文件夹最外层.
    if not os.environ.get('PIPE_CLIMB_AUTOQUIT'):
        try:
            rob.exportImage(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         '..', '..', 'climb_final.png'), 95)
        except Exception:
            pass
    rob.verify()

    # 设置了环境变量 PIPE_CLIMB_AUTOQUIT 时是"无人值守"模式, 跑完自动退出;
    # 正常打开 Webots 观看时, 到顶后保持竖直立在原地.
    if os.environ.get('PIPE_CLIMB_AUTOQUIT'):
        time.sleep(1.5)        # 给日志一点时间写完
        rob.simulationQuit(0)
    else:
        while rob.step(rob.tstep) != -1:
            rob.hold_upright()


def main():
    log('controller started, pid=%d, tstep=%d' % (os.getpid(), 0))
    rob = Climber()
    log('devices ok (9 joints + 3 tracks), tstep=%d' % rob.tstep)
    try:
        _climb(rob)
    except Exception:
        # 万一出错: 把完整错误信息写进日志再退出, 方便排查.
        traceback.print_exc(file=sys.stdout)
        traceback.print_exc(file=LOGF)
        LOGF.flush()
        time.sleep(1.5)
        rob.simulationQuit(1)   # 无人值守模式下崩溃也退出, 不空转


if __name__ == '__main__':
    try:
        main()
    except Exception:
        traceback.print_exc(file=sys.stdout)
        traceback.print_exc(file=LOGF)
        LOGF.flush()
