#!/usr/bin/env python3
"""AutoRace race manager: environment control + judge, on simulation time.

Drives the mission devices (light cycle, intersection sign, barrier) and judges the run
from environment truth regions. Nothing here is meant as robot input: the robot must use
its own camera / LiDAR.

  sub  /autorace/truth/{start_line,branch_left,branch_right,s1,s2}  std_msgs/Bool
  sub  /autorace/devices/joint_states                              sensor_msgs/JointState
  pub  /autorace/devices/{light,direction,barrier}                 std_msgs/String
  pub  /autorace/race/state   std_msgs/String (JSON, 5 Hz)
  pub  /autorace/race/events  std_msgs/String (JSON per event; also written to log_dir/*.jsonl)
  srv  /autorace/race/reset   std_srvs/Trigger
  srv  /autorace/race/barrier_open  std_srvs/Trigger  (manual opening policy)
"""
import json
import os
import random
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger

REGIONS = ('start_line', 'branch_left', 'branch_right', 's1', 's2')
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class RaceManager(Node):

    def __init__(self):
        super().__init__('race_manager')
        p = self.declare_parameter
        self.cfg = {
            'red': p('light_red_duration_s', -1.0).value,
            'yellow': p('light_yellow_duration_s', -1.0).value,
            'green': p('light_green_duration_s', 5.0).value,
            'initial_delay': p('light_initial_delay_s', 0.0).value,
            'seed': p('intersection_seed', -1).value,
            'reseed': p('intersection_reseed_on_reset', False).value,
            'p_left': p('intersection_probability_left', -1.0).value,
            'select_delay': p('intersection_selection_delay_s', -1.0).value,
            'open_policy': p('barrier_open_policy', '').value,
            'open_delay': p('barrier_open_delay_s', -1.0).value,
            'lowered_thr': p('barrier_lowered_threshold_rad', -1.0).value,
            's2_closing': p('barrier_s2_fails_while_closing', True).value,
            'start_on': p('start_line_judge_on', 'enter').value,
            'reset_world': p('reset_world_on_reset', True).value,
            'log_dir': os.path.expanduser(p('log_dir', '~/.ros/autorace_logs').value),
        }
        unset = [k for k in ('red', 'yellow', 'p_left', 'select_delay', 'lowered_thr') if self.cfg[k] < 0]
        if self.cfg['open_policy'] not in ('delay_after_lowered', 'manual'):
            unset.append('open_policy')
        elif self.cfg['open_policy'] == 'delay_after_lowered' and self.cfg['open_delay'] < 0:
            unset.append('open_delay')
        if unset:
            raise SystemExit(f'race_manager: unset runtime settings {unset}; see config/race_runtime.yaml')
        if self.cfg['seed'] < 0:
            self.cfg['seed'] = int(time.time_ns() % 2**31)

        self.pub = {k: self.create_publisher(String, f'/autorace/devices/{k}', 10)
                    for k in ('light', 'direction', 'barrier')}
        self.state_pub = self.create_publisher(String, '/autorace/race/state', LATCHED)
        self.event_pub = self.create_publisher(String, '/autorace/race/events', 50)
        for r in REGIONS:
            self.create_subscription(Bool, f'/autorace/truth/{r}',
                                     lambda m, r=r: self.on_region(r, m.data), LATCHED)
        self.create_subscription(JointState, '/autorace/devices/joint_states', self.on_joints, 10)
        self.create_service(Trigger, '/autorace/race/reset', self.srv_reset)
        self.create_service(Trigger, '/autorace/race/barrier_open', self.srv_open)
        self.reset_world_cli = self.create_client(Empty, '/reset_world')

        os.makedirs(self.cfg['log_dir'], exist_ok=True)
        self.log_path = os.path.join(self.cfg['log_dir'], datetime.now().strftime('race_%Y%m%d_%H%M%S.jsonl'))
        self.log_file = open(self.log_path, 'a')

        self.run = 0
        self.seed = self.cfg['seed']
        self.region = {r: False for r in REGIONS}
        self.started = False
        self.create_timer(0.01, self.tick)
        self.create_timer(0.2, self.publish_state)
        self.get_logger().info(f'race manager ready, log {self.log_path}')

    # ------------------------------------------------------------ run state

    def begin_run(self):
        self.run += 1
        if self.run > 1 and self.cfg['reseed']:
            self.seed = int(time.time_ns() % 2**31)
        self.rng = random.Random(self.seed)
        self.epoch = self.now()
        self.light = None
        self.first_green_t = None          # race timer start, latched once per run
        self.direction = None
        self.start_crossed = False
        self.branch_taken = None
        self.barrier = 'open'              # open | closing | lowered | opening
        self.barrier_target = 'open'
        self.barrier_q = None
        self.lowered_t = None
        self.s1_latched = False
        self.results = {'start': None, 'intersection': None, 'barrier': None}
        self.failures = []
        self.send('direction', 'right')    # default state: red / right upright / bar open
        self.send('barrier', 'open')
        self.event('run_start', run=self.run, seed=self.seed, config=self.cfg)
        self.started = True

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def t(self):
        return self.now() - self.epoch

    def race_time(self):
        return None if self.first_green_t is None else self.t() - self.first_green_t

    def send(self, dev, value):
        self.pub[dev].publish(String(data=value))

    def event(self, name, **kw):
        rec = {'event': name, 'run': self.run, 'sim_t': round(self.now(), 4)}
        if self.started or name == 'run_start':
            rec['t'] = round(self.t(), 4)
            rt = self.race_time()
            rec['race_t'] = None if rt is None else round(rt, 4)
        rec.update(kw)
        line = json.dumps(rec, ensure_ascii=False)
        self.event_pub.publish(String(data=line))
        self.log_file.write(line + '\n')
        self.log_file.flush()
        self.get_logger().info(line)

    def fail(self, mission, reason, **kw):
        if self.results.get(mission) is None:
            self.results[mission] = 'fail'
        self.failures.append(reason)
        self.event('fail', mission=mission, reason=reason, **kw)

    # ------------------------------------------------------------ main loop

    def tick(self):
        if not self.started:
            # Wait for /clock so the epoch is simulation time.
            if self.now() > 0:
                self.begin_run()
            return
        t = self.t()
        self.update_light(t)
        if (self.direction is None and self.first_green_t is not None
                and t - self.first_green_t >= self.cfg['select_delay']):
            self.direction = 'left' if self.rng.random() < self.cfg['p_left'] else 'right'
            self.send('direction', self.direction)
            self.event('intersection_selected', direction=self.direction, seed=self.seed)
        if (self.barrier == 'lowered' and self.cfg['open_policy'] == 'delay_after_lowered'
                and t - self.lowered_t >= self.cfg['open_delay']):
            self.command_barrier('open', 'policy_delay')
        self.check_s2()

    def update_light(self, t):
        r, y, g = self.cfg['red'], self.cfg['yellow'], self.cfg['green']
        t0 = t - self.cfg['initial_delay']
        if t0 < 0:
            phase = 'red'
        else:
            k = t0 % (r + y + g)
            phase = 'red' if k < r else 'yellow' if k < r + y else 'green'
        if phase != self.light:
            self.light = phase
            self.send('light', phase)
            self.event('light', color=phase)
            if phase == 'green' and self.first_green_t is None:
                self.first_green_t = t
                self.event('race_timer_start')

    # ------------------------------------------------------------ inputs

    def on_region(self, name, active):
        prev = self.region[name]
        self.region[name] = active
        if not self.started or prev == active:
            return
        self.event('region', region=name, active=active)
        if name == 'start_line' and not self.start_crossed:
            if (active and self.cfg['start_on'] == 'enter') or (not active and self.cfg['start_on'] == 'exit'):
                self.start_crossed = True
                if self.light == 'green':
                    self.results['start'] = 'pass'
                    self.event('start_ok', light=self.light)
                else:
                    self.fail('start', 'start_line_crossed_on_' + str(self.light), light=self.light)
        elif name in ('branch_left', 'branch_right') and active and self.branch_taken is None:
            self.branch_taken = name.split('_')[1]
            if self.direction is None:
                self.fail('intersection', 'branch_before_selection', branch=self.branch_taken)
            elif self.branch_taken == self.direction:
                self.results['intersection'] = 'pass'
                self.event('intersection_ok', branch=self.branch_taken)
            else:
                self.fail('intersection', 'wrong_branch', branch=self.branch_taken, shown=self.direction)
        elif name == 's1' and active and not self.s1_latched and self.barrier == 'open':
            # S1 requests lowering once per run [RULE]; S2 never opens the bar [RULE].
            self.s1_latched = True
            self.command_barrier('close', 's1')
        elif name == 's2' and not active and self.s1_latched and self.barrier in ('open', 'opening') \
                and self.results['barrier'] is None:
            self.results['barrier'] = 'pass'
            self.event('barrier_ok')

    def on_joints(self, msg):
        if not self.started or 'barrier_joint' not in msg.name:
            return
        q = msg.position[msg.name.index('barrier_joint')]
        self.barrier_q = q
        thr = self.cfg['lowered_thr']
        if self.barrier == 'closing' and q <= thr:
            self.barrier = 'lowered'
            self.lowered_t = self.t()
            self.event('barrier_lowered', q=round(q, 4))
        elif self.barrier == 'opening' and q >= 1.5707963 - thr:
            self.barrier = 'open'
            self.event('barrier_opened', q=round(q, 4))
        self.check_s2()

    def check_s2(self):
        if not self.region['s2'] or self.results['barrier'] == 'fail':
            return
        if self.barrier == 'lowered' or (self.barrier == 'closing' and self.cfg['s2_closing']):
            self.fail('barrier', 's2_while_' + self.barrier)

    def command_barrier(self, cmd, why):
        self.barrier = 'closing' if cmd == 'close' else 'opening'
        self.send('barrier', cmd)
        self.event('barrier_command', command=cmd, cause=why)

    # ------------------------------------------------------------ services / output

    def srv_open(self, _req, res):
        if self.barrier in ('lowered', 'closing'):
            self.command_barrier('open', 'manual')
            res.success = True
        else:
            res.message = f'barrier is {self.barrier}'
        return res

    def srv_reset(self, _req, res):
        self.event('reset_requested')
        if self.cfg['reset_world'] and self.reset_world_cli.service_is_ready():
            self.reset_world_cli.call_async(Empty.Request())
        self.started = False   # begin_run on next tick, after the world reset lands
        self.region = {r: False for r in REGIONS}
        res.success = True
        res.message = f'run {self.run + 1} starting'
        return res

    def publish_state(self):
        if not self.started:
            return
        rt = self.race_time()
        st = {
            'run': self.run, 'seed': self.seed, 't': round(self.t(), 3),
            'race_t': None if rt is None else round(rt, 3),
            'light': self.light, 'direction': self.direction,
            'barrier': self.barrier, 'barrier_q': None if self.barrier_q is None else round(self.barrier_q, 3),
            'regions': self.region, 'results': self.results, 'failures': self.failures,
        }
        self.state_pub.publish(String(data=json.dumps(st)))


def main():
    rclpy.init()
    node = RaceManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.log_file.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
