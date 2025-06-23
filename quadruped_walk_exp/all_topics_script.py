#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from datetime import datetime, timezone, timedelta
import csv, os
from std_msgs.msg import Float32, UInt16, UInt8
from unitree_go.msg import LowState, SportModeState, WirelessController

ROOT_SAVE_DIR = os.path.expanduser('~/data/quadruped_walk_2') # '~/data/quadruped_walk'

def timestamp_str():
    now = rclpy.clock.Clock().now().to_msg()
    t = now.sec + now.nanosec * 1e-9
    return datetime.fromtimestamp(t, tz=timezone(timedelta(hours=2))).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

def make_save_folder():
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    folder = os.path.join(ROOT_SAVE_DIR, f'exp_50hz_{ts}') # exp_{ts}, exp_50hz_{ts}
    os.makedirs(folder, exist_ok=True)
    return folder

def unpack_lowstate(msg, prefix):
    result = []
    # head, level_flag, frame_reserve, sn, version, bandwidth
    result += list(msg.head) + [msg.level_flag, msg.frame_reserve] + list(msg.sn) + list(msg.version) + [msg.bandwidth]
    # imu_state
    imu = msg.imu_state
    result += list(imu.quaternion) + list(imu.gyroscope) + list(imu.accelerometer) + list(imu.rpy) + [imu.temperature]
    # 12 motors (0~11)
    for m in msg.motor_state[:12]:
        result += [m.mode, m.q, m.dq, m.ddq, m.tau_est, m.q_raw, m.dq_raw, m.ddq_raw, m.temperature, m.lost] + list(m.reserve)
    # bms_state
    bms = msg.bms_state
    result += [bms.version_high, bms.version_low, bms.status, bms.soc, bms.current, bms.cycle]
    result += list(bms.bq_ntc)[:2] + list(bms.mcu_ntc)[:2] + list(bms.cell_vol)[:15]
    # foot_force, foot_force_est, tick, wireless_remote, bit_flag...
    result += list(msg.foot_force)[:4] + list(msg.foot_force_est)[:4] + [msg.tick]
    result += list(msg.wireless_remote)[:40] + [msg.bit_flag, msg.adc_reel, msg.temperature_ntc1, msg.temperature_ntc2, msg.power_v, msg.power_a]
    result += list(msg.fan_frequency)[:4] + [msg.reserve, msg.crc]
    return result

def unpack_sportmodestate(msg, prefix):
    result = []
    # stamp
    result += [msg.stamp.sec, msg.stamp.nanosec, msg.error_code]
    # imu_state
    imu = msg.imu_state
    result += list(imu.quaternion) + list(imu.gyroscope) + list(imu.accelerometer) + list(imu.rpy) + [imu.temperature]
    # base fields
    result += [msg.mode, msg.progress, msg.gait_type, msg.foot_raise_height]
    result += list(msg.position)[:3] + [msg.body_height]
    result += list(msg.velocity)[:3] + [msg.yaw_speed]
    result += list(msg.range_obstacle)[:4] + list(msg.foot_force)[:4]
    result += list(msg.foot_position_body)[:12] + list(msg.foot_speed_body)[:12]
    return result

class AllTopicsLogger(Node):
    def __init__(self):
        super().__init__('all_topics_logger')
        save_folder = make_save_folder()
        self.csv_path = os.path.join(save_folder, 'all_topics.csv')
        self.file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.file)

        # latest buffers
        self.latest = {
            'wireless': None,
            'low': None,
            'lf_low': None,
            'sport': None,
            'lf_sport': None
        }

        # CSV header
        header = ['timestamp']
        # wirelesscontroller
        header += ['wc_lx', 'wc_ly', 'wc_rx', 'wc_ry', 'wc_keys']
        # lowstate/lf_lowstate
        for prefix in ['low', 'lf_low']:
            header += [f'{prefix}_head0', f'{prefix}_head1', f'{prefix}_level_flag', f'{prefix}_frame_reserve',
                       f'{prefix}_sn0', f'{prefix}_sn1', f'{prefix}_version0', f'{prefix}_version1', f'{prefix}_bandwidth']
            header += [f'{prefix}_imu_quat{i}' for i in range(4)]
            header += [f'{prefix}_imu_gyro{i}' for i in range(3)]
            header += [f'{prefix}_imu_acc{i}' for i in range(3)]
            header += [f'{prefix}_imu_rpy{i}' for i in range(3)]
            header += [f'{prefix}_imu_temp']
            for j in range(12):
                for field in ['mode', 'q', 'dq', 'ddq', 'tau_est', 'q_raw', 'dq_raw', 'ddq_raw', 'temperature', 'lost', 'reserve0', 'reserve1']:
                    header.append(f'{prefix}_m{j}_{field}')
            header += [f'{prefix}_bms_version_high', f'{prefix}_bms_version_low', f'{prefix}_bms_status', f'{prefix}_bms_soc',
                       f'{prefix}_bms_current', f'{prefix}_bms_cycle',
                       f'{prefix}_bms_bq_ntc0', f'{prefix}_bms_bq_ntc1',
                       f'{prefix}_bms_mcu_ntc0', f'{prefix}_bms_mcu_ntc1'] \
                   + [f'{prefix}_bms_cell_vol{i}' for i in range(15)]
            header += [f'{prefix}_foot_force{i}' for i in range(4)]
            header += [f'{prefix}_foot_force_est{i}' for i in range(4)]
            header += [f'{prefix}_tick']
            header += [f'{prefix}_wireless_remote{i}' for i in range(40)]
            header += [f'{prefix}_bit_flag', f'{prefix}_adc_reel', f'{prefix}_temperature_ntc1', f'{prefix}_temperature_ntc2', f'{prefix}_power_v', f'{prefix}_power_a']
            header += [f'{prefix}_fan_frequency{i}' for i in range(4)]
            header += [f'{prefix}_reserve', f'{prefix}_crc']
        # sportmodestate/lf_sportmodestate
        for prefix in ['sport', 'lf_sport']:
            header += [f'{prefix}_stamp_sec', f'{prefix}_stamp_nanosec', f'{prefix}_error_code']
            header += [f'{prefix}_imu_quat{i}' for i in range(4)]
            header += [f'{prefix}_imu_gyro{i}' for i in range(3)]
            header += [f'{prefix}_imu_acc{i}' for i in range(3)]
            header += [f'{prefix}_imu_rpy{i}' for i in range(3)]
            header += [f'{prefix}_imu_temp']
            header += [f'{prefix}_mode', f'{prefix}_progress', f'{prefix}_gait_type', f'{prefix}_foot_raise_height']
            header += [f'{prefix}_pos{i}' for i in range(3)] + [f'{prefix}_body_height']
            header += [f'{prefix}_vel{i}' for i in range(3)] + [f'{prefix}_yaw_speed']
            header += [f'{prefix}_range_obs{i}' for i in range(4)]
            header += [f'{prefix}_foot_force{i}' for i in range(4)]
            header += [f'{prefix}_foot_position_body{i}' for i in range(12)]
            header += [f'{prefix}_foot_speed_body{i}' for i in range(12)]
        self.writer.writerow(header)

        # topic subscription
        self.create_subscription(WirelessController, '/wirelesscontroller', self.cb_wireless, 10)
        self.create_subscription(LowState, '/lowstate', self.cb_lowstate, 10)
        self.create_subscription(LowState, '/lf/lowstate', self.cb_lflow, 10)
        self.create_subscription(SportModeState, '/sportmodestate', self.cb_sport, 10)
        self.create_subscription(SportModeState, '/lf/sportmodestate', self.cb_lfsport, 10)

        self.create_timer(0.02, self.record_row) # set the recording freq #0.05, 0.02

    def cb_wireless(self, msg):
        self.latest['wireless'] = [msg.lx, msg.ly, msg.rx, msg.ry, msg.keys]

    def cb_lowstate(self, msg):
        self.latest['low'] = msg

    def cb_lflow(self, msg):
        self.latest['lf_low'] = msg

    def cb_sport(self, msg):
        self.latest['sport'] = msg

    def cb_lfsport(self, msg):
        self.latest['lf_sport'] = msg

    def record_row(self):
        row = [timestamp_str()]
        # wirelesscontroller
        wc = self.latest['wireless'] or ['']*5
        row += wc
        # lowstate/lf_lowstate
        for key in ['low', 'lf_low']:
            if self.latest[key]:
                row += unpack_lowstate(self.latest[key], key)
            else:
                row += ['']*(
                    2+1+1+2+2+1 +    # head+level_flag+frame_reserve+sn+version+bandwidth
                    4+3+3+3+1 +      # imu
                    12*12 +          # 12 motors * 12 fields
                    6+2+2+15 +       # bms
                    4+4+1+40+1+1+1+1+1+1+4+1+1  # foot_force等
                )
        # sportmodestate/lf_sportmodestate
        for key in ['sport', 'lf_sport']:
            if self.latest[key]:
                row += unpack_sportmodestate(self.latest[key], key)
            else:
                row += ['']*(
                    3 + 4+3+3+3+1 +  # stamp+imu
                    1+1+1+1+3+1+3+1+4+4+12+12   # 剩下全部字段
                )
        self.writer.writerow(row)

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = AllTopicsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    print("Saved CSV at", node.csv_path)

if __name__ == '__main__':
    main()