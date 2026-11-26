#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Hunter Online Phase 1/2 - standalone PyQt5 ROS1 Melodic device and bag GUI.

Does not import rospy (the ROS Melodic environment is Python 2).  The GUI itself
uses Python 3 and manages ROS CLI subprocesses in owned process groups.
This first version intentionally does not start NDT/autonomous navigation.
"""
import datetime
import os
import re
import select
import shutil
import signal
import socket
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from xmlrpc.client import ServerProxy

from PyQt5.QtCore import Qt, QTimer, QSettings
from PyQt5.QtWidgets import (QApplication, QCheckBox, QFileDialog, QFormLayout,
    QGridLayout, QGroupBox, QHBoxLayout, QInputDialog, QLabel, QLineEdit,
    QMainWindow, QMessageBox, QPushButton, QScrollArea, QSpinBox, QTabWidget,
    QTextEdit, QVBoxLayout, QWidget)

HOME = os.path.expanduser('~')
BUNDLE = os.path.dirname(os.path.abspath(__file__))
SENSOR_TOPICS = ['/velodyne_points', '/hunter_status', '/odom', '/tf', '/tf_static', '/cmd_vel', '/points_raw', '/filtered_points', '/ndt_pose', '/ndt_stat', '/current_pose', '/hunter_nav/route', '/hunter_nav/safety_state', '/hunter_nav/applied_cmd', '/offline_nav/twist_cmd']
HUNTER_SETUP = os.path.join(HOME, 'hunter_ws', 'devel', 'setup.bash')
GUI_VERSION = '1.6.10'


def ros_cmd(args, hunter=False):
    """Source the same ROS + Hunter overlay for *all* ROS commands.

    Hunter's custom message types (hunter_msgs/HunterStatus) must be visible not
    only to roslaunch, but also to rostopic/rosmsg/rosbag and Python2 helpers.
    Without the overlay, `rostopic echo /hunter_status` cannot deserialize.
    """
    import shlex
    script = 'set -e; source /opt/ros/melodic/setup.bash; '
    if hunter:
        script += ('test -f "$HOME/hunter_ws/devel/setup.bash" || '
                   '{ echo "找不到 ~/hunter_ws/devel/setup.bash" >&2; exit 2; }; ')
    autoware = os.environ.get('AUTOWARE_LIVE_WS', os.path.join(HOME, 'autoware.ai'))
    overlay = os.path.join(autoware, 'install', 'setup.bash')
    script += 'if [ -f ' + shlex.quote(overlay) + ' ]; then source ' + shlex.quote(overlay) + '; fi; '
    script += ('if [ -f "$HOME/hunter_ws/devel/setup.bash" ]; then '
               'source "$HOME/hunter_ws/devel/setup.bash"; fi; ')
    script += 'exec ' + ' '.join(shlex.quote(str(a)) for a in args)
    return ['bash', '-lc', script]


def run_cmd(args, timeout=5, ros=False):
    try:
        cmd = ros_cmd(args) if ros else args
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           universal_newlines=True, timeout=timeout)
        return p.returncode, p.stdout.strip()
    except (OSError, subprocess.TimeoutExpired) as e:
        return -1, str(e)


def master_ok():
    try:
        old = socket.getdefaulttimeout()
        socket.setdefaulttimeout(0.7)
        try:
            uri = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
            result = ServerProxy(uri, allow_none=True).getPid('/hunter_online_gui')
            return result[0] == 1
        finally:
            socket.setdefaulttimeout(old)
    except Exception:
        return False


def parse_hunter_auto_preflight(returncode, output):
    """Interpret repeated actual /hunter_status reports, not GUI button state.

    Only a stable tail of CAN-command mode (1) is accepted. A former motion
    test is NOT sufficient to establish the current control mode, especially
    after an operator switches back to remote control.
    """
    if returncode != 0:
        return {'ok': False, 'mode': None, 'sample_count': 0,
                'message': '無法讀取 /hunter_status 即時資料；請檢查 Hunter Driver、CAN RX 與 ROS 訊息。'}
    modes = [int(x) for x in re.findall(r'^\s*control_mode:\s*(\d+)\s*$', output, re.M)]
    if not modes:
        return {'ok': False, 'mode': None, 'sample_count': 0,
                'message': '/hunter_status 沒有 control_mode 欄位；無法確認底盤控制模式。'}
    mode = modes[-1]
    tail = modes[-min(5,len(modes)):]
    stable_auto = all(value == 1 for value in tail)
    if stable_auto:
        return {'ok': True, 'mode': 1, 'sample_count': len(modes),
                'message': 'Hunter 回報 CAN 指令模式 1；最近 {} 筆一致。'.format(len(tail))}
    return {'ok': False, 'mode': mode, 'sample_count': len(modes),
            'message': ('Hunter 最近 {} 筆模式回報：{}（最後={}).\n'
                        '這是 Driver 的 CAN 控制模式回報，並非 GUI 判定遙控器開關位置。'
                        '即使先前的 1 秒前進測試成功，也不能保證目前仍在 CAN 指令模式。'
                        '將自動比對 can0 的原始 0x151 回饋，不會發送任何移動命令。'
                        ).format(len(modes), ','.join(str(x) for x in modes[-10:]), mode)}


def parse_hunter_can_mode_samples(output):
    """Parse both modern candump -L ('151#...') and classic candump formats.

    Hunter AGX_V1 status frame 0x151: byte[1] is control_mode.
    This is read-only and never sends CAN traffic.
    """
    values=[]
    for line in output.splitlines():
        m=re.search(r'\b151#([0-9A-Fa-f]{4,16})\b', line)
        if m:
            values.append(int(m.group(1)[2:4], 16))
            continue
        m=re.search(r'\b151\s+\[8\]\s+([0-9A-Fa-f]{2})\s+([0-9A-Fa-f]{2})', line)
        if m:
            values.append(int(m.group(2),16))
    return values


def check_hunter_mode_report():
    """Passive multi-sample mode check with independent CAN frame diagnosis."""
    rc, out=run_cmd(['rostopic','echo','-n','12','/hunter_status'],timeout=7,ros=True)
    result=parse_hunter_auto_preflight(rc,out)
    if result.get('ok') and result.get('sample_count',0)<5:
        result={'ok':False, 'mode':result.get('mode'), 'sample_count':result.get('sample_count',0),
                'message':'Hunter 有回報 AUTO=1，但有效樣本少於 5 筆；先確認資料連續性。'}
    # Report raw bus mode even when the ROS status is suspicious; use a passive
    # candump, not a motion pulse or a remote-control command.
    rc_can, raw = run_cmd(['candump','-L','-n','12','can0,151:7FF'],timeout=4)
    raw_modes=parse_hunter_can_mode_samples(raw if rc_can==0 else '')
    result['raw_can_modes']=raw_modes
    if raw_modes:
        uniq=sorted(set(raw_modes))
        suffix='\nCAN 0x151 原始 mode(byte1)：{}（{}筆）。'.format(','.join(map(str,uniq)),len(raw_modes))
        if result.get('mode') is not None and uniq==[result['mode']]:
            suffix+=' ROS 與 CAN 回報一致。'
        elif result.get('mode') is not None:
            suffix+=' ROS 與 CAN 回報不一致，應檢查 Driver／訊息映射及取樣時間。'
        # Never authorize hardware when raw mode and ROS status disagree, nor
        # when mode switched to remote after status was sampled.
        if result.get('ok') and any(value != 1 for value in raw_modes[-min(5,len(raw_modes)) :]):
            result['ok']=False
            suffix+=' CAN 原始回報不是穩定指令模式；啟動已取消。'
        result['message']+=suffix
    else:
        result['message']+='\n暫無 CAN 0x151 原始取樣（確認 can-utils、can0 和訊息週期）。'
    return result


def status_snapshot():
    result = {'master': master_ok(), 'can': '無介面', 'rx': 0, 'topics': set(), 'nodes': set(), 'error': ''}
    rc, out = run_cmd(['ip', '-s', '-details', 'link', 'show', 'can0'], timeout=3)
    if rc == 0:
        m = re.search(r'can state\s+(\S+)', out)
        state = m.group(1) if m else '?'
        is_up = 'state UP' in out.splitlines()[0] or '<NOARP,UP,' in out.splitlines()[0]
        br = re.search(r'bitrate\s+(\d+)', out)
        result['can'] = ('UP' if is_up else 'DOWN') + ' / ' + state + (' / '+br.group(1)+'bps' if br else '')
        m = re.search(r'RX:\s*bytes\s+packets[^\n]*\n\s*(\d+)\s+(\d+)', out)
        if m:
            result['rx'] = int(m.group(2))
    if result['master']:
        rc, out = run_cmd(['rostopic', 'list'], timeout=5, ros=True)
        if rc == 0:
            result['topics'] = set(out.splitlines())
        else:
            result['error'] = out[-180:]
        rc, out = run_cmd(['rosnode', 'list'], timeout=5, ros=True)
        if rc == 0:
            result['nodes'] = set(out.splitlines())
    return result


def test_udp(port):
    """Only bind UDP before the velodyne driver is running."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.settimeout(3.5)
        sock.bind(('0.0.0.0', port))
        data, remote = sock.recvfrom(4096)
        return '收到 UDP {} Bytes，來源 {}:{}'.format(len(data), remote[0], remote[1])
    except OSError as exc:
        return '未收到／無法綁定 UDP {}：{}'.format(port, exc)
    finally:
        sock.close()


def topic_rate(topic):
    # `timeout` returns 124 normally; still preserves the informative ros output.
    rc, out = run_cmd(['timeout', '5s', 'rostopic', 'hz', topic], timeout=7, ros=True)
    found = re.findall(r'average rate:\s*([\d.]+)', out)
    if found:
        return '{}：{} Hz'.format(topic, found[-1])
    return '{}：未取得頻率\n{}'.format(topic, out[-450:])


def capture_stdout_lines(proc, max_bytes=15000):
    result = []
    if proc.stdout is None:
        return result
    fd = proc.stdout.fileno()
    for _ in range(5):
        ready = select.select([fd], [], [], 0)[0]
        if not ready:
            break
        try:
            data = os.read(fd, max_bytes)
        except (OSError, BlockingIOError):
            break
        if not data:
            break
        result.append(data.decode('utf-8', 'replace'))
    return result


class OnlineGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('HUNTER / AUTOWARE-LITE ONLINE — Phase 1+2')
        self.resize(1210, 850)
        self.proc = {}
        self.hunter_protocol_model=None
        self.hunter_protocol_tail=''
        self.settings = QSettings('NTUT-Robotics', 'HunterOnlinePhase12')
        self.pool = ThreadPoolExecutor(max_workers=3)
        self.pending = []
        self.snapshot = {}
        self.last_snapshot = 0
        self.motion_active = False
        self.ui()
        self.apply_font_size(self.font_spin.value())
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(260)
        self.log('Phase 1+2 v{} 就緒。所有 ROS 子程序均載入 Hunter overlay；不會自動讓車輛移動。'.format(GUI_VERSION))
        if not os.path.isfile(HUNTER_SETUP):
            self.log('警告：找不到 {}，自訂 Hunter ROS message 無法解析。'.format(HUNTER_SETUP))
        self.schedule_status(force=True)

    def apply_font_size(self, size):
        # Scale the full Qt UI and persist the setting for the next run.
        size = int(size)
        css = (self.ui_style_template
               .replace('__BASE__', str(size))
               .replace('__TITLE__', str(size + 10))
               .replace('__LOG__', str(max(11, size - 2))))
        self.setStyleSheet(css)
        self.settings.setValue('ui_font_px', size)

    def scroll_page(self, widget):
        area = QScrollArea()
        area.setWidgetResizable(True)
        area.setFrameShape(QScrollArea.NoFrame)
        area.setWidget(widget)
        return area

    def group(self, title):
        box = QGroupBox(title); box.setLayout(QVBoxLayout()); return box

    def btn(self, title, callback, danger=False):
        b = QPushButton(title)
        if danger: b.setObjectName('danger')
        b.clicked.connect(callback)
        return b

    def page_bag(self):
        w=QWidget(); l=QVBoxLayout(w)
        g=self.group('錄製 ROS Topics（本階段可用）'); l.addWidget(g)
        form=QFormLayout(); g.layout().addLayout(form)
        self.bag_folder=QLineEdit(os.path.join(HOME,'hunter_online_gui','bags'))
        row=QHBoxLayout(); row.addWidget(self.bag_folder)
        row.addWidget(self.btn('瀏覽',self.pick_bag_folder))
        form.addRow('儲存資料夾',row)
        self.bag_prefix=QLineEdit('hunter_live'); form.addRow('檔名前綴',self.bag_prefix)
        self.topic_check={}
        for topic in SENSOR_TOPICS:
            chk=QCheckBox(topic); chk.setChecked(topic in ['/velodyne_points','/hunter_status','/odom','/tf','/tf_static','/cmd_vel'])
            self.topic_check[topic]=chk; g.layout().addWidget(chk)
        self.bag_info=QLabel('尚未錄製。ROS Bag 也能錄 Hunter /odom、/hunter_status，不只有點雲。')
        g.layout().addWidget(self.bag_info)
        row=QHBoxLayout(); g.layout().addLayout(row)
        row.addWidget(self.btn('開始錄製 Bag', self.start_bag))
        row.addWidget(self.btn('停止並完成 Bag', lambda: self.stop('bag')))
        row.addWidget(self.btn('開啟 Bag 資料夾', self.open_bag_folder))
        g.layout().addWidget(QLabel('預設每 1024 MB 分檔。寫入中的 .active 檔必須正常停止後才能完整封存。'))
        l.addStretch(); return self.scroll_page(w)

    def log(self, txt):
        s='[{}] {}'.format(datetime.datetime.now().strftime('%H:%M:%S'), txt)
        self.logs.append(s)

    def async_task(self, label, work, update=None):
        future = self.pool.submit(work)
        self.pending.append((label, future, update))

    def schedule_status(self, force=False):
        if self.pending and any(n=='status' for n,_,_ in self.pending): return
        if not force and time.monotonic()-self.last_snapshot<5: return
        self.last_snapshot=time.monotonic()
        self.async_task('status', status_snapshot, self.show_snapshot)

    def show_snapshot(self, x):
        self.snapshot=x
        t=x['topics']; can=x['can']
        self.master_label.setText('ROS Master：{}'.format('已連線' if x['master'] else '未啟動'))
        self.can_label.setText('CAN：{}，RX={} frames'.format(can,x['rx']))
        self.can_details.setText('can0：{}，RX={} frames'.format(can,x['rx']))
        self.hunter_label.setText('Hunter：{}  |  /hunter_status：{}'.format('Driver 執行中' if self.is_running('hunter') else '由 ROS 狀態判斷', '存在' if '/hunter_status' in t else '未偵測到'))
        self.lidar_label.setText('LiDAR：/velodyne_points {}'.format('存在' if '/velodyne_points' in t else '未偵測到'))
        self.odom_label.setText('Odometry：/odom {}'.format('存在' if '/odom' in t else '未偵測到'))
        self.global_status.setText('ROS：{}  |  CAN：{} RX={}  |  Hunter：{}  |  LiDAR：{}'.format(
            '連線' if x['master'] else '斷線',can,x['rx'],
            '可見' if '/hunter_status' in t else '未連線',
            '可見' if '/velodyne_points' in t else '未連線'))

    def tick(self):
        for label, fut, callback in list(self.pending):
            if fut.done():
                self.pending.remove((label,fut,callback))
                try:
                    val=fut.result()
                    if callback: callback(val)
                    elif isinstance(val,str): self.log('{}：{}'.format(label,val))
                except Exception as e:
                    self.log('{} 發生錯誤：{}'.format(label,e))
        for role, proc in list(self.proc.items()):
            for data in capture_stdout_lines(proc):
                if role=='hunter':
                    # Parse full driver stdout (including chunk boundaries)
                    # without depending on the truncated GUI display.
                    merged=self.hunter_protocol_tail+data
                    matches=re.findall(r'Detected protocol:\s*AGX_V([12])',merged)
                    if matches:self.hunter_protocol_model='hunter'+matches[-1]
                    self.hunter_protocol_tail=merged[-96:]
                lines=[ln for ln in data.replace('\r','\n').splitlines() if ln.strip()]
                for ln in lines[:8]:
                    self.log('[{}] {}'.format(role,ln[:320]))
            if proc.poll() is not None:
                self.log('{} 程序結束 (exit={})'.format(role,proc.returncode))
                try: proc.stdout.close()
                except Exception: pass
                self.proc.pop(role,None)
                if role=='bag':self.bag_info.setText('Bag 已停止。請到資料夾確認 .bag 檔案。')
                if role=='motion':self.motion_active=False
        self.schedule_status()

    def is_running(self,role):
        return role in self.proc and self.proc[role].poll() is None

    def start_proc(self,role,args,ros=True,hunter=False,stdin_data=None):
        if self.is_running(role):
            self.log('{} 已在執行，避免重複啟動。'.format(role)); return False
        cmd=ros_cmd(args,hunter=hunter) if ros else args
        try:
            proc=subprocess.Popen(cmd,stdin=subprocess.PIPE if stdin_data else subprocess.DEVNULL,
                stdout=subprocess.PIPE,stderr=subprocess.STDOUT,start_new_session=True,bufsize=0)
            os.set_blocking(proc.stdout.fileno(),False)
            self.proc[role]=proc
            if role=='hunter':
                self.hunter_protocol_model=None
                self.hunter_protocol_tail=''
            if stdin_data:
                proc.stdin.write(stdin_data); proc.stdin.flush()
                if role != 'motion': proc.stdin.close()
            self.log('啟動 {}：{}'.format(role,' '.join(args)))
            return True
        except OSError as exc:
            self.log('{} 無法啟動：{}'.format(role,exc)); return False

    def stop(self,role):
        proc=self.proc.get(role)
        if proc is None or proc.poll() is not None:
            self.log('{} 不是本 GUI 啟動的執行程序。'.format(role));return
        try:
            os.killpg(os.getpgid(proc.pid),signal.SIGINT)
            self.log('要求停止 {}（SIGINT）'.format(role))
        except OSError as exc:self.log(str(exc))

    def stop_all(self):
        for role in ['motion','zero','bag','rviz','lidar','hunter','core']:
            if self.is_running(role): self.stop(role)

    def start_master(self):
        if master_ok():self.log('重用既有 ROS Master，不會關閉外部 roscore。');return
        self.start_proc('core',['roscore'],ros=True)

    def can_up(self):
        if 'UP' in self.snapshot.get('can','').split(' / ')[0]:
            self.log('can0 已是 UP；不需要重設。');return
        if not os.path.exists('/sys/class/net/can0'):
            self.log('找不到 can0，請先在 Windows Bridge 執行 usbipd attach。');return
        args=['ip','link','set','can0','up','type','can','bitrate','500000']
        rc,out=run_cmd(['sudo','-n']+args,timeout=5)
        if rc!=0:
            pw,ok=QInputDialog.getText(self,'啟用 can0','僅本次執行 sudo ip link；請輸入 Ubuntu 密碼：',QLineEdit.Password)
            if not ok:return
            try:
                p=subprocess.run(['sudo','-S','-p','']+args,input=pw+'\n',stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,universal_newlines=True,timeout=8)
                rc,out=p.returncode,p.stdout.strip()
            finally: pw=None
        self.log('can0 啟動：{}'.format('成功' if rc==0 else out))
        self.schedule_status(force=True)

    def read_can(self):
        self.async_task('candump',lambda:run_cmd(['timeout','6s','candump','-n','8','can0'],timeout=8)[1] or '未收到 CAN Frame')

    def can_rx_ok(self):
        x=status_snapshot()
        return x['rx']>0 and x['can'].startswith('UP')

    def ensure_live_clock(self):
        rc, out = run_cmd(['rosparam','get','/use_sim_time'], timeout=3, ros=True)
        if rc == 0 and out.strip().lower() == 'true':
            answer = QMessageBox.question(self, '線上模式時間設定',
                '目前 ROS Master 設定 /use_sim_time=true，通常是之前離線 Bag 播放留下的。\n'
                '要切換成 Live System Time（false）嗎？\n'
                '若還在播放離線 Bag，請選 No。',
                QMessageBox.Yes|QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                self.log('拒絕更動 /use_sim_time；取消線上 Driver 啟動。')
                return False
            rc, msg = run_cmd(['rosparam','set','/use_sim_time','false'],timeout=4,ros=True)
            if rc != 0:
                self.log('無法設定 /use_sim_time=false：'+msg)
                return False
            self.log('已設定 /use_sim_time=false，線上資料採用系統時間。')
        return True

    def start_when_master(self,role,args,hunter=False,attempt=0):
        if master_ok():
            if role in ('hunter','lidar') and not self.ensure_live_clock(): return
            self.start_proc(role,args,hunter=hunter)
        elif attempt>=12:
            self.log('ROS Master 未就緒，取消啟動 {}。'.format(role))
        else:
            if not self.is_running('core'): self.start_master()
            QTimer.singleShot(500,lambda:self.start_when_master(role,args,hunter,attempt+1))

    def start_hunter(self):
        if '/hunter_status' in self.snapshot.get('topics',set()) and not self.is_running('hunter'):
            self.log('ROS Master 上已存在 Hunter 狀態 Topic，請先確認是否已有外部 Hunter Driver。')
            return
        if not self.snapshot.get('can','').startswith('UP') or self.snapshot.get('rx',0)==0:
            self.log('Hunter Driver 啟動前請先確認 can0 UP 且 RX>0（可按重新檢測）。');return
        if QMessageBox.question(self,'Hunter 驅動安全確認',
           'Driver 可能執行 EnableCommandedMode()。請確認 Hunter 停止、遙控器與實體急停可用。\n\n啟動 Hunter ROS Driver？',
           QMessageBox.Yes|QMessageBox.No,QMessageBox.No)!=QMessageBox.Yes:return
        # Isolate Hunter's odom TF from Autoware's independently estimated NDT pose:
        # odom -> hunter_odom_base_link; map -> base_link -> velodyne.
        # The CAN feedback and /odom topic remain available; this is not map/odom fusion.
        tf_launch=os.path.join(BUNDLE,'launch','hunter_ndt_tf_isolated.launch')
        if not os.path.isfile(tf_launch):
            self.log('缺少 TF 隔離啟動檔：'+tf_launch);return
        self.start_when_master('hunter',['roslaunch',tf_launch],hunter=True)
        self.log('Hunter odom TF 已隔離：odom→hunter_odom_base_link；NDT 將發布 map→base_link。')

    def start_lidar(self):
        if '/velodyne_points' in self.snapshot.get('topics',set()) and not self.is_running('lidar'):
            self.log('ROS Master 上已有 /velodyne_points，禁止重複啟動 LiDAR Driver。')
            return
        port=self.port.value()
        self.start_when_master('lidar',['roslaunch','velodyne_pointcloud','VLP16_points.launch',
                                        'device_ip:=','port:={}'.format(port)])
        self.log('device_ip 空白；請確保 Windows UDP Relay 持續轉送到 WSL2 此次 IP。')

    def udp_check(self):
        if self.is_running('lidar') or '/velodyne_points' in self.snapshot.get('topics',set()):
            self.log('LiDAR Driver 已在運作。請用 /velodyne_points Hz 驗證，避免 UDP 2368 同時綁定。');return
        self.async_task('UDP 檢測',lambda:test_udp(self.port.value()))

    def check_rates(self):
        self.async_task('Hunter Hz',lambda:topic_rate('/hunter_status'))
        self.async_task('LiDAR Hz',lambda:topic_rate('/velodyne_points'))

    def open_rviz(self):
        cfg=os.path.join(BUNDLE,'hunter_live.rviz')
        args=['rosrun','rviz','rviz']
        if os.path.isfile(cfg): args+=['-d',cfg]
        self.start_when_master('rviz',args)

    def check_hunter_message(self):
        """Verify custom ROS message loading under the same sourced environment."""
        def work():
            rc, out = run_cmd(['python2','-c',
                'from hunter_msgs.msg import HunterStatus; print("hunter_msgs/HunterStatus OK")'],
                timeout=6, ros=True)
            if rc == 0:
                return out or 'hunter_msgs/HunterStatus OK'
            return ('Hunter Msg 載入失敗：' + out[-500:] +
                    '\n請確認 ~/hunter_ws/devel/setup.bash 與 hunter_msgs 已編譯。')
        self.async_task('Hunter Msg', work)

    def check_hunter_state(self):
        self.async_task('Hunter 狀態',lambda:run_cmd(['rostopic','echo','-n','1','/hunter_status'],timeout=5,ros=True)[1][:1200])

    def request_motion(self,action):
        if self.is_running('motion') or self.motion_active:
            self.log('已存在執行中的動作測試，禁止同時啟動第二個。');return
        script=os.path.join(BUNDLE,'hunter_motion_test.py')
        if not os.path.isfile(script):script=os.path.join(HOME,'hunter_motion_test.py')
        if not os.path.isfile(script):
            self.log('找不到原有測試程式：{}，請先確認你先前的檔案。'.format(script));return
        if not self.is_running('hunter') and '/hunter_status' not in self.snapshot.get('topics',set()):
            self.log('找不到 Hunter Driver /hunter_status。');return
        self.motion_active=True
        self.async_task('運動安全預檢',lambda:self.motion_preflight(),lambda s:self.finish_motion_preflight(action,s))

    def motion_preflight(self):
        rc,info=run_cmd(['rostopic','info','/cmd_vel'],timeout=5,ros=True)
        if rc!=0:return '無法讀取 /cmd_vel 狀態：'+info[-180:]
        section=info.split('Publishers:',1)[-1].split('Subscribers:',1)[0]
        if re.search(r'^\s*\*\s*/',section,re.M):return '已存在其他 /cmd_vel Publisher，禁止測試。'
        rc,out=run_cmd(['rostopic','echo','-n','1','/hunter_status'],timeout=5,ros=True)
        if rc!=0:return '無法取得 Hunter 狀態：'+out[-180:]
        def number(field):
            m=re.search(r'^'+field+r':\s*([-\d.]+)',out,re.M)
            return float(m.group(1)) if m else None
        if number('fault_code') is None or number('base_state') is None or number('linear_velocity') is None:
            return 'Hunter 狀態資料不完整，禁止測試。'
        if number('fault_code')!=0 or number('base_state')!=0:
            return 'Hunter 當前 fault_code={} / base_state={}，禁止測試。'.format(number('fault_code'),number('base_state'))
        if abs(number('linear_velocity'))>0.02:return 'Hunter 尚未靜止，禁止測試。'
        return 'OK'

    def finish_motion_preflight(self,action,result):
        if result!='OK':
            self.motion_active=False;self.log('控制測試取消：'+result);return
        labels={'forward':'前進','left':'左轉','right':'右轉'}
        if QMessageBox.warning(self,'實車低速測試：'+labels[action],
            '確認周圍淨空、可立即使用 Hunter 實體急停、無人站在車輛行進路線上。\n'
            '本測試會讓實車移動，軟體 STOP 不等於實體急停。\n\n繼續？',
            QMessageBox.Yes|QMessageBox.No,QMessageBox.No)!=QMessageBox.Yes:
            self.motion_active=False;return
        typed,ok=QInputDialog.getText(self,'第一階段授權','請輸入 RUN，讓腳本先進入零速度準備階段：')
        if not ok or typed!='RUN':self.motion_active=False;return
        script=os.path.join(BUNDLE,'hunter_motion_test.py')
        if not os.path.isfile(script):script=os.path.join(HOME,'hunter_motion_test.py')
        if not self.start_proc('motion',['python2','-u',script,action],ros=True,hunter=True,stdin_data=b'RUN\n'):
            self.motion_active=False
        else:
            self.log('已輸入 RUN，等待零速度準備階段；尚未發送 GO。')
            QTimer.singleShot(2200, lambda a=action: self.motion_go_dialog(a))

    def motion_go_dialog(self,action):
        proc=self.proc.get('motion')
        if proc is None or proc.poll() is not None:
            self.motion_active=False
            self.log('腳本已結束或無法繼續，沒有發出 GO。')
            return
        typed,ok=QInputDialog.getText(self,'最終動作授權',
            '腳本已進入準備階段。最後確認車輛周圍淨空、遙控器與急停可用。\n請輸入 GO 進行 1 秒動作：')
        if not ok or typed!='GO':
            self.log('操作員取消 GO，終止動作測試。')
            self.soft_stop()
            return
        try:
            proc.stdin.write(b'GO\n');proc.stdin.flush();proc.stdin.close()
            self.log('已發送 GO，請監看車體動作。')
        except (OSError,ValueError) as e:
            self.log('無法發送 GO：'+str(e))
            self.soft_stop()

    def soft_stop(self):
        if self.is_running('motion'): self.stop('motion')
        helper=os.path.join(BUNDLE,'publish_zero.py')
        if os.path.isfile(helper):
            self.start_proc('zero',['python2','-u',helper],ros=True,hunter=True)
        self.log('已要求發送零速度。注意：這不是實體急停。')

    def pick_bag_folder(self):
        x=QFileDialog.getExistingDirectory(self,'選擇 ROS Bag 資料夾',self.bag_folder.text())
        if x:self.bag_folder.setText(x)

    def start_bag(self):
        if self.is_running('bag'):
            self.log('Bag 正在錄製中。');return
        if not master_ok(): self.log('請先啟動 ROS Master。');return
        selected=[t for t,c in self.topic_check.items() if c.isChecked()]
        if not selected:self.log('請至少選擇一個 ROS Topic。');return
        folder=os.path.abspath(os.path.expanduser(self.bag_folder.text()))
        try:os.makedirs(folder,exist_ok=True)
        except OSError as e:self.log('無法建立 Bag 資料夾：'+str(e));return
        usage=shutil.disk_usage(folder)
        if usage.free<2*1024**3:
            if QMessageBox.warning(self,'磁碟剩餘空間不足',
                '剩餘空間不足 2 GiB，點雲 Bag 可能快速寫滿磁碟。\n仍要錄製？',
                QMessageBox.Yes|QMessageBox.No,QMessageBox.No)!=QMessageBox.Yes:return
        prefix=re.sub(r'[^A-Za-z0-9_-]','_',self.bag_prefix.text().strip() or 'hunter_live')
        stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        path=os.path.join(folder,prefix+'_'+stamp)
        if self.start_proc('bag',['rosbag','record','--split','--size=1024','-o',path]+selected):
            self.bag_info.setText('正在錄製：'+path+'*.bag')
            self.log('注意：Bag 預設使用即時系統時間；請勿同時啟用 /use_sim_time 或 --clock 離線播放。')

    def open_bag_folder(self):
        folder=os.path.abspath(os.path.expanduser(self.bag_folder.text()))
        os.makedirs(folder,exist_ok=True)
        subprocess.Popen(['xdg-open',folder],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)

    def closeEvent(self,event):
        if any(self.is_running(k) for k in self.proc):
            result=QMessageBox.question(self,'結束 Hunter GUI',
                '將停止此 GUI 啟動的 ROS 節點及錄製工作。確定離開？',
                QMessageBox.Yes|QMessageBox.No,QMessageBox.No)
            if result!=QMessageBox.Yes:event.ignore();return
            self.stop_all()
            bag=self.proc.get('bag')
            if bag is not None and bag.poll() is None:
                try: bag.wait(timeout=8)
                except subprocess.TimeoutExpired: self.log('Bag 尚未完成關閉，請檢查 .active 暫存檔。')
        self.pool.shutdown(wait=False)
        event.accept()


# -------- v1.3 extension: preserve the stable Phase 1/2 device backend --------
import json
from PyQt5.QtWidgets import QDoubleSpinBox
from aw_gui_core import (mapping_config, match_config, ndt_config_command,
                         tf_arguments, MAP_OUTPUT_TYPE, convert_pose_csv)


class OnlineGUIV13(OnlineGUI):
    """Single device hub + live NDT workflow + non-actuating waypoint preview."""

    def ui(self):
        self.ui_style_template = '''
            QWidget { background:#101826; color:#eaf1f9; font-size:__BASE__px; font-family:'Noto Sans CJK TC','Microsoft JhengHei',sans-serif; }
            QLabel#title { font-size:__TITLE__px; font-weight:800; color:#53e2cf; }
            QGroupBox { background:#192639; border:1px solid #33465e; border-radius:9px; padding:13px; margin-top:16px; font-weight:bold; }
            QGroupBox::title { subcontrol-origin:margin; left:14px; padding:0 8px; }
            QPushButton { background:#30445e; border:1px solid #476581; border-radius:6px; padding:8px 12px; }
            QPushButton:hover { background:#43607a; } QPushButton:disabled { color:#8692a4; background:#273243; }
            QPushButton#danger { background:#833943; border-color:#ad5964; }
            QLineEdit,QSpinBox,QDoubleSpinBox { background:#0b1522; border:1px solid #40516a; padding:6px; border-radius:6px; }
            QTabWidget::pane { border:1px solid #33465e; }
            QTabBar::tab { padding:12px 18px; background:#1e2c40; }
            QTabBar::tab:selected { background:#24506b; }
            QTextEdit { background:#071120; font-family:'DejaVu Sans Mono'; font-size:__LOG__px; border:1px solid #35485e; }
        '''
        self.setWindowTitle('HUNTER / AUTOWARE-LITE ONLINE — v1.5 Controlled Navigation Test')
        self.resize(1280, 880)
        root=QWidget(); self.setCentralWidget(root); layout=QVBoxLayout(root)
        header=QHBoxLayout();layout.addLayout(header)
        title=QLabel('HUNTER / AUTOWARE-LITE ONLINE');title.setObjectName('title')
        header.addWidget(title);header.addStretch();header.addWidget(QLabel('介面字級'))
        self.font_spin=QSpinBox();self.font_spin.setRange(12,24);self.font_spin.setSuffix(' px')
        self.font_spin.setFixedWidth(105)
        try: sz=int(self.settings.value('ui_font_px',15))
        except (TypeError,ValueError): sz=15
        self.font_spin.setValue(max(12,min(24,sz)));self.font_spin.valueChanged.connect(self.apply_font_size)
        header.addWidget(self.font_spin)
        layout.addWidget(QLabel('Windows 11 / WSL2 / ROS Melodic · Live Mapping / Matching · Controlled Navigation Test'))
        self.global_status=QLabel('ROS / CAN / Hunter / LiDAR：檢測中…')
        self.global_status.setWordWrap(True)
        self.global_status.setStyleSheet('background:#203249; padding:10px; border-radius:7px')
        layout.addWidget(self.global_status)
        self.tabs=QTabWidget();layout.addWidget(self.tabs,1)
        self.tabs.addTab(self.page_overview(),'00  設備總覽')
        self.tabs.addTab(self.page_settings(),'01  專案 / TF')
        self.tabs.addTab(self.page_mapping(),'02  NDT 建圖')
        self.tabs.addTab(self.page_localization(),'03  NDT 定位')
        self.tabs.addTab(self.page_navigation(),'04  Waypoint / 路徑測試')
        self.tabs.addTab(self.page_bag(),'05  ROS Bag')
        layout.addWidget(QLabel('執行日誌（GUI 管理的 ROS 程序、演算法啟停與錯誤）'))
        self.logs=QTextEdit();self.logs.setReadOnly(True);self.logs.document().setMaximumBlockCount(800)
        self.logs.setMinimumHeight(130);layout.addWidget(self.logs,1)
        footer=QHBoxLayout();layout.addLayout(footer)
        footer.addWidget(QLabel('實車路徑測試需要持續按住授權鍵；GUI 的 STOP 不是硬體急停。'))
        footer.addStretch();footer.addWidget(self.btn('停止本 GUI 啟動的節點',self.stop_all, danger=True))

    def page_overview(self):
        w=QWidget();l=QVBoxLayout(w)
        g=self.group('系統健康與 ROS Topic 狀態');l.addWidget(g)
        self.master_label=QLabel('ROS Master：待檢查')
        self.can_label=QLabel('CAN：待檢查')
        self.hunter_label=QLabel('Hunter：待檢查')
        self.lidar_label=QLabel('LiDAR：待檢查')
        self.odom_label=QLabel('Odometry：待檢查')
        self.can_details=QLabel('CAN 詳細資料待檢查')
        for label in (self.master_label,self.can_label,self.hunter_label,self.lidar_label,self.odom_label):
            label.setWordWrap(True);g.layout().addWidget(label)
        row=QHBoxLayout();g.layout().addLayout(row)
        row.addWidget(self.btn('重新檢測',lambda:self.schedule_status(force=True)))
        row.addWidget(self.btn('檢查點雲 / Hunter Hz',self.check_rates))
        row.addWidget(self.btn('查看 /hunter_status',self.check_hunter_state))
        row.addWidget(self.btn('驗證 Hunter Msg',self.check_hunter_message))
        g=self.group('設備控制（CAN / ROS Master / Hunter / VLP-16 / RViz 的唯一操作入口）');l.addWidget(g)
        row=QHBoxLayout();g.layout().addLayout(row)
        row.addWidget(self.btn('啟動 can0（500000）',self.can_up))
        row.addWidget(self.btn('讀取 8 筆 CAN',self.read_can))
        row.addWidget(self.btn('檢查 CAN 統計',lambda:self.schedule_status(force=True)))
        row.addWidget(self.btn('啟動 ROS Master',self.start_master))
        row=QHBoxLayout();g.layout().addLayout(row)
        row.addWidget(self.btn('啟動 Hunter Driver',self.start_hunter))
        row.addWidget(self.btn('停止 Hunter Driver',lambda:self.stop('hunter')))
        row.addWidget(self.btn('啟動 VLP-16 Driver',self.start_lidar))
        row.addWidget(self.btn('停止 VLP-16 Driver',lambda:self.stop('lidar')))
        row=QHBoxLayout();g.layout().addLayout(row)
        row.addWidget(QLabel('Velodyne UDP Port'))
        self.port=QSpinBox();self.port.setRange(1,65535)
        self.port.setValue(int(self.settings.value('udp_port',2368)));self.port.valueChanged.connect(lambda v:self.settings.setValue('udp_port',v))
        row.addWidget(self.port)
        row.addWidget(self.btn('測試 UDP（啟動 Driver 前）',self.udp_check))
        row.addWidget(self.btn('開啟 RViz',self.open_rviz))
        row.addWidget(self.btn('停止 RViz',lambda:self.stop('rviz')))
        g.layout().addWidget(QLabel('Live：device_ip 留空、port 預設 2368；Windows UDP Relay 必須持續運作。'))
        g=self.group('Hunter 人工低速測試（會移動真車，需現場安全授權）');l.addWidget(g)
        row=QHBoxLayout();g.layout().addLayout(row)
        for action,label in (('forward','前進 1 秒'),('left','左轉 1 秒'),('right','右轉 1 秒')):
            row.addWidget(self.btn(label,lambda _=None,a=action:self.request_motion(a)))
        g.layout().addWidget(self.btn('軟體 STOP（送零速度，不取代實體急停）',self.soft_stop,danger=True))
        l.addStretch();return self.scroll_page(w)

    def page_settings(self):
        w=QWidget();l=QVBoxLayout(w)
        g=self.group('專案路徑與實機參數');l.addWidget(g)
        form=QFormLayout();g.layout().addLayout(form)
        self.workspace=QLineEdit(str(self.settings.value('autoware_workspace',os.path.join(HOME,'autoware.ai'))))
        self.map_file=QLineEdit(str(self.settings.value('ndt_map_path',os.path.join(HOME,'autoware.ai','maps','hunter_live.pcd'))))
        self.traj_folder=QLineEdit(str(self.settings.value('traj_folder',os.path.join(HOME,'hunter_online_gui','trajectories'))))
        form.addRow('Autoware 工作空間',self.workspace)
        form.addRow('PCD 地圖路徑',self.map_file)
        form.addRow('NDT 軌跡輸出目錄',self.traj_folder)
        g.layout().addWidget(self.btn('儲存專案設定',self.save_live_settings))
        g=self.group('TF：base_link → velodyne（實測安裝外參）');l.addWidget(g)
        g.layout().addWidget(QLabel('數值不可視為自動校正；全零只是預設值。先測量感測器安裝位置後再啟用。'))
        grid=QGridLayout();g.layout().addLayout(grid)
        self.tf_spin={}
        for idx,(name,unit) in enumerate((('x','m'),('y','m'),('z','m'),('roll','rad'),('pitch','rad'),('yaw','rad'))):
            s=QDoubleSpinBox();s.setRange(-100,100);s.setDecimals(5);s.setSingleStep(.05)
            s.setValue(float(self.settings.value('live_tf_'+name,0)))
            self.tf_spin[name]=s;grid.addWidget(QLabel('{} ({})'.format(name,unit)),idx//3*2,idx%3)
            grid.addWidget(s,idx//3*2+1,idx%3)
        row=QHBoxLayout();g.layout().addLayout(row)
        row.addWidget(self.btn('套用 / 啟動 TF',self.start_live_tf))
        row.addWidget(self.btn('停止本 GUI 的 TF',lambda:self.stop('live_tf')))
        row.addWidget(self.btn('檢查 TF',self.check_tf))
        g=self.group('Live 資料鏈路');l.addWidget(g)
        g.layout().addWidget(QLabel('/velodyne_points → (Live Bridge) → /points_raw（建圖）或 /filtered_points（定位）。'))
        g.layout().addWidget(QLabel('Live Mode 必須 /use_sim_time=false。Mapping 與 Matching 不能同時執行。'))
        g.layout().addWidget(QLabel('v1.4：Hunter TF 使用 odom→hunter_odom_base_link，NDT 使用 map→base_link；兩個 TF 分支刻意分離，不假造 map→odom。'))
        l.addStretch();return self.scroll_page(w)

    def ndt_form(self,mode):
        defaults=mapping_config() if mode=='mapping' else match_config()
        keys=['resolution','step_size','trans_epsilon','max_iterations']
        if mode=='mapping':keys+=['leaf_size','min_scan_range','max_scan_range','min_add_scan_shift']
        box=self.group('NDT 參數（啟動時套用）')
        grid=QGridLayout();box.layout().addLayout(grid)
        self.ndt_fields[mode]={}
        for i,key in enumerate(keys):
            x=QSpinBox() if key=='max_iterations' else QDoubleSpinBox()
            if key=='max_iterations':x.setRange(1,300)
            else:x.setRange(0 if key in ('min_scan_range','min_add_scan_shift') else .00001,300);x.setDecimals(5)
            try:value=float(self.settings.value('v13_ndt_{}_{}'.format(mode,key),defaults[key]))
            except (TypeError,ValueError):value=defaults[key]
            x.setValue(value);self.ndt_fields[mode][key]=x
            grid.addWidget(QLabel(key),i//4*2,i%4);grid.addWidget(x,i//4*2+1,i%4)
        box.layout().addWidget(QLabel('預設參數來自原離線版；修改後需要停止並重啟 NDT 節點。'))
        return box

    def page_mapping(self):
        self.ndt_fields={}
        w=QWidget();l=QVBoxLayout(w)
        g=self.group('02 Live NDT Mapping（僅使用即時光達；不播放 Bag）');l.addWidget(g)
        g.layout().addWidget(QLabel('先在 00 啟動 LiDAR 並確認 Hz，在 01 套用 TF。RViz 顯示 /ndt_map + 即時點雲 + /current_pose Hunter；可用遙控器低速建圖。'))
        row=QHBoxLayout();g.layout().addLayout(row)
        row.addWidget(self.btn('開始線上建圖',self.start_mapping))
        row.addWidget(self.btn('停止建圖',self.stop_mapping))
        row.addWidget(self.btn('確認 /points_raw Hz',lambda:self.async_task('points_raw Hz',lambda:topic_rate('/points_raw'))))
        row.addWidget(self.btn('開啟建圖 RViz',lambda:self.open_ndt_rviz('mapping')))
        row.addWidget(self.btn('檢查建圖顯示 / TF',lambda:self.diagnose_ndt_rviz('mapping')))
        l.addWidget(self.ndt_form('mapping'))
        g=self.group('輸出 PCD');l.addWidget(g)
        row=QHBoxLayout();g.layout().addLayout(row)
        row.addWidget(QLabel('輸出 Voxel Filter (m)'))
        self.pcd_resolution=QDoubleSpinBox();self.pcd_resolution.setRange(.01,5);self.pcd_resolution.setValue(.2)
        row.addWidget(self.pcd_resolution)
        row.addWidget(self.btn('儲存 PCD（建圖進行中）',self.export_pcd))
        self.map_note=QLabel('建圖尚未啟動。');g.layout().addWidget(self.map_note)
        l.addStretch();return self.scroll_page(w)

    def page_localization(self):
        w=QWidget();l=QVBoxLayout(w)
        g=self.group('03 Live NDT Matching（載入 PCD，接收即時光達）');l.addWidget(g)
        row=QHBoxLayout();g.layout().addLayout(row)
        row.addWidget(self.btn('開始線上定位',self.start_matching))
        row.addWidget(self.btn('停止定位',self.stop_matching))
        row.addWidget(self.btn('檢查 /ndt_pose Hz',lambda:self.async_task('NDT Pose Hz',lambda:topic_rate('/ndt_pose'))))
        row.addWidget(self.btn('開啟定位 RViz',lambda:self.open_ndt_rviz('matching')))
        row.addWidget(self.btn('檢查定位顯示 / TF',lambda:self.diagnose_ndt_rviz('matching')))
        g.layout().addWidget(QLabel('定位 RViz 同時顯示 /points_map（PCD）與 /velodyne_points（即時掃描），以及 /ndt_pose 的 Hunter 標記與軌跡。'))
        g.layout().addWidget(QLabel('定位失敗或匹配跳點時，不可使用推估位姿控制真車。'))
        perf=self.group('NDT Matching CPU 負載（只影響 NDT 輸入，不影響 RViz / Bag）')
        perf_row=QHBoxLayout();perf.layout().addLayout(perf_row)
        perf_row.addWidget(QLabel('點雲取樣間隔：每 N 點保留 1 點'))
        self.ndt_stride=QSpinBox();self.ndt_stride.setRange(1,4)
        self.ndt_stride.setValue(int(self.settings.value('ndt_match_stride',3)))
        self.ndt_stride.setToolTip('3 約保留 1/3，2 約保留 1/2；需要停止並重啟 03 定位才會套用。')
        perf_row.addWidget(self.ndt_stride)
        perf_row.addStretch()
        perf.layout().addWidget(QLabel('建議先用 3；若定位點數不足／不穩，請停止定位後改 2 或 1，並檢查 CPU 時間。請確認定位沒有跳點、點雲與 PCD 對齊。'))
        l.addWidget(perf)
        l.addWidget(self.ndt_form('matching'))
        g=self.group('定位初始位姿（map frame）');l.addWidget(g)
        self.init_fields={};grid=QGridLayout();g.layout().addLayout(grid)
        for i,key in enumerate(('x','y','z','yaw')):
            box=QDoubleSpinBox();box.setRange(-10000,10000);box.setDecimals(4)
            box.setValue(float(self.settings.value('init_'+key,0)))
            self.init_fields[key]=box;grid.addWidget(QLabel(key+' (m)' if key!='yaw' else 'yaw (rad)'),0,i)
            grid.addWidget(box,1,i)
        g.layout().addWidget(self.btn('重新發布 NDT 初始化',self.publish_initial_pose))
        g=self.group('定位軌跡 CSV');l.addWidget(g)
        row=QHBoxLayout();g.layout().addLayout(row)
        row.addWidget(self.btn('開始錄製 NDT CSV',self.start_pose_capture))
        row.addWidget(self.btn('停止並整理 CSV',self.stop_pose_capture))
        self.loc_note=QLabel('尚未開始定位。');g.layout().addWidget(self.loc_note)
        l.addStretch();return self.scroll_page(w)

    def page_navigation(self):
        w=QWidget();l=QVBoxLayout(w)
        g=self.group('04 Waypoint / Pure Pursuit：只計算、不輸出馬達命令');l.addWidget(g)
        g.layout().addWidget(QLabel('必須先取得穩定的 Live /ndt_pose，載入正確 PCD。所有預覽結果只存在 /offline_nav/*。'))
        self.nav_csv=QLineEdit(str(self.settings.value('nav_input_csv',os.path.join(HOME,'hunter_online_gui','trajectories','ndt_matching_pose.csv'))))
        self.nav_path=QLineEdit(str(self.settings.value('nav_waypoint_csv',os.path.join(HOME,'hunter_online_gui','trajectories','hunter_waypoints.csv'))))
        form=QFormLayout();g.layout().addLayout(form)
        form.addRow('NDT 整理後 CSV',self.nav_csv)
        form.addRow('Waypoint CSV',self.nav_path)
        row=QHBoxLayout();g.layout().addLayout(row)
        row.addWidget(QLabel('取樣間距 m'))
        self.nav_spacing=QDoubleSpinBox();self.nav_spacing.setRange(.05,5);self.nav_spacing.setValue(.5)
        row.addWidget(self.nav_spacing)
        row.addWidget(QLabel('預覽速度 km/h'))
        self.nav_speed=QDoubleSpinBox();self.nav_speed.setRange(.1,18.0);self.nav_speed.setValue(1.0)
        row.addWidget(self.nav_speed)
        row.addWidget(self.btn('產生 Waypoint CSV',self.generate_nav_csv))
        row=QHBoxLayout();g.layout().addLayout(row)
        row.addWidget(self.btn('載入 Waypoint',self.load_waypoints))
        row.addWidget(self.btn('啟動 Pure Pursuit 預覽',self.start_nav_preview))
        row.addWidget(self.btn('停止導航預覽',self.stop_nav_preview))
        row.addWidget(self.btn('讀取 Twist 預覽',self.check_nav_twist))
        self.nav_note=QLabel('尚未啟動；不會對 /cmd_vel 發布任何命令。');g.layout().addWidget(self.nav_note)
        l.addStretch();return self.scroll_page(w)

    def save_live_settings(self):
        self.settings.setValue('autoware_workspace',self.workspace.text().strip())
        self.settings.setValue('ndt_map_path',self.map_file.text().strip())
        self.settings.setValue('traj_folder',self.traj_folder.text().strip())
        for k,v in self.tf_spin.items():self.settings.setValue('live_tf_'+k,v.value())
        for k,v in self.init_fields.items():self.settings.setValue('init_'+k,v.value())
        for mode,values in self.ndt_fields.items():
            for k,v in values.items():self.settings.setValue('v13_ndt_{}_{}'.format(mode,k),v.value())
        self.settings.setValue('ndt_match_stride',self.ndt_stride.value())
        self.settings.setValue('nav_input_csv',self.nav_csv.text().strip())
        self.settings.setValue('nav_waypoint_csv',self.nav_path.text().strip())
        self.settings.sync()
        self.log('已儲存 Live 專案、TF、NDT、Waypoint 設定；不修改離線 GUI。')

    def vpath(self,widget):return os.path.abspath(os.path.expanduser(widget.text().strip()))

    def autoware_ready(self,packages):
        workspace=self.vpath(self.workspace)
        if not os.path.isfile(os.path.join(workspace,'install','setup.bash')):
            self.log('找不到 Autoware overlay：'+os.path.join(workspace,'install','setup.bash'));return False
        os.environ['AUTOWARE_LIVE_WS']=workspace
        for pkg in packages:
            rc,out=run_cmd(['rospack','find',pkg],timeout=7,ros=True)
            if rc!=0:self.log('缺少 Autoware ROS 套件 {}：{}'.format(pkg,out[-200:]));return False
        return True

    def live_source_ready(self):
        if not master_ok():self.log('請先在 00 啟動 ROS Master。');return False
        if not self.ensure_live_clock():return False
        rc,t=run_cmd(['rostopic','list'],timeout=6,ros=True)
        if rc or '/velodyne_points' not in t.splitlines():
            self.log('找不到 /velodyne_points，先在 00 啟動 LiDAR Driver。');return False
        return True

    def no_ndt_conflicts(self):
        rc,nodes=run_cmd(['rosnode','list'],timeout=6,ros=True)
        if rc: self.log('無法檢查 ROS 節點：'+nodes[-200:]);return False
        if {'/ndt_mapping','/ndt_matching'} & set(nodes.splitlines()):
            self.log('偵測到既有 /ndt_mapping 或 /ndt_matching，請先停止其他程序。');return False
        return True

    def start_live_tf(self):
        if not master_ok():self.log('請先在 00 啟動 ROS Master。');return
        if self.is_running('live_tf'):self.log('請先停止舊 TF 才能重新設定。');return
        values={k:s.value() for k,s in self.tf_spin.items()}
        if not any(abs(v)>1e-9 for v in values.values()):
            reply=QMessageBox.warning(self,'外參尚未量測','目前 base_link → velodyne 外參全為零，並非實機校正結果。\n只做流程驗證才可繼續。',
                QMessageBox.Yes|QMessageBox.No,QMessageBox.No)
            if reply!=QMessageBox.Yes:return
        self.save_live_settings()
        self.start_proc('live_tf',tf_arguments(values)+['__name:=hunter_live_tf'])

    def open_ndt_rviz(self, mode):
        # Mapping and localization use different live topics and displays.
        if mode not in ('mapping', 'matching'): return
        role='ndt_rviz_'+mode
        other='ndt_rviz_'+('matching' if mode=='mapping' else 'mapping')
        if self.is_running(other):
            self.stop(other)
            QTimer.singleShot(500,lambda:self.open_ndt_rviz(mode))
            return
        cfg=os.path.join(BUNDLE,'hunter_ndt_'+mode+'.rviz')
        if not os.path.isfile(cfg):
            self.log('找不到 '+cfg+'；請完整更新 v1.4 WSL2 資料夾。');return
        self.start_when_master(role,['rosrun','rviz','rviz','-d',cfg])
        self.log('RViz 模式={}；Fixed Frame=map；Hunter 表示 NDT 估測，不是車輛 URDF。'.format(mode))

    def diagnose_ndt_rviz(self,mode):
        if mode not in ('mapping','matching'):return
        pose='/current_pose' if mode=='mapping' else '/ndt_pose'
        map_topic='/ndt_map' if mode=='mapping' else '/points_map'
        def work():
            rc,out=run_cmd(['rostopic','list'],timeout=6,ros=True)
            if rc:return 'rostopic list 失敗：'+out[-250:]
            topics=set(out.splitlines())
            names=('/velodyne_points',map_topic,pose,'/hunter_viz/robot_markers','/hunter_viz/path')
            lines=['RViz 顯示診斷 [{}]'.format(mode)]
            lines += [t+('：存在' if t in topics else '：尚未發布') for t in names]
            rc,tf=run_cmd(['timeout','4s','rosrun','tf','tf_echo','map','velodyne'],timeout=6,ros=True)
            if 'Translation:' in tf and 'Rotation:' in tf:
                lines.append('TF map→base_link→velodyne：可查詢')
            else:
                lines.append('TF map→velodyne：不可查詢，請確認 NDT 已產生位姿、01 外參與 Hunter Driver TF 隔離。')
                lines.append(tf[-250:])
            return '\n'.join(lines)
        self.async_task('NDT RViz 診斷',work)

    def check_tf(self):
        self.async_task('TF',lambda:run_cmd(['timeout','5s','rosrun','tf','tf_echo','base_link','velodyne'],timeout=7,ros=True)[1][-900:])

    def ndt_params(self,mode):
        x={k:f.value() for k,f in self.ndt_fields[mode].items()}
        if mode=='mapping':return mapping_config(x)
        pos={k:f.value() for k,f in self.init_fields.items()}
        return match_config(dict(x,**pos))

    def _start_ndt(self,mode):
        if self.is_running('mapping') or self.is_running('matching'):
            self.log('NDT Mapping / Matching 不可同時運作。');return
        if not self.live_source_ready() or not self.no_ndt_conflicts():return
        if not self.autoware_ready(['lidar_localizer']+(['pcl_ros'] if mode=='matching' else [])):return
        # Explicitly reject the old Hunter odom->base_link broadcaster.
        # Autoware NDT also broadcasts map->base_link: same child / two parents
        # makes the RViz TF tree ambiguous and can hide the live cloud.
        rc,base_frame=run_cmd(['rosparam','get','/hunter_base_node/base_frame'],timeout=4,ros=True)
        if rc==0 and base_frame.strip().lstrip('/')=='base_link':
            warn=('偵測到舊版 Hunter Driver 正發布 odom→base_link，與 NDT 的 '
                  'map→base_link 衝突。\n請先讓車靜止，停止舊 Driver，'
                  '再由 00 重新啟動 v1.4 Hunter Driver。')
            self.log(warn)
            QMessageBox.warning(self,'NDT / Hunter TF 衝突',warn)
            return
        if not self.is_running('hunter'):
            self.log('提示：Hunter Driver 非本 v1.4 GUI 啟動。必須確認沒有其他節點發布 odom→base_link；否則 NDT map→base_link 會衝突。')
        if not self.is_running('live_tf'):
            resp=QMessageBox.question(self,'TF 未由本 GUI 啟動',
                '本 GUI 尚未發布 base_link→velodyne。若你已在其他節點發布正確 TF 可繼續；否則請先到 01 啟動 TF。',
                QMessageBox.Yes|QMessageBox.No,QMessageBox.No)
            if resp!=QMessageBox.Yes:return
        if mode=='matching' and not os.path.isfile(self.vpath(self.map_file)):
            self.log('找不到 PCD 地圖：'+self.vpath(self.map_file));return
        try: cfg=self.ndt_params(mode)
        except (ValueError,TypeError) as e:self.log('NDT 參數有誤：'+str(e));return
        self.save_live_settings()
        for key,value in [('use_sim_time','false'),('localizer','velodyne')]:
            rc,out=run_cmd(['rosparam','set','/'+key,value],timeout=5,ros=True)
            if rc:self.log('無法設定 /'+key+'：'+out);return
        for key,field in self.tf_spin.items():
            run_cmd(['rosparam','set','/tf_'+key,str(field.value())],timeout=4,ros=True)
        target='/points_raw' if mode=='mapping' else '/filtered_points'
        bridge=os.path.join(BUNDLE,'live_points_bridge.py')
        points_args=['python2','-u',bridge,'--output',target]
        if mode=='matching':
            stride=self.ndt_stride.value()
            points_args+=['--stride',str(stride)]
            self.log('NDT Matching 取樣：每 {} 點保留 1 點；RViz / Bag 仍使用 /velodyne_points 原始點雲。'.format(stride))
        if not self.start_proc('live_points',points_args):return
        if mode=='matching':
            if not self.start_proc('map_publisher',['rosrun','pcl_ros','pcd_to_pointcloud',self.vpath(self.map_file),'0.5', '_frame_id:=map','/cloud_pcd:=/points_map']):
                self.stop('live_points');return
        args=['roslaunch','lidar_localizer','ndt_'+mode+'.launch','method_type:=0']
        if mode=='mapping':args+=['use_odom:=false','use_imu:=false']
        else:args+=['use_gnss:=false','use_odom:=false','use_imu:=false']
        if not self.start_proc(mode,args):
            self.stop('live_points');self.stop('map_publisher');return
        # RViz-only markers follow NDT pose. No TF or /cmd_vel is published.
        visualizer=os.path.join(BUNDLE,'hunter_pose_visualizer.py')
        pose_topic='/current_pose' if mode=='mapping' else '/ndt_pose'
        if os.path.isfile(visualizer):
            self.start_proc('pose_viz',['python2','-u',visualizer,'--pose-topic',pose_topic])
        else:
            self.log('缺少姿態顯示程式 '+visualizer+'，Hunter 外型標記無法顯示。')
        self.log('已啟動線上 {}；等待節點訂閱 /config/ndt* 後套用參數。'.format(mode))
        QTimer.singleShot(1600,lambda m=mode,c=cfg:self.wait_ndt_subscriber(m,c,0))

    def wait_ndt_subscriber(self,mode,config,attempt):
        if not self.is_running(mode):return
        topic='/config/ndt_mapping' if mode=='mapping' else '/config/ndt'
        rc,info=run_cmd(['rostopic','info',topic],timeout=4,ros=True)
        if rc==0 and 'Subscribers:' in info and '/ndt_'+mode in info.split('Subscribers:',1)[-1]:
            self.async_task('NDT '+mode+' Config',lambda:self.publish_ndt_config(mode,config))
            if mode=='matching':self.loc_note.setText('定位已啟動：等待 /ndt_pose，請檢查匹配品質與初始化位置。')
            else:self.map_note.setText('線上建圖已啟動，可於 RViz 查看並在此頁匯出 PCD。')
        elif attempt<12:
            QTimer.singleShot(950,lambda:self.wait_ndt_subscriber(mode,config,attempt+1))
        else:self.log('NDT '+mode+' 尚未訂閱 '+topic+'，請查看下方 ROS 日誌。')

    def publish_ndt_config(self,mode,config):
        cmd=ndt_config_command(mode,config)
        rc,out=run_cmd(cmd,timeout=12,ros=True)
        return 'Config {} {}'.format(mode,'已送出' if rc==0 else '失敗：'+out[-350:])

    def start_mapping(self):self._start_ndt('mapping')
    def stop_mapping(self):
        for role in ('config_mapping','mapping','live_points','save_map','pose_viz'):
            if self.is_running(role):self.stop(role)
        self.map_note.setText('已要求停止線上建圖。')

    def start_matching(self):self._start_ndt('matching')
    def stop_matching(self):
        self.stop_nav_preview()
        self.stop_pose_capture()
        for role in ('matching','map_publisher','live_points','pose_viz'):
            if self.is_running(role):self.stop(role)
        self.loc_note.setText('已要求停止線上定位。')

    def publish_initial_pose(self):
        if not self.is_running('matching'):
            self.log('請先啟動 NDT Matching。');return
        try:cfg=self.ndt_params('matching')
        except (TypeError,ValueError) as e:self.log(str(e));return
        self.async_task('重設 NDT 初始位姿',lambda:self.publish_ndt_config('matching',cfg))

    def export_pcd(self):
        if not self.is_running('mapping'):
            self.log('必須在建圖節點運作時匯出 PCD。');return
        path=self.vpath(self.map_file)
        if os.path.exists(path):
            reply=QMessageBox.question(self,'PCD 已存在','PCD 已存在，確定覆寫？\n'+path,
                QMessageBox.Yes|QMessageBox.No,QMessageBox.No)
            if reply!=QMessageBox.Yes:return
        os.makedirs(os.path.dirname(path),exist_ok=True)
        cfg={'header':{'frame_id':'map'},'filename':path,'filter_res':float(self.pcd_resolution.value())}
        self.async_task('儲存 PCD',lambda:run_cmd(['rostopic','pub','-1','/config/ndt_mapping_output',MAP_OUTPUT_TYPE,json.dumps(cfg)],timeout=13,ros=True)[1][-700:] or '已發送 PCD 匯出要求')
        self.log('PCD 匯出要求：'+path+'；請確認節點實際產生檔案及點數。')

    def start_pose_capture(self):
        if self.is_running('csv_pose'):
            self.log('NDT CSV 已在錄製。');return
        if not self.is_running('matching'):
            self.log('請先啟動 Live NDT Matching。');return
        folder=self.vpath(self.traj_folder);os.makedirs(folder,exist_ok=True)
        stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        path=os.path.join(folder,'ndt_matching_'+stamp+'_raw.csv')
        f=open(path,'w',encoding='utf-8')
        cmd=ros_cmd(['rostopic','echo','-p','/ndt_pose'])
        try:
            proc=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.PIPE,start_new_session=True)
            self.proc['csv_pose']=proc;self.csv_file=f;self.csv_raw_path=path
            self.loc_note.setText('正在記錄：'+path)
            self.log('開始錄製 NDT 位姿 CSV：'+path)
        except OSError as e:f.close();self.log('CSV 無法啟動：'+str(e))

    def stop_pose_capture(self):
        if not self.is_running('csv_pose'):return
        self.stop('csv_pose')
        QTimer.singleShot(350,self.finalize_pose_csv)

    def finalize_pose_csv(self,attempt=0):
        proc=self.proc.get('csv_pose')
        if proc and proc.poll() is None:
            if attempt<30:QTimer.singleShot(200,lambda:self.finalize_pose_csv(attempt+1))
            else:self.log('CSV 停止逾時；請先檢查輸出檔案。')
            return
        if hasattr(self,'csv_file') and not self.csv_file.closed:self.csv_file.close()
        raw=getattr(self,'csv_raw_path','')
        if raw and os.path.exists(raw):
            dest=raw.replace('_raw.csv','_pose.csv')
            def work():
                try:return ('CSV 已整理 {} 筆：{}'.format(convert_pose_csv(raw,dest),dest))
                except Exception as e:return 'CSV 整理失敗：'+str(e)
            self.async_task('NDT CSV',work,self._csv_finish)
    def _csv_finish(self,text):
        self.log(text);self.loc_note.setText(text)
        if 'CSV 已整理' in text:
            raw=getattr(self,'csv_raw_path','')
            self.nav_csv.setText(raw.replace('_raw.csv','_pose.csv'))
            self.save_live_settings()

    def generate_nav_csv(self):
        source,target=self.vpath(self.nav_csv),self.vpath(self.nav_path)
        if not os.path.isfile(source):self.log('找不到整理後 NDT CSV：'+source);return
        script=os.path.join(BUNDLE,'generate_waypoints.py')
        cmd=['python3',script,'--input',source,'--output',target,
             '--spacing-m',str(self.nav_spacing.value()),'--speed-kmh',str(self.nav_speed.value())]
        self.async_task('Waypoint CSV',lambda:self._run_waypoint_generator(cmd))

    def _run_waypoint_generator(self,cmd):
        rc,out=run_cmd(cmd,timeout=35)
        return ('成功：' if rc==0 else '失敗：')+(out[-1000:] or '程式沒有輸出')

    def load_waypoints(self):
        if not self.is_running('matching'):
            self.log('請先啟動 Live NDT Matching，確認 /ndt_pose 穩定後再載入 Waypoint。');return
        if not self.autoware_ready(['waypoint_maker']):return
        path=self.vpath(self.nav_path)
        if not os.path.isfile(path):self.log('Waypoint CSV 不存在：'+path);return
        if self.is_running('nav_bridge') or self.is_running('nav_loader'):
            self.log('Waypoint 已載入；若要更換請先停止導航預覽。');return
        bridge=os.path.join(BUNDLE,'offline_nav_bridge.py')
        if self.start_proc('nav_bridge',['python2','-u',bridge]):
            QTimer.singleShot(1100,lambda:self._launch_waypoint_loader(path))
    def _launch_waypoint_loader(self,path):
        if not self.is_running('nav_bridge'):self.log('導航橋接未啟動。');return
        if self.start_proc('nav_loader',['rosrun','waypoint_maker','waypoint_loader','_multi_lane_csv:='+path]):
            self.nav_note.setText('已載入 Waypoint：'+path+'；等待 /based/lane_waypoints_raw。')

    def start_nav_preview(self):
        if not self.is_running('nav_bridge'):
            self.log('請先載入 Waypoint。');return
        if self.is_running('nav_preview'):
            self.log('Pure Pursuit 預覽已在執行。');return
        direct=bool(getattr(self,'nav_loaded_path',None))
        required=['twist_filter'] if direct else ['pure_pursuit','twist_filter']
        if not self.autoware_ready(required):return
        launch=os.path.join(BUNDLE,'launch','online_pure_pursuit_preview.launch')
        args=['roslaunch',launch,'direct_ordered_pp:='+('true' if direct else 'false')]
        if hasattr(self,'nav_lookahead_ratio') and hasattr(self,'nav_min_lookahead'):
            ratio,minimum=self.pure_pursuit_follow_values()
            # Set ROS params at launch, then publish ConfigWaypointFollower
            # after Pure Pursuit is subscribed. Both paths use the same values.
            args.extend(['lookahead_ratio:={:.4f}'.format(ratio),
                         'min_lookahead:={:.4f}'.format(minimum)])
        if self.start_proc('nav_preview',args):
            if hasattr(self,'nav_lookahead_ratio') and hasattr(self,'nav_min_lookahead'):
                self.nav_preview_follow_params=self.pure_pursuit_follow_values()
            self.nav_note.setText('預覽運作中：/offline_nav/twist_cmd；導航命令由 Supervisor 管理。')
            if direct:self.log('Ordered PP：直接使用弧長預瞄點計算曲率；保留 Autoware twist_filter，避免舊 PP 重選目標後輸出零速度。')
            QTimer.singleShot(1500,self.publish_nav_config)

    def _start_nav_config_publisher(self,role,topic,typ,obj,attempt=0):
        """Re-publish config without blocking GUI heartbeats while a prior
        `rostopic pub -1` process is still exiting (it can live ~3 seconds).
        """
        if self.is_running(role):
            if attempt==0:self.stop(role)
            if attempt>=25:
                self.log('{} 設定發布程序未退出，請停止預覽後重啟。'.format(role))
                return
            QTimer.singleShot(120,lambda:self._start_nav_config_publisher(
                role,topic,typ,obj,attempt+1))
            return
        self.start_proc(role,['rostopic','pub','-1',topic,typ,json.dumps(obj)])

    def publish_nav_config(self):
        if not self.is_running('nav_preview'):return
        speed_kmh=(self.applied_nav_speed()*3.6 if hasattr(self,'nav_cruise_speed')
                   else self.nav_speed.value())
        ratio,minimum=(self.pure_pursuit_follow_values()
                       if hasattr(self,'nav_lookahead_ratio') and hasattr(self,'nav_min_lookahead')
                       else (2.0,1.5))
        # Match the original offline GUI's ConfigWaypointFollower settings.
        payload={'param_flag':0,'velocity':speed_kmh,
                 'lookahead_distance':minimum,'lookahead_ratio':ratio,
                 'minimum_lookahead_distance':minimum}
        filt={'lateral_accel_limit':1.0,'lateral_jerk_limit':5.0,
              'lowpass_gain_linear_x':0.0,'lowpass_gain_angular_z':0.0}
        for role,topic,typ,obj in (
            ('nav_cfg','/offline_nav/config/waypoint_follower','autoware_config_msgs/ConfigWaypointFollower',payload),
            ('nav_filter_cfg','/offline_nav/config/twist_filter','autoware_config_msgs/ConfigTwistFilter',filt)):
            # ros CLI pub -1 may live a few seconds. Restart asynchronously
            # to avoid blocking the GUI's 100-ms navigation watchdog heartbeat.
            self._start_nav_config_publisher(role,topic,typ,obj)
        if hasattr(self,'nav_preview_follow_params'):
            self.nav_preview_follow_params=(ratio,minimum)
        self.log('Pure Pursuit 參數已發送：Lookahead Ratio={:.2f}，Min Lookahead={:.2f}m。'.format(
            ratio,minimum))

    def check_nav_twist(self):
        if not self.is_running('nav_preview'):self.log('先啟動 Pure Pursuit 預覽。');return
        self.async_task('Pure Pursuit Twist 預覽',lambda:run_cmd(['timeout','6s','rostopic','echo','-n','1','/offline_nav/twist_cmd'],timeout=8,ros=True)[1][-700:] or '無訊息')

    def stop_nav_preview(self):
        for role in ('nav_filter_cfg','nav_cfg','nav_preview','nav_loader','nav_bridge'):
            if self.is_running(role):self.stop(role)
        self.nav_note.setText('已要求停止導航預覽。車體控制未連接此流程。')

    def stop_all(self):
        self.stop_nav_preview()
        self.stop_pose_capture()
        for role in ('ndt_rviz','ndt_rviz_mapping','ndt_rviz_matching','pose_viz','mapping','matching','map_publisher','live_points','live_tf','save_map'):
            if self.is_running(role):self.stop(role)
        super().stop_all()

    def show_snapshot(self,x):
        super().show_snapshot(x)
        self.global_status.setText(self.global_status.text()+'  |  NDT：'+
            ('Mapping' if self.is_running('mapping') else 'Matching' if self.is_running('matching') else 'idle'))

    def tick(self):
        # The stock poller expects readable stdout; CSV recorder writes to file.
        super().tick()
        if 'csv_pose' in self.proc and self.proc['csv_pose'].poll() is not None:
            p=self.proc.pop('csv_pose')
            try:p.stderr.close()
            except Exception:pass
            if hasattr(self,'csv_file') and not self.csv_file.closed:self.csv_file.close()

    def closeEvent(self,event):
        # Stop algorithm nodes and recorder before stock cleanup.
        super().closeEvent(event)
        if event.isAccepted():
            if hasattr(self,'csv_file') and not self.csv_file.closed:self.csv_file.close()



# -------- v1.5: shared sensor hub, validated direct CSV route + supervised test --------
from route_safety import load_route, validate


class OnlineGUIV15(OnlineGUIV13):
    """Adds hold-to-run supervised route testing without touching Phase 1/2."""

    def __init__(self):
        self.nav_holding = False
        self.nav_route_validated = False
        self.nav_hardware = False
        self.nav_last_poll = 0.0
        super().__init__()
        self.hold_timer = QTimer(self)
        self.hold_timer.setInterval(100)
        self.hold_timer.timeout.connect(self.send_nav_heartbeat)
        QApplication.instance().applicationStateChanged.connect(self.nav_application_state)
        self.log('v1.5：Waypoint 直接載入 CSV → RViz 路徑；實車僅限有監督之按住運行測試。')

    def page_navigation(self):
        w=QWidget();l=QVBoxLayout(w)
        g=self.group('04 Waypoint / Pure Pursuit（線上路徑）');l.addWidget(g)
        g.layout().addWidget(QLabel('先在 03 建立穩定 NDT 定位，再選 CSV；路徑檢查只採 XY / yaw，不以 Z 高差阻擋。'))
        self.nav_csv=QLineEdit(str(self.settings.value('nav_input_csv',os.path.join(HOME,'hunter_online_gui','trajectories','ndt_matching_pose.csv'))))
        self.nav_path=QLineEdit(str(self.settings.value('nav_waypoint_csv',os.path.join(HOME,'hunter_online_gui','trajectories','hunter_waypoints.csv'))))
        form=QFormLayout();g.layout().addLayout(form)
        form.addRow('NDT 整理後 CSV',self.nav_csv)
        form.addRow('Waypoint CSV',self.nav_path)
        row=QHBoxLayout();g.layout().addLayout(row)
        row.addWidget(QLabel('取樣間距 m'))
        self.nav_spacing=QDoubleSpinBox();self.nav_spacing.setRange(.05,5);self.nav_spacing.setValue(.5)
        row.addWidget(self.nav_spacing)
        row.addWidget(QLabel('CSV 速度 km/h（僅生成檔案）'))
        self.nav_speed=QDoubleSpinBox();self.nav_speed.setRange(.1,18.0);self.nav_speed.setValue(1.0)
        row.addWidget(self.nav_speed)
        row.addWidget(self.btn('產生 Waypoint CSV',self.generate_nav_csv))
        row=QHBoxLayout();g.layout().addLayout(row)
        row.addWidget(self.btn('檢查 CSV / XY 路徑',self.validate_nav_csv))
        row.addWidget(self.btn('載入 Waypoint',self.load_waypoints))
        row.addWidget(self.btn('啟動 Pure Pursuit',self.start_nav_preview))
        row.addWidget(self.btn('停止路徑預覽',self.stop_nav_preview))
        row=QHBoxLayout();g.layout().addLayout(row)
        row.addWidget(self.btn('開啟導航 RViz',self.open_nav_rviz))
        row.addWidget(self.btn('檢查路徑與 Twist',self.diagnose_route))
        self.nav_note=QLabel('尚未載入路徑。Preview 仍不會連線 /cmd_vel。')
        self.nav_note.setWordWrap(True);g.layout().addWidget(self.nav_note)

        g=self.group('受監督低速路徑測試（每次只允許一位操作員）');l.addWidget(g)
        g.layout().addWidget(QLabel('先用 Dry Run 驗證：NDT、Waypoint、控制輸出、LiDAR 障礙候選檢查都正常。'
            '\n目標巡航速度在開始前設定；實際輸出仍會依定位、障礙距離、轉彎與加速度調整；車端須為 AUTO。'
            '\n操作員必須一直按住按鈕，放開後本輪測試結束。此機制非安全認證。'))
        row=QHBoxLayout();g.layout().addLayout(row)
        row.addWidget(self.btn('① Dry Run（不控制 Hunter）',lambda:self.arm_nav(False)))
        row.addWidget(self.btn('② 準備實車路徑測試',lambda:self.arm_nav(True)))
        row.addWidget(self.btn('停止導航監督節點',self.stop_hardware_nav,danger=True))
        limitrow=QHBoxLayout();g.layout().addLayout(limitrow)
        limitrow.addWidget(QLabel('NDT fitness score 上限（必須依實測校準）'))
        self.nav_ndt_score_limit=QDoubleSpinBox();self.nav_ndt_score_limit.setDecimals(2)
        self.nav_ndt_score_limit.setRange(0.10,10.00);self.nav_ndt_score_limit.setValue(2.0)
        limitrow.addWidget(self.nav_ndt_score_limit)
        self.nav_fail_safe_check=QCheckBox('已實測底盤失聯停車、實體急停、遙控接管，且場地已清空')
        self.nav_fail_safe_check.setChecked(False)
        g.layout().addWidget(self.nav_fail_safe_check)
        self.nav_hold_button=QPushButton('按住：沿 Waypoint 低速行駛（放開即停）')
        self.nav_hold_button.setObjectName('danger')
        self.nav_hold_button.pressed.connect(self.hold_nav_press)
        self.nav_hold_button.released.connect(self.hold_nav_release)
        self.nav_hold_button.setEnabled(False)
        g.layout().addWidget(self.nav_hold_button)
        self.nav_safety_label=QLabel('尚未啟動導航監督節點。')
        self.nav_safety_label.setWordWrap(True)
        g.layout().addWidget(self.nav_safety_label)
        g.layout().addWidget(QLabel('注意：目前尚無經驗證的完整避障／安全停車系統；只適合封閉、清空、有人持續監控的測試區。'
            '\n中止後要重新啟動 supervisor，且必須重新確認起點位置。'))
        l.addStretch()
        return self.scroll_page(w)

    def validate_nav_csv(self):
        path=self.vpath(self.nav_path)
        if not os.path.isfile(path):
            self.nav_route_validated=False
            self.log('找不到 Waypoint CSV：'+path)
            return False
        try:
            data=load_route(path)
            info=validate(data)
        except (ValueError,OSError) as e:
            self.nav_route_validated=False
            self.nav_note.setText('路徑不可用：'+str(e))
            self.log('路徑安全檢查未通過：'+str(e))
            return False
        self.nav_route_validated=True
        self.nav_route_path=path
        self.nav_note.setText('平面路徑檢查通過：{} 點 / {:.2f} m；'
                              '起點 ({:.2f},{:.2f}), yaw {:.2f}rad。Z 高差不作為阻擋條件。'.format(
                               info['count'],info['length_m'],*info['start']))
        self.log('路徑驗證通過：'+self.nav_note.text())
        return True

    def load_waypoints(self):
        if not self.is_running('matching'):
            self.log('請先啟動 Live NDT Matching。');return
        if self.is_running('nav_bridge') or self.is_running('nav_preview'):
            self.log('若要更換路徑，請先停止舊的導航預覽和監督節點。');return
        if not self.validate_nav_csv():return
        bridge=os.path.join(BUNDLE,'offline_nav_bridge.py')
        args=['python2','-u',bridge,'--csv',self.nav_route_path]
        if hasattr(self,'nav_cruise_speed'):
            args+=['--cruise-mps',str(self.applied_nav_speed())]
        if hasattr(self,'nav_lookahead_ratio'):
            ratio,minimum=self.pure_pursuit_follow_values()
            args+=['--lookahead-ratio',str(ratio),'--min-lookahead',str(minimum)]
        if self.start_proc('nav_bridge',args):
            self.nav_loaded_path=self.nav_route_path
            self.nav_loaded_cruise_mps=(self.nav_cruise_speed.value() if hasattr(self,'nav_cruise_speed') else None)
            self.nav_note.setText('路徑已送出：/offline_nav/final_waypoints + /hunter_nav/route；請開啟導航 RViz。')
            self.save_live_settings()

    def open_nav_rviz(self):
        path=os.path.join(BUNDLE,'hunter_nav.rviz')
        if not os.path.isfile(path):self.log('找不到導航 RViz：'+path);return
        for role in ('ndt_rviz_mapping','ndt_rviz_matching','ndt_rviz_navigation','rviz'):
            if self.is_running(role):
                self.stop(role)
                QTimer.singleShot(900,self.open_nav_rviz)
                return
        self.start_when_master('ndt_rviz_navigation',['rosrun','rviz','rviz','-d',path])
        self.log('導航 RViz：綠色剩餘路徑、Waypoint (X,Y) 編號、黃色 NDT 軌跡、PCD 和 VLP-16。')

    def diagnose_route(self):
        def work():
            topics=('/offline_nav/final_waypoints','/offline_nav/twist_raw','/offline_nav/twist_cmd',
                    '/hunter_nav/route','/ndt_pose')
            rc,output=run_cmd(['rostopic','list'],timeout=7,ros=True)
            if rc:return '無法讀 ROS Topics：'+output[-300:]
            names=set(output.splitlines())
            s=' / '.join('{}={}'.format(x,'OK' if x in names else 'MISSING') for x in topics)
            rc,data=run_cmd(['timeout','5s','rostopic','echo','-n','1','/offline_nav/twist_cmd'],timeout=7,ros=True)
            return s+'\nTwist Preview：'+(data[-450:] if rc in (0,124) else '無有效輸出 '+data[-220:])
        self.async_task('導航路徑診斷',work)

    def arm_nav(self,hardware):
        if self.is_running('nav_supervisor') or self.is_running('nav_gate'):
            self.log('已有監督節點，請先停止再重啟。');return
        if any(self.is_running(x) for x in ('motion','zero')):
            self.log('人工控制腳本仍在執行，禁止導航。');return
        for role in ('hunter','lidar','matching','nav_bridge','nav_preview'):
            if not self.is_running(role):
                self.log('無法啟動監督節點：{} 尚未在本 GUI 運作。'.format(role));return
        if not self.validate_nav_csv():return
        if getattr(self,'nav_loaded_path',None)!=self.nav_route_path:
            self.log('檔案路徑與已載入的路徑不同，請停止路徑預覽後重新載入 CSV。');return
        self.async_task('導航 /cmd_vel 前置檢查',lambda:self.motion_preflight(),
                        lambda status:self._finish_arm_nav(hardware,status))

    def _finish_arm_nav(self,hardware,status):
        if status!='OK':
            self.log('導航啟動取消：'+status);return
        if hardware:
            if not self.nav_fail_safe_check.isChecked():
                self.log('必須先驗證底盤失聯停車、實體急停與遙控接管，並勾選確認。')
                return
            status_cmd=run_cmd(['rostopic','echo','-n','1','/hunter_status'],timeout=5,ros=True)
            raw=status_cmd[1]
            m=re.search(r'^control_mode:\s*(\d+)',raw,re.M)
            if status_cmd[0]!=0 or m is None or int(m.group(1))!=1:
                self.log('Hunter 尚未顯示 control_mode=1（AUTO），請使用官方遙控程序切換後重新檢查。')
                return
            msg=('本模式會向實體 Hunter 發布 /cmd_vel；最高速度由導航頁設定，並依 NDT 自適應降速。\n'
                 '必須已測試獨立實體急停、車體命令失聯停車、遙控接管；測試區完全清空且有人監控。\n'
                 'VLP-16 前方檢測不是安全認證避障系統。\n'
                 '路徑起點與現在車體必須對齊，否則不會行駛。\n\n啟用實車模式？')
            # Simplified operation: one START click after independent physical
            # safety verification; no redundant typed confirmation. The actual
            # supervisor safety interlocks are NOT bypassed.
            if not getattr(self,'simple_request',False):
                if QMessageBox.warning(self,'實車路徑測試授權',msg,
                     QMessageBox.Yes|QMessageBox.No,QMessageBox.No)!=QMessageBox.Yes:return
                typed,ok=QInputDialog.getText(self,'第二次確認','請輸入 HUNTER TEST 以啟用實車命令：')
                if not ok or typed!='HUNTER TEST':return
        supervisor=os.path.join(BUNDLE,'hunter_nav_supervisor.py')
        args=['python2','-u',supervisor,'--max-ndt-score',str(self.nav_ndt_score_limit.value())]
        model=self.hunter_protocol_model or 'hunter1'
        args+=['--hunter-model',model]
        self.log('Hunter 導航幾何：{}（{}）'.format(model,'驅動自動偵測' if self.hunter_protocol_model else '本車 AGX_V1 預設'))
        if hasattr(self,'nav_cruise_speed'):
            args+=['--max-speed',str(self.applied_nav_speed())]
        if hasattr(self,'nav_vehicle_width'):
            args+=['--vehicle-width-m',str(self.nav_vehicle_width.value())]
        if hasattr(self,'nav_corridor_half_width'):
            args+=['--corridor-half-width-m',str(self.nav_corridor_half_width.value())]
        if hasattr(self,'nav_cross_track_limit'):
            args+=['--max-cross-track-m',str(self.nav_cross_track_limit.value())]
        if hasattr(self,'nav_obstacle_stop_distance'):
            args+=['--obstacle-stop-m',str(self.nav_obstacle_stop_distance.value())]
        args+=['--pose-timeout','1.0','--min-pose-hz','2.0']
        if hasattr(self,'nav_lookahead_ratio'):
            ratio,minimum=self.pure_pursuit_follow_values()
            args+=['--lookahead-ratio',str(ratio),'--min-lookahead',str(minimum)]
        if hasattr(self,'nav_recovery_enable') and self.nav_recovery_enable.isChecked():
            args+=['--recovery-enable']
            for key,widget in self.nav_recovery_fields.items():
                args+=['--recovery-'+key,str(widget.value())]
            self.log('Recovery：已啟用自動後退脫困（彎道障礙／車頭偏差／有效預瞄點失效）；後方障礙／未知格時自動停車等待。')
        if hardware:args.append('--hardware')
        if not self.start_proc('nav_supervisor',args,hunter=True):return
        script=os.path.join(BUNDLE,'hunter_nav_operator_gate.py')
        try:
            proc=subprocess.Popen(ros_cmd(['python2','-u',script],hunter=True),
                   stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                   start_new_session=True,bufsize=0)
            os.set_blocking(proc.stdout.fileno(),False)
            self.proc['nav_gate']=proc
        except OSError as e:
            self.log('啟動 operator gate 失敗：'+str(e))
            self.stop('nav_supervisor')
            return
        self.nav_hardware=hardware
        self.nav_hold_button.setEnabled(True)
        self.nav_hold_button.setText('按住：'+('實車低速沿路徑行駛' if hardware else 'Dry Run 產生測試命令')+'；放開即停止')
        self.nav_safety_label.setText('監督節點已啟動（{}）。等待感測器與定位檢查。'
                                      .format('實車模式' if hardware else 'Dry Run'))
        if getattr(self,'simple_request',False) and hardware:
            self.nav_holding=True
            self.send_nav_heartbeat()
            self.hold_timer.start()
            self.nav_safety_label.setText('一鍵導航啟動中：背景心跳自動運作；正在確認定位、路徑、車體模式與光達。')
            self.log('一鍵導航：背景心跳已建立；等待 NDT／Hunter／光達等自動檢查。')
        else:
            self.log('導航已設定 {}；操作員須持續按住按鈕。'.format('實車' if hardware else '無馬達 Dry Run'))

    def send_nav_heartbeat(self):
        if not self.nav_holding:return
        p=self.proc.get('nav_gate')
        if p is None or p.poll() is not None or not self.is_running('nav_supervisor'):
            self.hold_nav_release()
            self.log('operator gate / supervisor 失去連線，已撤回操作授權。')
            return
        try:
            p.stdin.write(b'H');p.stdin.flush()
        except (OSError,ValueError):
            self.hold_nav_release()
            self.log('operator gate 管道中斷，已撤回操作授權。')

    def hold_nav_press(self):
        if not self.is_running('nav_gate') or not self.is_running('nav_supervisor'):
            self.log('請先啟動導航監督節點。');return
        if self.nav_holding:return
        self.nav_holding=True
        self.nav_release_logged=False
        self.send_nav_heartbeat()
        self.hold_timer.start()
        self.log('操作員按住授權鍵，監督節點仍需檢查所有前置條件才會運作。')

    def hold_nav_release(self):
        if hasattr(self,'hold_timer'):self.hold_timer.stop()
        self.nav_holding=False
        p=self.proc.get('nav_gate')
        if p is not None and p.poll() is None:
            try:p.stdin.write(b'0');p.stdin.flush()
            except (OSError,ValueError):pass
        if getattr(self,'nav_release_logged',False) is False:
            self.log('操作員已放開授權鍵，已撤回命令。')
        self.nav_release_logged=True

    def nav_application_state(self,state):
        if state!=Qt.ApplicationActive and self.nav_holding:
            self.hold_nav_release()

    def stop_hardware_nav(self):
        self.hold_nav_release()
        if hasattr(self,'nav_hold_button'):
            self.nav_hold_button.setEnabled(False)
        for role in ('nav_gate','nav_supervisor'):
            if self.is_running(role):self.stop(role)
        if hasattr(self,'nav_safety_label'):self.nav_safety_label.setText('監督節點已停止；請確認 Hunter 車體已實際停車。')

    def _safety_result(self,result):
        if hasattr(self,'nav_safety_label'):
            self.nav_safety_label.setText('導航監督狀態：'+result[-500:])

    def tick(self):
        super().tick()
        if self.nav_holding and not (self.is_running('nav_gate') and self.is_running('nav_supervisor')):
            self.hold_nav_release()
        if self.is_running('nav_supervisor') and time.monotonic()-self.nav_last_poll>2.0:
            self.nav_last_poll=time.monotonic()
            self.async_task('導航監督狀態',lambda:run_cmd(
                ['rostopic','echo','-n','1','/hunter_nav/safety_state'],timeout=4,ros=True)[1],
                self._safety_result)

    def stop_nav_preview(self):
        self.stop_hardware_nav()
        super().stop_nav_preview()

    def stop_all(self):
        self.stop_hardware_nav()
        if self.is_running('ndt_rviz_navigation'):self.stop('ndt_rviz_navigation')
        super().stop_all()

    def soft_stop(self):
        self.stop_hardware_nav()
        super().soft_stop()

    def closeEvent(self,event):
        self.stop_hardware_nav()
        super().closeEvent(event)

class OnlineGUISimple(OnlineGUIV15):
    # User-selectable CRUISE TARGET is distinct from achievable speed.
    # No fixed 0.30 m/s GUI clamp; Supervisor applies dynamic guards.
    MAX_TARGET_MPS = 5.00

    def pure_pursuit_follow_values(self):
        """The two parameters used by the original offline Pure Pursuit GUI."""
        return (float(self.nav_lookahead_ratio.value()),
                float(self.nav_min_lookahead.value()))

    def use_recommended_pure_pursuit(self):
        # Existing saved settings are not silently overwritten on upgrade.
        # One deliberate click selects the comparison configuration.
        if self.simple_request or self.is_running('nav_supervisor'):
            self.log('實車導航正在執行，先停止後才能變更 Pure Pursuit 測試參數。')
            return
        self.nav_lookahead_ratio.setValue(1.17)
        self.nav_min_lookahead.setValue(2.00)
        self.apply_pure_pursuit_settings()
        self.log('已選用建議比較值：Lookahead Ratio=1.17，Min Lookahead=2.00m。')

    def save_pure_pursuit_settings(self, _value=None):
        ratio,minimum=self.pure_pursuit_follow_values()
        self.settings.setValue('nav_lookahead_ratio',ratio)
        self.settings.setValue('nav_min_lookahead',minimum)

    def apply_pure_pursuit_settings(self):
        # Editing/republishing during live motor control would abruptly change
        # commanded curvature, so require STOP first. Preview may be updated.
        if self.simple_request or self.is_running('nav_supervisor'):
            self.log('請先按「停止自主導航」，再變更／套用 Pure Pursuit 參數。')
            return
        self.save_pure_pursuit_settings()
        self.settings.sync()
        ratio,minimum=self.pure_pursuit_follow_values()
        if self.is_running('nav_preview'):
            self.publish_nav_config()
            self.log('已要求更新現有 Pure Pursuit 節點（僅預覽／停車狀態）。')
        else:
            self.log('已儲存 Lookahead Ratio={:.2f}、Min Lookahead={:.2f}m；'
                     '下次開始導航時會自動套用。'.format(ratio,minimum))

    def applied_nav_speed(self):
        if not hasattr(self,'nav_cruise_speed'):
            return 0.10
        return float(self.nav_cruise_speed.value())

    def nav_speed_label_changed(self, value):
        self.settings.setValue('nav_cruise_mps',float(value))
        if hasattr(self,'nav_speed_limit_info'):
            self.nav_speed_limit_info.setText(
                '已設定目標巡航 {:.2f} m/s（{:.1f} km/h）；啟動後鎖定。'
                ' 實際速度由 NDT FPS／運算時間、前方可用距離、彎道曲率與平順加速共同限制。'
                ' 速度不是車型最高速度保證；高速測試前必須驗證煞車及感測距離。'
                .format(float(value),float(value)*3.6))

    """Single-entry-point route request; preserves independent Supervisor interlocks.

    START requests autonomy, and automatically loads CSV / preview / supervisor.
    The vehicle only receives nonzero /cmd_vel after the Supervisor confirms
    continuous NDT, route alignment, AUTO mode, obstacle and watchdog checks.
    STOP or GUI shutdown revokes the operator gate and sends a zero command.
    """
    def __init__(self):
        self.simple_request=False
        self.simple_request_t0=0.0
        self.simple_arm_sent=False
        super().__init__()
        self.setWindowTitle('HUNTER / AUTOWARE-LITE ONLINE — 可調路徑走廊半寬 v1.5.18')
        self.log('v1.5.12：自動比對 ROS 多筆 Hunter 模式與 CAN 0x151 原始回報；保留實車安全互鎖。')
        self.log('v1.5.13：路徑障礙走廊改為車體實際最外寬 + 左右各0.15m；請先量測並核對RViz。')
        self.log('v1.5.14：偏離路徑上限可設為1.00m；同時記錄完整/剩餘偏差、WP進度與曲率。')
        self.log('v1.5.15：側牆回波依路徑分類；前方停車距離可由 GUI 設定，車體近前方仍防護。')
        self.log('v1.5.17：修正啟動點在 WP01 前方時進度卡住；附加進度停滯停車保護。')
        self.log('v1.5.18：Route Corridor Half Width 可由 GUI 調整；最低為車寬/2 + 0.15m。')

    def page_navigation(self):
        page=super().page_navigation()
        # Hide now-redundant manual activation and Dry Run controls. We keep
        # diagnostics, route editing and RViz available for troubleshooting.
        for btn in page.findChildren(QPushButton):
            name=btn.text()
            if 'Dry Run' in name or '準備實車路徑測試' in name or '按住：' in name:
                btn.hide()
            if name=='停止導航監督節點':
                btn.hide()
            if name in ('載入 Waypoint','啟動 Pure Pursuit','停止路徑預覽'):
                btn.hide()
        for lab in page.findChildren(QLabel):
            if '先用 Dry Run 驗證' in lab.text():
                lab.setText('一鍵導航：自動載入 Waypoint、啟動 Pure Pursuit，並檢查 NDT、Hunter AUTO、前方光達與指令來源。\n'
                            '軟體僅為研究測試，沒有安全認證；必須先完成實體急停、遙控接管與命令失聯停車驗證。')
            if '中止後要重新啟動 supervisor' in lab.text():
                lab.setText('安全條件異常會停車；重新開始前必須檢查故障原因、定位與車輛狀態。')
        # The supervisor safety check box remains visible and unchecked each run.
        buttons=QHBoxLayout()
        self.simple_start_btn=QPushButton('開始自主導航')
        self.simple_start_btn.clicked.connect(self.simple_start)
        self.simple_stop_btn=QPushButton('停止自主導航')
        self.simple_stop_btn.setObjectName('danger')
        self.simple_stop_btn.clicked.connect(self.simple_stop)
        buttons.addWidget(self.simple_start_btn)
        buttons.addWidget(self.simple_stop_btn)
        # Place the high-priority controls at the top of the page, ahead of
        # optional CSV controls. scroll_page contains a QWidget -> layout.
        body=page.widget()
        body.layout().insertLayout(0,buttons)
        speedrow=QHBoxLayout()
        speedrow.addWidget(QLabel('目標巡航速度 (m/s，啟動前設定)'))
        self.nav_cruise_speed=QDoubleSpinBox()
        self.nav_cruise_speed.setRange(0.05,5.00)
        self.nav_cruise_speed.setDecimals(2)
        self.nav_cruise_speed.setSingleStep(0.05)
        # Never preselect a potentially high cruise target after a restart.
        # Require the operator to consciously set the desired speed each run.
        self.nav_cruise_speed.setValue(0.10)
        self.nav_cruise_speed.valueChanged.connect(self.nav_speed_label_changed)
        speedrow.addWidget(self.nav_cruise_speed)
        speedrow.addWidget(QLabel('慢 NDT 自動降速；定位逾時 1 秒停；連續 ≥2 Hz 恢復；障礙淨空 1.2 秒恢復'))
        body.layout().insertLayout(1,speedrow)
        self.nav_speed_limit_info=QLabel()
        self.nav_speed_limit_info.setWordWrap(True)
        body.layout().insertWidget(2,self.nav_speed_limit_info)
        self.nav_speed_label_changed(self.nav_cruise_speed.value())

        # The original offline GUI exposed these two Pure Pursuit parameters.
        # Restore matching ranges/defaults and apply to the same ROS config
        # topic used by the online controller.
        followrow=QHBoxLayout()
        followrow.addWidget(QLabel('Pure Pursuit — Lookahead Ratio'))
        self.nav_lookahead_ratio=QDoubleSpinBox()
        self.nav_lookahead_ratio.setRange(0.1,20.0)
        self.nav_lookahead_ratio.setDecimals(2)
        self.nav_lookahead_ratio.setSingleStep(0.1)
        self.nav_lookahead_ratio.setValue(float(self.settings.value('nav_lookahead_ratio',1.17)))
        self.nav_lookahead_ratio.setToolTip('預瞄比例；本次比較建議 1.17，仍保留原本儲存設定。')
        followrow.addWidget(self.nav_lookahead_ratio)
        followrow.addWidget(QLabel('Min Lookahead (m)'))
        self.nav_min_lookahead=QDoubleSpinBox()
        self.nav_min_lookahead.setRange(0.1,30.0)
        self.nav_min_lookahead.setDecimals(2)
        self.nav_min_lookahead.setSingleStep(0.1)
        self.nav_min_lookahead.setValue(float(self.settings.value('nav_min_lookahead',2.0)))
        self.nav_min_lookahead.setToolTip('最小預瞄距離；本次比較建議 2.00m，仍保留原本儲存設定。')
        followrow.addWidget(self.nav_min_lookahead)
        self.nav_apply_lookahead_btn=QPushButton('套用 Pure Pursuit 參數')
        self.nav_apply_lookahead_btn.clicked.connect(self.apply_pure_pursuit_settings)
        followrow.addWidget(self.nav_apply_lookahead_btn)
        self.nav_recommended_lookahead_btn=QPushButton('套用建議測試值 1.17 / 2.00m')
        self.nav_recommended_lookahead_btn.clicked.connect(self.use_recommended_pure_pursuit)
        followrow.addWidget(self.nav_recommended_lookahead_btn)
        body.layout().insertLayout(3,followrow)
        self.nav_lookahead_ratio.valueChanged.connect(self.save_pure_pursuit_settings)
        self.nav_min_lookahead.valueChanged.connect(self.save_pure_pursuit_settings)
        self.nav_follow_note=QLabel('參數會在按「開始自主導航」時自動套用；'
                                    '若 Pure Pursuit 預覽已運作，可按「套用」重新發布。實車導航中請先停止再修改。')
        self.nav_follow_note.setWordWrap(True)
        body.layout().insertWidget(4,self.nav_follow_note)
        widthrow=QHBoxLayout()
        widthrow.addWidget(QLabel('車體最外寬 (m，含輪胎／外掛設備，請實際量測)'))
        self.nav_vehicle_width=QDoubleSpinBox()
        self.nav_vehicle_width.setRange(0.60,2.50)
        self.nav_vehicle_width.setDecimals(2)
        self.nav_vehicle_width.setSingleStep(0.02)
        # This is a placeholder, NOT an independently verified Hunter width.
        self.nav_vehicle_width.setValue(float(self.settings.value('nav_vehicle_width_m',0.80)))
        self.nav_vehicle_width.setToolTip('請量測實際車體含輪胎/外掛設備最寬處。0.80m 僅是未核實的預設值。')
        self.nav_vehicle_width.valueChanged.connect(self.update_nav_corridor_note)
        widthrow.addWidget(self.nav_vehicle_width)
        widthrow.addWidget(QLabel('Route Corridor Half Width (m)'))
        self.nav_corridor_half_width=QDoubleSpinBox()
        self.nav_corridor_half_width.setDecimals(2)
        self.nav_corridor_half_width.setSingleStep(0.05)
        self.nav_corridor_half_width.setRange(round(self.nav_vehicle_width.value()/2.0+0.15,2),1.75)
        self.nav_corridor_half_width.setValue(float(self.settings.value(
            'nav_corridor_half_width_m',self.nav_vehicle_width.value()/2.0+0.15)))
        self.nav_corridor_half_width.setToolTip(
            '藍色路徑走廊中心線到單側邊界距離。最小值=量測車寬/2+0.15m；'
            '不等於牆面淨空或車體轉彎掃掠。開始導航後鎖定。')
        self.nav_corridor_half_width.valueChanged.connect(self.save_corridor_half_width)
        widthrow.addWidget(self.nav_corridor_half_width)
        body.layout().insertLayout(5,widthrow)
        self.nav_corridor_note=QLabel('')
        self.nav_corridor_note.setWordWrap(True)
        body.layout().insertWidget(6,self.nav_corridor_note)
        self.update_nav_corridor_note(self.nav_vehicle_width.value())
        devrow=QHBoxLayout()
        devrow.addWidget(QLabel('偏離路徑停車門檻 (m；依剩餘路徑)'))
        self.nav_cross_track_limit=QDoubleSpinBox()
        self.nav_cross_track_limit.setRange(0.30,1.00)
        self.nav_cross_track_limit.setDecimals(2)
        self.nav_cross_track_limit.setSingleStep(0.05)
        self.nav_cross_track_limit.setValue(1.00)
        self.nav_cross_track_limit.setToolTip('本次設定 1.00m，從原本 0.45m 放寬；停車保護不是牆面淨空。啟動後鎖定。')
        self.nav_cross_track_limit.valueChanged.connect(self.save_cross_track_limit)
        devrow.addWidget(self.nav_cross_track_limit)
        devrow.addWidget(QLabel('超限停車；完整/剩餘路徑偏差會同時記錄。窄走廊仍需確認實際車體碰撞空間。'))
        body.layout().insertLayout(7,devrow)
        obstacle_row=QHBoxLayout()
        obstacle_row.addWidget(QLabel('前方障礙停車距離 (m；啟動前設定)'))
        self.nav_obstacle_stop_distance=QDoubleSpinBox()
        self.nav_obstacle_stop_distance.setRange(1.50,10.00)
        self.nav_obstacle_stop_distance.setDecimals(2)
        self.nav_obstacle_stop_distance.setSingleStep(0.10)
        self.nav_obstacle_stop_distance.setValue(float(self.settings.value('nav_obstacle_stop_m',2.40)))
        self.nav_obstacle_stop_distance.setToolTip('預設 2.40m；僅可在停止導航時調整，實際值可能被速度對應煞停模型提高。牆面若進入車體或路徑範圍仍需停止。')
        self.nav_obstacle_stop_distance.valueChanged.connect(self.save_obstacle_stop_distance)
        obstacle_row.addWidget(self.nav_obstacle_stop_distance)
        obstacle_row.addWidget(QLabel('側牆在路徑外以橘色提示；車體近前方／行駛走廊內仍停車'))
        body.layout().insertLayout(8,obstacle_row)
        self.nav_obstacle_stop_note=QLabel('')
        self.nav_obstacle_stop_note.setWordWrap(True)
        body.layout().insertWidget(9,self.nav_obstacle_stop_note)
        self.nav_cruise_speed.valueChanged.connect(lambda _:self.update_obstacle_stop_note())
        self.update_obstacle_stop_note()
        resetrow=QHBoxLayout()
        self.nav_reset_progress_btn=QPushButton('重設 Waypoint 進度／從起點重跑（停車時）')
        self.nav_reset_progress_btn.clicked.connect(self.reset_waypoint_progress)
        resetrow.addWidget(self.nav_reset_progress_btn)
        body.layout().insertLayout(10,resetrow)
        mode_row=QHBoxLayout()
        self.nav_mode_diag_btn=QPushButton('自動診斷 Hunter 控制模式（只讀）')
        self.nav_mode_diag_btn.clicked.connect(self.diagnose_hunter_auto_mode)
        mode_row.addWidget(self.nav_mode_diag_btn)
        self.nav_mode_diag_label=QLabel('AUTO 訊號以 Hunter Driver 與 CAN 0x151 即時回報比對；診斷不會讓車移動。')
        self.nav_mode_diag_label.setWordWrap(True)
        mode_row.addWidget(self.nav_mode_diag_label)
        body.layout().insertLayout(11,mode_row)
        return page

    def save_cross_track_limit(self, value):
        self.settings.setValue('nav_cross_track_limit_m',float(value))
        if hasattr(self,'nav_cross_track_limit'):
            self.log('已設定偏離路徑停車門檻 {:.2f}m；下次開始導航時生效。'.format(float(value)))

    def save_obstacle_stop_distance(self, value):
        self.settings.setValue('nav_obstacle_stop_m',float(value))
        self.update_obstacle_stop_note()
        self.log('前方障礙停車距離設為 {:.2f}m；下次開始導航時生效。'.format(float(value)))

    def update_obstacle_stop_note(self):
        if not hasattr(self,'nav_obstacle_stop_note'):
            return
        from nav_motion_policy import required_clearance
        requested=float(self.nav_obstacle_stop_distance.value())
        speed=float(self.nav_cruise_speed.value())
        model_floor=required_clearance(speed,margin_m=1.0)
        effective=max(requested,model_floor)
        self.nav_obstacle_stop_note.setText(
            '設定 {:.2f}m；依巡航上限 {:.2f}m/s 的簡化反應/煞停模型，實際生效至少 {:.2f}m。'
            '此數值不是安全認證煞停距離；不會忽略侵入車體或轉彎掃掠範圍的牆面。'
            '請確認 LiDAR→base_link 外參及實際可通行淨空。'.format(requested,speed,effective))

    def save_corridor_half_width(self, half_width_m):
        half_width_m=float(half_width_m)
        self.settings.setValue('nav_corridor_half_width_m',half_width_m)
        self.update_nav_corridor_note(self.nav_vehicle_width.value())
        self.log('Route Corridor Half Width 設為 {:.2f}m；下次啟動導航生效。'.format(half_width_m))

    def update_nav_corridor_note(self, vehicle_width_m):
        vehicle_width_m=float(vehicle_width_m)
        self.settings.setValue('nav_vehicle_width_m',vehicle_width_m)
        minimum=round(vehicle_width_m/2.0+0.15,2)
        if hasattr(self,'nav_corridor_half_width'):
            # Constrain even persisted settings after changing measured width;
            # the spinbox will emit valueChanged and update QSettings if raised.
            self.nav_corridor_half_width.setMinimum(minimum)
            half=self.nav_corridor_half_width.value()
            self.settings.setValue('nav_corridor_half_width_m',half)
        else:
            half=minimum
        if hasattr(self,'nav_corridor_note'):
            self.nav_corridor_note.setText(
                '藍色 Route Corridor：Half Width={:.2f}m，總寬={:.2f}m；'
                '車寬={:.2f}m，每側額外保留 {:.2f}m（下限0.15m）。'
                '紅色=走廊內候選，橘色=走廊外候選；此 ROI 並非完整的車體掃掠/防撞認證。'
                '請在停止狀態下核對 RViz、光達外參與牆面實際淨空。'.format(
                    half,2.0*half,vehicle_width_m,half-vehicle_width_m/2.0))

    def reset_waypoint_progress(self):
        if self.simple_request or self.is_running('nav_supervisor') or self.is_running('nav_gate'):
            self.log('請先按「停止自主導航」後，才可以重設 Waypoint 進度。')
            return
        from nav_waypoint_progress import reset_progress
        try:
            csv_path=self.vpath(self.nav_path)
            points=load_route(csv_path)
            validate(points)
            # Stop the bridge before changing the persisted progress, otherwise
            # its in-memory tracker would overwrite the reset state.
            if self.is_running('nav_bridge') or self.is_running('nav_preview'):
                self.stop_nav_preview()
            reset_progress(csv_path,points)
            self.nav_loaded_path=None
            self.log('Waypoint 進度已重設；下一次按「開始自主導航」會從 WP01 檢查起點。')
        except (OSError,ValueError) as exc:
            self.log('重設 Waypoint 進度失敗：'+str(exc))

    def diagnose_hunter_auto_mode(self):
        self.nav_mode_diag_btn.setEnabled(False)
        self.nav_mode_diag_label.setText('正在被動讀取 /hunter_status 與 CAN 0x151，無車輛命令...')
        self.async_task('Hunter AUTO 訊號診斷', check_hunter_mode_report,
                        self._finish_mode_diagnostics)

    def _finish_mode_diagnostics(self,result):
        self.nav_mode_diag_btn.setEnabled(True)
        message=result.get('message','診斷失敗')
        self.nav_mode_diag_label.setText('目前狀態：' + ('CAN 指令模式已確認' if result.get('ok') else '模式未確認／需檢查回報'))
        self.log('Hunter AUTO 自動診斷：'+message.replace('\n',' / '))
        QMessageBox.information(self,'Hunter 模式自動診斷（只讀）', message)

    def _check_hunter_auto_mode(self):
        # Samples actual ROS status and raw CAN; NEVER moves the robot.
        return check_hunter_mode_report()

    def _finish_hunter_auto_mode_check(self, result):
        # STOP may be pressed while the worker is running: never resume a
        # cancelled navigation request from a delayed response.
        if not self.simple_request:
            return
        if not result.get('ok', False):
            message = result.get('message', 'Hunter AUTO 狀態未知')
            self.simple_stop()
            self.nav_note.setText('導航尚未開始：' + message.replace('\n', ' '))
            self.log('啟動前 AUTO 檢查未通過：' + message.replace('\n', ' '))
            QMessageBox.information(self, 'Hunter 控制模式回報尚未確認', message)
            return
        self.simple_request_t0 = time.monotonic()
        self.log('Hunter AUTO=1 經多筆狀態確認，接著檢查 NDT 位姿與 Waypoint 起點。')
        self.async_task('導航起點檢查', self._check_startpoint,
                        self._finish_startpoint_check)

    def _check_startpoint(self):
        """One new NDT pose, route XY and yaw only. Never move the vehicle."""
        helper=os.path.join(BUNDLE,'nav_startpoint_precheck.py')
        rc,out=run_cmd(['python2','-u',helper,'--csv',self.nav_route_path],timeout=10,ros=True)
        for line in reversed(out.splitlines()):
            if line.startswith('NAV_STARTPOINT_RESULT='):
                try:return __import__('json').loads(line.split('=',1)[1])
                except ValueError:break
        return {'ok':False,'type':'error','message':'無法取得最新 NDT 位姿或起點資訊：'+out[-300:]}

    def _finish_startpoint_check(self, result):
        if not self.simple_request:return
        if result.get('ok'):
            if result.get('resume'):
                self.log('接續剩餘路徑：下一個 WP{:02d}/{}；距離剩餘路段 {:.2f}m。'.format(
                    result.get('next_index',0)+1, result.get('waypoint_count',0),
                    result.get('distance_m',0)))
            else:
                self.log('起點檢查通過：距離 {:.2f}m、Yaw 差 {:.2f}rad。'.format(
                    result.get('distance_m',0),result.get('yaw_difference_rad',0)))
            self._simple_continue()
            return
        message=result.get('message','未通過起點檢查')
        if result.get('type')=='too_far' and result.get('resume'):
            message = (message + '\n需靠近剩餘路徑，不會自動跳回已走過的 Waypoint。')
            QMessageBox.information(self,'請移到剩餘 Waypoint 附近',message)
        elif result.get('type')=='too_far':
            message=('目前 Hunter 距離 Waypoint 起點 {:.2f} m，允許距離 {:.2f} m。\n'
                     '請先使用遙控器把 Hunter 移到起點附近，再按「開始自主導航」。\n\n'
                     '目前 NDT：({:.2f}, {:.2f})\n路徑起點：({:.2f}, {:.2f})').format(
                       result['distance_m'],result['limit_m'],
                       result['current_xy'][0],result['current_xy'][1],
                       result['start_xy'][0],result['start_xy'][1])
            QMessageBox.information(self,'請先移到路徑起點',message)
        elif result.get('type')=='yaw_mismatch':
            QMessageBox.information(self,'請調整 Hunter 車頭朝向',message+'\n先手動調整車頭朝向後，再按開始。')
        else:
            QMessageBox.warning(self,'導航尚未就緒',message)
        self.log('本次未啟動導航：'+message.replace('\n',' '))
        self.simple_stop()

    def simple_start(self):
        if self.simple_request or self.is_running('nav_supervisor') or self.is_running('nav_gate'):
            self.log('導航已啟動或正在準備，請勿重複按 Start。')
            return
        if not self.nav_fail_safe_check.isChecked():
            self.log('啟動取消：請先完成實體急停、失聯停車、遙控接管與場地安全驗證，再勾選確認。')
            return
        for role in ('hunter','lidar','matching'):
            if not self.is_running(role):
                self.log('啟動取消：{} 未由此 GUI 啟動；請先到 00／03 啟動。'.format(role))
                return
        if self.is_running('motion') or self.is_running('zero'):
            self.log('啟動取消：仍有人工運動測試程序運作。')
            return
        if not self.validate_nav_csv():
            return
        self.log('巡航目標：{:.2f}m/s（{:.1f}km/h）；實際速度將受定位品質、'
                 '前方可用煞車距離、彎道與加速斜率限制。'.format(
                  self.applied_nav_speed(),self.applied_nav_speed()*3.6))
        # Explicit START always reloads the stored index. This discards the
        # previous bridge's in-memory completion latch without skipping resume
        # checks; NDT/ROS/driver stay alive. No automatic new lap.
        self.nav_reload_pending = self.is_running('nav_bridge') or self.is_running('nav_preview')
        if self.nav_reload_pending:
            self.log('重新載入 Waypoint 進度與 Pure Pursuit（保留 NDT／感測器）。')
            self.stop_nav_preview()
        self.simple_request=True
        self.simple_arm_sent=False
        self.simple_request_t0=time.monotonic()
        self.simple_start_btn.setEnabled(False)
        self.nav_cruise_speed.setEnabled(False)
        self.nav_lookahead_ratio.setEnabled(False)
        self.nav_min_lookahead.setEnabled(False)
        self.nav_vehicle_width.setEnabled(False)
        self.nav_corridor_half_width.setEnabled(False)
        self.nav_cross_track_limit.setEnabled(False)
        self.nav_obstacle_stop_distance.setEnabled(False)
        self.nav_apply_lookahead_btn.setEnabled(False)
        self.nav_recommended_lookahead_btn.setEnabled(False)
        self.log('開始自主導航：自動採樣 Hunter AUTO / CAN 0x151，再檢查 NDT 位姿和 Waypoint；'
                 '不會在 AUTO 未確認時啟動導航控制節點。')
        self.async_task('Hunter AUTO 啟動前檢查', self._check_hunter_auto_mode,
                        self._finish_hunter_auto_mode_check)

    def _simple_continue(self):
        if not self.simple_request or self.simple_arm_sent:
            return
        if time.monotonic()-self.simple_request_t0>16:
            self.log('導航啟動逾時；請檢查 /ndt_pose、Pure Pursuit 與 ROS 日誌。')
            self.simple_stop()
            return
        if not all(self.is_running(role) for role in ('hunter','lidar','matching')):
            self.log('導航準備取消：Hunter、VLP 或 NDT Matching 已停止。')
            self.simple_stop()
            return
        if getattr(self,'nav_reload_pending',False):
            if self.is_running('nav_bridge') or self.is_running('nav_preview'):
                QTimer.singleShot(300,self._simple_continue)
                return
            self.nav_reload_pending=False
        if not self.is_running('nav_bridge'):
            self.load_waypoints()
        elif not self.is_running('nav_preview'):
            self.start_nav_preview()
        else:
            self.simple_arm_sent=True
            self.arm_nav(True)
            return
        QTimer.singleShot(900,self._simple_continue)

    def _finish_arm_nav(self,hardware,status):
        if not self.simple_request:
            self.log('導航啟動程序已取消，忽略較晚返回的前置檢查。')
            return
        if status!='OK':
            self.log('導航啟動取消：'+status)
            self.simple_stop()
            return
        super()._finish_arm_nav(hardware,status)
        if not self.is_running('nav_supervisor'):
            self.simple_stop()

    def simple_stop(self):
        self.simple_request=False
        self.simple_arm_sent=False
        if hasattr(self,'simple_start_btn'):
            self.simple_start_btn.setEnabled(True)
        if hasattr(self,'nav_cruise_speed'):
            self.nav_cruise_speed.setEnabled(True)
        for name in ('nav_lookahead_ratio','nav_min_lookahead','nav_apply_lookahead_btn','nav_recommended_lookahead_btn','nav_vehicle_width','nav_corridor_half_width','nav_cross_track_limit','nav_obstacle_stop_distance'):
            if hasattr(self,name):getattr(self,name).setEnabled(True)
        self.stop_hardware_nav()
        # Important: leave NDT/ROS/sensors running so the operator can inspect
        # failures without re-starting the entire vehicle and pointcloud chain.
        self.log('已要求停止自主導航，請確認 Hunter 實際停車。')

    def stop_hardware_nav(self):
        self.simple_request=False
        if hasattr(self,'simple_start_btn'):
            self.simple_start_btn.setEnabled(True)
        if hasattr(self,'nav_cruise_speed'):
            self.nav_cruise_speed.setEnabled(True)
        for name in ('nav_lookahead_ratio','nav_min_lookahead','nav_apply_lookahead_btn','nav_recommended_lookahead_btn','nav_vehicle_width','nav_corridor_half_width','nav_cross_track_limit','nav_obstacle_stop_distance'):
            if hasattr(self,name):getattr(self,name).setEnabled(True)
        super().stop_hardware_nav()

    def nav_application_state(self,state):
        # Changing focus to RViz must NOT be interpreted as releasing an
        # explicit one-click navigation request; the gate still requires the
        # GUI's 100-ms heartbeat and will time out if the GUI stalls/closes.
        if getattr(self,'simple_request',False) and getattr(self,'nav_hardware',False):
            return
        super().nav_application_state(state)

    def soft_stop(self):
        self.simple_stop()
        super().soft_stop()

    def closeEvent(self,event):
        self.simple_stop()
        super().closeEvent(event)



# v1.6.0: Live recording feasibility visualizer + self-contained CSV statuses.
# This only augments the existing GUI workflow. The ROS preview process creates
# no cmd_vel/CAN publisher, and uses the same pure evaluator as CSV export.
import json as _json
from nav_trajectory_feasibility import (profile_for as _profile_for,
    profile_id as _profile_id, counts as _feasibility_counts)


class OnlineGUIV160(OnlineGUISimple):
    def __init__(self):
        self.feasibility_state_path=None
        self.feasibility_latest_file=None
        self.feasibility_profile=None
        self.feasibility_last_read=0.0
        super().__init__()
        self.setWindowTitle('HUNTER / AUTOWARE-LITE ONLINE — 線上軌跡可行性 v1.6.0')
        self.log('v1.6.0：NDT 錄製時延遲預檢彩色軌跡；CSV 保存每段狀態，04 導航 RViz 重現。僅供研究預檢。')

    def page_localization(self):
        page=super().page_localization()
        body=page.widget().layout()
        group=self.group('錄製中導航可行性預檢（不控制 Hunter）')
        self.feasibility_enabled=QCheckBox('開始錄製 NDT CSV 時同步啟用延遲預檢與 RViz 彩色軌跡')
        self.feasibility_enabled.setChecked(True)
        group.layout().addWidget(self.feasibility_enabled)
        note=QLabel('綠=幾何預檢通過；黃=風險／無地圖驗證；紅=預檢未通過；灰=前方資料不足。\n'
                    '採用 04 分頁現有 Lookahead、車寬、Half Width、偏離上限、停車距離及取樣間距。'
                    '檢查結果依未來預瞄路段延遲更新。未執行認證級碰撞檢查。')
        note.setWordWrap(True)
        group.layout().addWidget(note)
        self.feasibility_note=QLabel('尚未錄製；按「開始錄製 NDT CSV」後會同步啟動預檢。')
        self.feasibility_note.setWordWrap(True)
        group.layout().addWidget(self.feasibility_note)
        # Keep the new control directly next to the existing trajectory recorder.
        body.insertWidget(max(0,body.count()-1),group)
        return page

    def page_navigation(self):
        page=super().page_navigation()
        self.nav_spacing.setValue(float(self.settings.value('nav_trajectory_spacing_m',.25)))
        self.nav_spacing.valueChanged.connect(
            lambda value:self.settings.setValue('nav_trajectory_spacing_m',float(value)))
        row=QHBoxLayout()
        self.feasibility_view_csv_btn=QPushButton('只讀載入彩色 Waypoint CSV（不啟動車輛）')
        self.feasibility_view_csv_btn.clicked.connect(self.preview_saved_waypoint_csv)
        row.addWidget(self.feasibility_view_csv_btn)
        page.widget().layout().insertLayout(12,row)
        return page

    def preview_saved_waypoint_csv(self):
        if self.is_running('nav_supervisor') or getattr(self,'simple_request',False):
            self.log('請先停止實車導航，再更換或預覽彩色 Waypoint。');return
        if self.is_running('nav_bridge'):
            if getattr(self,'nav_loaded_path',None)==self.vpath(self.nav_path):
                self.open_nav_rviz()
                return
            self.log('已載入另一條路徑；請先停止原本的導航預覽，再載入新 CSV。')
            return
        if not self.validate_nav_csv():return
        self.load_waypoints()
        if self.is_running('nav_bridge'):
            self.open_nav_rviz()

    def _current_feasibility_profile(self):
        width=self.nav_vehicle_width.value()
        return _profile_for(
            self.nav_lookahead_ratio.value(),self.nav_min_lookahead.value(),
            width,self.nav_corridor_half_width.value(),
            self.nav_cross_track_limit.value(),self.nav_obstacle_stop_distance.value(),
            self.nav_cruise_speed.value(),spacing=self.nav_spacing.value())

    def start_pose_capture(self):
        if self.is_running('nav_supervisor') or getattr(self,'simple_request',False):
            self.log('實車導航進行中不能同時錄製預檢；請先停止自主導航。')
            return
        super().start_pose_capture()
        if not self.is_running('csv_pose') or not self.feasibility_enabled.isChecked():
            return
        try:
            profile=self._current_feasibility_profile()
        except (ValueError,TypeError) as exc:
            self.log('軌跡預檢參數錯誤：'+str(exc));return
        raw=getattr(self,'csv_raw_path','')
        suffix=os.path.basename(raw).replace('_raw.csv','')
        state=os.path.join('/tmp','hunter_feasibility_{}_{}.json'.format(suffix,os.getpid()))
        self.feasibility_state_path=state
        self.feasibility_profile=profile
        self.feasibility_last_read=0.0
        self.settings.setValue('nav_trajectory_spacing_m',profile['spacing_m'])
        self.settings.setValue('feasibility_spacing_m',profile['spacing_m'])
        # No motor control: this node only subscribes and publishes Markers/JSON.
        script=os.path.join(BUNDLE,'nav_trajectory_preview_node.py')
        if not self.start_proc('feasibility_preview',[
                'python2','-u',script,'--profile-json',_json.dumps(profile),
                '--state-file',state]):
            self.log('預檢節點啟動失敗；原本 NDT CSV 錄製仍進行。')
            return
        self.feasibility_note.setText('預檢中：使用當次 GUI 參數；依預瞄距離延遲評估，灰色代表尚未完成驗證。')
        self.log('線上預檢啟動：{}；Half Width={:.2f}m、Min Lookahead={:.2f}m、間距={:.2f}m。'.format(
            _profile_id(profile),profile['corridor_half_width_m'],
            profile['min_lookahead_m'],profile['spacing_m']))

    def stop_pose_capture(self):
        if self.is_running('feasibility_preview'):
            self.stop('feasibility_preview')
        super().stop_pose_capture()

    def _csv_finish(self,text):
        super()._csv_finish(text)
        if not text.startswith('CSV 已整理'):
            return
        if self.feasibility_state_path:
            QTimer.singleShot(500,lambda:self._finish_colored_csv(0))
        else:
            self.log('此次未啟用線上預檢；可在 04 用「產生 Waypoint CSV」執行離線預檢。')

    def _finish_colored_csv(self,attempt):
        path=self.feasibility_state_path
        if not path:return
        ready=False
        try:
            with open(path,encoding='utf-8') as f:
                snap=_json.load(f)
            ready=(snap.get('completed') is True and
                   len(snap.get('segments',[]))==len(snap.get('samples',[])))
        except (IOError,OSError,ValueError):pass
        if not ready:
            if attempt<35:
                QTimer.singleShot(200,lambda:self._finish_colored_csv(attempt+1))
            else:
                self.log('線上預檢尚未完成快照：不會假造錄製時的顏色；已保留整理後 NDT CSV。')
                self.feasibility_note.setText('未取得完整預檢快照，請先檢查 ROS 日誌。')
            return
        source=self.vpath(self.nav_csv)
        target=source.replace('_pose.csv','_waypoints.csv')
        if target==source:target=source+'.waypoints.csv'
        profile=snap['profile']
        cmd=['python3',os.path.join(BUNDLE,'generate_waypoints.py'),
             '--input',source,'--output',target,
             '--spacing-m',str(profile['spacing_m']),
             '--speed-kmh',str(min(5.0,max(.10,profile['speed_mps']*3.6))),
             '--state-json',path]
        def finished(message):
            if message.startswith('成功：'):
                self.nav_path.setText(target)
                self.feasibility_latest_file=target
                self.save_live_settings()
                self.feasibility_note.setText('已儲存彩色 Waypoint CSV：'+target)
                self.log('03→04 同一份彩色路段資料：'+target)
            else:
                self.feasibility_note.setText('彩色 CSV 產生失敗：'+message[-250:])
            self.log(message)
        self.async_task('彩色 Waypoint CSV',lambda:self._run_waypoint_generator(cmd),finished)

    def generate_nav_csv(self):
        source,target=self.vpath(self.nav_csv),self.vpath(self.nav_path)
        if not os.path.isfile(source):
            self.log('找不到 NDT 整理後 CSV：'+source);return
        profile=self._current_feasibility_profile()
        script=os.path.join(BUNDLE,'generate_waypoints.py')
        cmd=['python3',script,'--input',source,'--output',target,
             '--spacing-m',str(self.nav_spacing.value()),
             '--speed-kmh',str(self.nav_speed.value()),
             '--profile-json',_json.dumps(profile)]
        self.async_task('離線預檢 Waypoint CSV',lambda:self._run_waypoint_generator(cmd))

    def validate_nav_csv(self):
        ok=super().validate_nav_csv()
        if not ok:return False
        try:
            data=load_route(self.vpath(self.nav_path))
        except (IOError,OSError,ValueError):return ok
        extras=[p.get('segment_status') for p in data if p.get('segment_status')]
        if extras:
            tally=dict((s,extras.count(s)) for s in ('PASS','WARN','FAIL','PENDING'))
            ids=set(p.get('check_profile','') for p in data if p.get('check_profile'))
            current=_profile_id(self._current_feasibility_profile())
            stale=(len(ids)!=1 or current not in ids)
            self.feasibility_route_has_fail=('FAIL' in extras)
            self.feasibility_route_stale=stale
            self.nav_note.setText(self.nav_note.text()+'\nCSV 預檢：'+str(tally)+
                    ('；⚠ 檢查參數已改變，儲存顏色僅供參考' if stale else '；參數一致'))
            self.log('CSV 路段預檢：{}；{}'.format(tally,'參數已變動需重檢' if stale else '參數一致'))
        else:
            self.feasibility_route_has_fail=False
            self.feasibility_route_stale=False
            self.log('傳統六欄 Waypoint CSV：未包含彩色預檢狀態。')
        return ok

    def simple_start(self):
        if self.is_running('csv_pose') or self.is_running('feasibility_preview'):
            self.log('錄製中的路線尚未完成預檢；請先停止錄製、檢查彩色 CSV，再開始導航。')
            return
        return super().simple_start()

    def arm_nav(self,hardware):
        if self.is_running('csv_pose') or self.is_running('feasibility_preview'):
            self.log('錄製預檢未完成，實車導航不允許與錄製同時執行。')
            return
        # Fail-closed for NEW colored CSV known to have a failed segment or a
        # stale parameter profile. Old 6-field CSV follows previous checks.
        if hardware and self.validate_nav_csv():
            if getattr(self,'feasibility_route_has_fail',False):
                self.log('此 CSV 含預檢 FAIL 路段；請在 RViz 檢查並重新錄製，不啟動實車導航。')
                return
            if getattr(self,'feasibility_route_stale',False):
                self.log('當前導航參數和 CSV 預檢設定不同；請重新產生可行性狀態後再行駛。')
                return
        return super().arm_nav(hardware)

    def tick(self):
        super().tick()
        if not getattr(self,'feasibility_state_path',None):return
        if not self.is_running('feasibility_preview'):return
        if time.monotonic()-self.feasibility_last_read < 1.2:return
        self.feasibility_last_read=time.monotonic()
        try:
            with open(self.feasibility_state_path,encoding='utf-8') as f:
                data=_json.load(f)
        except (IOError,OSError,ValueError):return
        d=_feasibility_counts(data.get('segments',[]))
        self.feasibility_note.setText('錄製中：{} 點｜綠 {} 黃 {} 紅 {} 灰 {}｜地圖={}。'
            '注意前方 Lookahead 尚未錄到的路段會延遲更新。'.format(
            len(data['samples']),d['PASS'],d['WARN'],d['FAIL'],d['PENDING'],
            '有高度回波' if data['map_loaded'] else '未驗證'))

    def stop_matching(self):
        if self.is_running('feasibility_preview'):self.stop('feasibility_preview')
        super().stop_matching()

    def stop_all(self):
        if self.is_running('feasibility_preview'):self.stop('feasibility_preview')
        super().stop_all()

    def closeEvent(self,event):
        if self.is_running('feasibility_preview'):self.stop('feasibility_preview')
        super().closeEvent(event)

# v1.6.5: supervised VLP-16 recovery with no retry quota, backoff or reset timer.
# No motor/CAN commands are emitted by this watchdog. During hardware navigation
# a sensor loss requests navigation STOP and never auto-resumes navigation.
class OnlineGUIV161(OnlineGUIV160):
    def __init__(self):
        self.lidar_watch_armed = False
        self.lidar_monitor_file = '/tmp/hunter_lidar_health_{}.json'.format(os.getpid())
        self.lidar_health_first_seen = False
        self.lidar_started_wall = 0.0
        self.lidar_started_monotonic = 0.0
        self.lidar_last_watch_tick = 0.0
        # Attempt counter is informational only; it does not gate retries.
        self.lidar_recovery_attempts = 0
        self.lidar_recovery_phase = None
        self.lidar_recovery_started = 0.0
        self.lidar_monitor_last_good = 0.0
        self.lidar_good_since = None
        self.lidar_recording_interrupted = False
        self.lidar_recovery_internal_stop = False
        self.lidar_sensor_healthy = False
        self.lidar_monitor_notice = 0.0
        self.lidar_active_port = 2368
        super().__init__()
        self.setWindowTitle('HUNTER / AUTOWARE-LITE ONLINE — VLP 自動恢復 v1.6.5')
        self.log('v1.6.6：終點與減速依 Waypoint 順序／剩餘路徑長度，完成後需重新 Start。')
        self.log('v1.6.5：VLP 失聯可持續重試，無次數上限、無指數退避、無 30 秒重設條件。')

    def page_overview(self):
        page = super().page_overview()
        layout = page.widget().layout()
        box = self.group('VLP-16 點雲健康監控／失聯恢復（僅重啟本 GUI 啟動的 Driver）')
        self.lidar_auto_recover = QCheckBox('開啟自動恢復：點雲持續逾時才重啟 VLP Driver；不自動恢復導航')
        enabled = str(self.settings.value('lidar_auto_recover_enabled','true')).lower() in ('1','true','yes')
        self.lidar_auto_recover.setChecked(enabled)
        self.lidar_auto_recover.toggled.connect(self._lidar_auto_toggle)
        box.layout().addWidget(self.lidar_auto_recover)
        row = QHBoxLayout()
        row.addWidget(QLabel('點雲連續無更新門檻（秒）'))
        self.lidar_stall_seconds = QSpinBox()
        self.lidar_stall_seconds.setRange(3,15)
        self.lidar_stall_seconds.setValue(int(self.settings.value('lidar_stall_seconds',5)))
        self.lidar_stall_seconds.valueChanged.connect(lambda v:self.settings.setValue('lidar_stall_seconds',v))
        row.addWidget(self.lidar_stall_seconds)
        row.addStretch()
        box.layout().addLayout(row)
        self.lidar_watch_status = QLabel('未啟動監控；請在 00 頁啟動 VLP-16 Driver。')
        self.lidar_watch_status.setWordWrap(True)
        box.layout().addWidget(self.lidar_watch_status)
        helptext = QLabel('VLP 自動恢復無重啟次數上限、無遞增等待、無穩定 30 秒重設條件。'
            '只要舊 Driver 已退出、ROS Master 可用且導航/運動已安全停止，就直接重啟。'
            '若上述必要條件暫時不成立，監控會持續等待並自動再試，不永久停用。'
            '錄製／建圖中斷會停止當次作業；導航絕不自動恢復行駛。')
        helptext.setWordWrap(True)
        box.layout().addWidget(helptext)
        layout.insertWidget(max(0,layout.count()-1),box)
        return page

    def _lidar_auto_toggle(self, enabled):
        self.settings.setValue('lidar_auto_recover_enabled',bool(enabled))
        if not enabled:
            self.lidar_recovery_phase = None
            if self.is_running('lidar_health'): super().stop('lidar_health')
            self.lidar_watch_status.setText('自動恢復已關閉；LiDAR Driver 不受影響。')
        elif self.is_running('lidar'):
            self._ensure_lidar_monitor()

    def _ensure_lidar_monitor(self):
        if not self.lidar_auto_recover.isChecked(): return
        if not self.is_running('lidar'): return
        if self.is_running('lidar_health'): return
        try:
            if os.path.exists(self.lidar_monitor_file):os.unlink(self.lidar_monitor_file)
        except OSError:pass
        self.start_proc('lidar_health',['python2','-u',
            os.path.join(BUNDLE,'lidar_health_monitor.py'),
            '--state-file',self.lidar_monitor_file])

    def start_proc(self,role,args,ros=True,hunter=False,stdin_data=None):
        ok=super().start_proc(role,args,ros=ros,hunter=hunter,stdin_data=stdin_data)
        if ok and role=='lidar':
            for arg in args:
                if str(arg).startswith('port:='):
                    try:self.lidar_active_port=int(str(arg).split(':=',1)[1])
                    except (ValueError,TypeError):pass
            self.lidar_watch_armed=True
            self.lidar_started_wall=time.time()
            self.lidar_started_monotonic=time.monotonic()
            self.lidar_health_first_seen=False
            self.lidar_good_since=None
            self.lidar_sensor_healthy=False
            self.lidar_monitor_last_good=0.0
            self._ensure_lidar_monitor()
        return ok

    def stop(self,role):
        if role=='lidar' and not self.lidar_recovery_internal_stop:
            # Operator intentionally stopped the driver: never auto restart.
            self.lidar_watch_armed=False
            self.lidar_recovery_phase=None
            if self.is_running('lidar_health'):super().stop('lidar_health')
            self.lidar_watch_status.setText('已由操作員停止 LiDAR；不會自動重啟。')
        return super().stop(role)

    def _lidar_nav_busy(self):
        return (getattr(self,'simple_request',False) or
                any(self.is_running(x) for x in ('nav_gate','nav_supervisor','motion','zero','mapping')))

    def _lidar_begin_recovery(self,reason):
        if self.lidar_recovery_phase is not None:return
        now=time.monotonic()
        if self._lidar_nav_busy():
            self.log('VLP 點雲逾時：先要求停止實車導航/運動，禁止自動恢復導航。')
            self.simple_stop()
            if self.is_running('motion'):self.stop('motion')
        if self.is_running('mapping'):
            self.log('VLP 點雲中斷：已要求停止正在執行的 NDT 建圖，以免持續累積不完整地圖。')
            self.stop_mapping()
        if self.is_running('csv_pose'):
            self.lidar_recording_interrupted=True
            # Never generate an apparently clean colored waypoint route from
            # a recorded pose segment containing a LiDAR dropout.
            self.feasibility_state_path=None
            self.log('VLP 點雲逾時：已要求停止 NDT CSV 錄製；本次資料不得直接用於導航。')
            self.stop_pose_capture()
        self.lidar_sensor_healthy=False
        self.lidar_recovery_phase='wait_safe_stop'
        self.lidar_recovery_started=now
        self.lidar_watch_status.setText('光達失聯：等待導航/測試命令停止，再恢復 VLP 驅動。')
        self.log('VLP AUTO-RECOVERY: {}。僅重啟 LiDAR Driver，不關閉地圖/NDT/RViz。'.format(reason))

    def _lidar_progress_recovery(self):
        """Keep retrying without a time/quota backoff, but never duplicate a Driver.

        The 1 Hz GUI health tick is a polling cadence, NOT an imposed restart
        cooldown. A restart is eligible immediately after the previous owned
        driver has exited and hardware-navigation is no longer active.
        """
        phase=self.lidar_recovery_phase
        if phase is None:return
        now=time.monotonic()
        if phase=='wait_safe_stop':
            if self._lidar_nav_busy():
                if now-self.lidar_recovery_started>=12.0 and now-self.lidar_monitor_notice>=12.0:
                    self.lidar_monitor_notice=now
                    self.log('VLP AUTO-RECOVERY: 仍在等待導航/運動安全停止；監控未停用，將持續確認。')
                return
            self.lidar_recovery_attempts+=1
            self.log('VLP AUTO-RECOVERY: 第 {} 次重啟嘗試（無次數上限及遞增等待）。'.format(
                self.lidar_recovery_attempts))
            if self.is_running('lidar'):
                self.lidar_recovery_internal_stop=True
                try:self.stop('lidar')
                finally:self.lidar_recovery_internal_stop=False
            self.lidar_recovery_phase='stopping'
            self.lidar_recovery_started=now
            return
        if phase=='stopping':
            if self.is_running('lidar'):
                # Never start another UDP 2368 receiver over an owned process.
                # Unlike v1.6.4, do not disarm the watchdog at an arbitrary
                # 9-second limit; wait for the old process to actually exit.
                if now-self.lidar_recovery_started>=9.0 and now-self.lidar_monitor_notice>=9.0:
                    self.lidar_monitor_notice=now
                    self.log('VLP AUTO-RECOVERY: 舊 Driver 尚未退出；持續等待，避免雙重綁定 UDP。')
                return
            self.lidar_recovery_phase='restart_ready'
            self.log('VLP AUTO-RECOVERY: 舊 Driver 已退出；直接準備重啟，不額外延遲。')
            phase='restart_ready'
        if phase=='restart_ready':
            if self._lidar_nav_busy():
                # Never restart a sensor while motion-control processes remain
                # active. Continue checking on the next GUI tick.
                return
            if not master_ok():
                if now-self.lidar_monitor_notice>=10.0:
                    self.lidar_monitor_notice=now
                    self.log('VLP AUTO-RECOVERY: ROS Master 暫時不可用；持續等待連線恢復，不停用監控。')
                return
            if self.is_running('lidar'):
                # The operator or another code path started the owned Driver.
                # Do not launch a duplicate instance; inspect its fresh data.
                self.lidar_recovery_phase='restarting'
                self.lidar_recovery_started=now
                return
            args=['roslaunch','velodyne_pointcloud','VLP16_points.launch',
                  'device_ip:=','port:={}'.format(self.lidar_active_port)]
            self.lidar_recovery_phase='restarting'
            self.lidar_recovery_started=now
            if not self.start_proc('lidar',args):
                # The GUI will try again on its next health tick. No backoff,
                # no attempt ceiling, no need to wait for 30 healthy seconds.
                self.lidar_recovery_phase='restart_ready'
                self.log('VLP AUTO-RECOVERY: Driver 啟動失敗；持續重試。')
                return
            self.log('VLP AUTO-RECOVERY: 已重新啟動 LiDAR Driver；等待新點雲。')
            return
        if phase=='restarting':
            if self.lidar_sensor_healthy and self.lidar_good_since is not None and now-self.lidar_good_since>2.5:
                self.lidar_recovery_phase=None
                self.lidar_watch_status.setText('點雲恢復正常。注意：NDT 仍需重新確認定位，導航不會自動繼續。')
                self.log('VLP AUTO-RECOVERY: /velodyne_points 已持續恢復。請重新檢查 NDT/錄製。')
            elif not self.is_running('lidar'):
                self.lidar_recovery_phase=None
                self._lidar_begin_recovery('新 Driver 程序已結束，直接再次嘗試恢復')
            elif now-self.lidar_recovery_started>20:
                # This is a detection timeout for a *failed* start, not a
                # wait imposed between attempts. Immediately start recovery.
                self.lidar_recovery_phase=None
                self._lidar_begin_recovery('重啟後仍未收到穩定點雲')

    def _lidar_check_health(self):
        now=time.monotonic()
        if now-self.lidar_last_watch_tick<1.0:return
        self.lidar_last_watch_tick=now
        if not self.lidar_auto_recover.isChecked() or not self.lidar_watch_armed:return
        if self.lidar_recovery_phase in ('wait_safe_stop','stopping','restart_ready'):
            self._lidar_progress_recovery()
            return
        self._ensure_lidar_monitor()
        if not self.is_running('lidar') and self.lidar_recovery_phase is None:
            self._lidar_begin_recovery('LiDAR Driver 程序已結束')
            return
        try:
            with open(self.lidar_monitor_file,encoding='utf-8') as stream:health=_json.load(stream)
        except (IOError,OSError,ValueError):
            health={}
        update_age=time.time()-float(health.get('updated_wall',0.0))
        last_wall=float(health.get('last_points_wall',0.0))
        age=time.time()-last_wall if last_wall>0 else None
        fresh_monitor=abs(update_age)<3.0
        has_new_data=(last_wall>=self.lidar_started_wall and int(health.get('count',0))>0)
        healthy=(fresh_monitor and has_new_data and age is not None and
                 age<min(2.5,float(self.lidar_stall_seconds.value())))
        self.lidar_sensor_healthy=bool(healthy)
        if healthy:
            self.lidar_health_first_seen=True
            if self.lidar_good_since is None:self.lidar_good_since=now
            self.lidar_monitor_last_good=now
            self.lidar_watch_status.setText('VLP 正常：/velodyne_points 最新訊息距今 {:.1f} 秒；自動恢復已待命。'.format(age))
        else:
            self.lidar_good_since=None
            if fresh_monitor:
                display='無新點雲' if age is None else '最近點雲已 {:.1f} 秒'.format(max(0.0,age))
                self.lidar_watch_status.setText('VLP 資料等待中：{}（觸發門檻 {} 秒）'.format(display,self.lidar_stall_seconds.value()))
            else:
                self.lidar_watch_status.setText('健康監控程序尚未準備／停止回報；不依過期狀態重啟 LiDAR。')
        if self.lidar_recovery_phase=='restarting':
            self._lidar_progress_recovery()
            return
        if not fresh_monitor:
            if now-self.lidar_started_monotonic>18.0 and now-self.lidar_monitor_notice>15:
                self.lidar_monitor_notice=now
                self.log('VLP 健康監控沒有新回報；請檢查 Python2/ROS Topic。未依不可信狀態重啟。')
            return
        timeout=float(self.lidar_stall_seconds.value())
        if self.lidar_health_first_seen and age is not None and age>=timeout:
            self._lidar_begin_recovery('/velodyne_points 已 {:.1f} 秒未更新'.format(age))
        elif not self.lidar_health_first_seen and now-self.lidar_started_monotonic>18.0:
            self._lidar_begin_recovery('啟動超過 18 秒仍無第一筆點雲')

    def start_pose_capture(self):
        if self.lidar_auto_recover.isChecked() and self.lidar_watch_armed and not self.lidar_sensor_healthy:
            self.log('目前無法確認新鮮 VLP 點雲，先確認 LiDAR 已恢復再開始 NDT CSV 錄製。')
            return
        self.lidar_recording_interrupted=False
        return super().start_pose_capture()

    def simple_start(self):
        if self.lidar_auto_recover.isChecked() and self.lidar_watch_armed and not self.lidar_sensor_healthy:
            self.log('VLP 健康檢查未通過，禁止開始實車自主導航。')
            return
        return super().simple_start()

    def tick(self):
        super().tick()
        self._lidar_check_health()

    def stop_all(self):
        self.lidar_watch_armed=False
        self.lidar_recovery_phase=None
        if self.is_running('lidar_health'):super().stop('lidar_health')
        super().stop_all()

    def closeEvent(self,event):
        self.lidar_watch_armed=False
        self.lidar_recovery_phase=None
        if self.is_running('lidar_health'):super().stop('lidar_health')
        return super().closeEvent(event)

# v1.6.3: remove all fixed/min-turn-radius preview settings. The precheck
# now estimates Pure Pursuit tracking using the configured lookahead/speed,
# and displays potential tracking failure vs route safety/clearance warnings.
# Existing Supervisor angular-rate, lateral acceleration and STOP protections
# remain separate and unchanged. No new motor/CAN publisher is introduced.
class OnlineGUIV164(OnlineGUIV161):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('HUNTER / AUTOWARE-LITE ONLINE — Pure Pursuit + Recovery v1.6.10')
        self.log('v1.6.10：彎道以局部路段投影更新 Waypoint 進度；依實際 AGX 協定套用底盤參數，新增預瞄點落後時的倒車脫困。')
        self.log('v1.6.9：修正弧長裁切與 Autoware PP 最小直線距離的衝突，不再因預瞄路徑被裁短而間歇發零速。')
        self.log('v1.6.8：配合實際 hunter_ws 的 yaw-rate 介面，依序弧長預瞄；前方受阻或彎道車頭偏差過大時自動倒車脫困。')
        self.log('v1.6.6：終點與減速依 Waypoint 順序／剩餘路徑長度，完成後需重新 Start。')
        self.log('v1.6.5：VLP 無次數限制、無 3～60 秒退避、無穩定 30 秒計數重設；只確認重啟必要條件。')
        self.log('v1.6.3：取消固定迴轉半徑的預檢紅黃燈。'
                 '以 Pure Pursuit Lookahead / 速度的預測追蹤偏差與進度顯示路段狀態；'
                 '實車角速度、側向加速度、障礙、定位及停車保護不變。')

    def page_navigation(self):
        page=super().page_navigation()
        layout=page.widget().layout()
        note=QLabel('預檢核心：Pure Pursuit 在目前 Lookahead 與速度下是否能跟隨路徑；實際控制使用依序弧長預瞄，避免跨越閉合路線。'
                    '不再以任意轉彎半徑將實際遙控錄製路線標成 FAIL／WARN。'
                    '綠色僅為簡化運動學預測，仍需檢查地圖回波與實際車體掃掠。')
        note.setWordWrap(True)
        layout.insertWidget(13,note)
        action=QHBoxLayout()
        self.nav_recheck_feasibility=QPushButton('使用 Pure Pursuit 參數重新預檢 CSV（保留原檔／只讀）')
        self.nav_recheck_feasibility.clicked.connect(self.recheck_saved_feasibility)
        action.addWidget(self.nav_recheck_feasibility)
        layout.insertLayout(14,action)
        recovery=QGroupBox('Recovery Planner｜彎道受阻／車頭偏差脫困（實驗功能）')
        rec=QVBoxLayout(); recovery.setLayout(rec)
        self.nav_recovery_enable=QCheckBox('啟用自動倒車脫困（後方受阻時自動等待）')
        self.nav_recovery_enable.setChecked(self.settings.value('nav_recovery_enabled',True,type=bool))
        self.nav_recovery_enable.toggled.connect(lambda value:self.settings.setValue('nav_recovery_enabled',value))
        rec.addWidget(self.nav_recovery_enable)
        form=QGridLayout()
        self.nav_recovery_fields={}
        fields=[
            ('front','base_link 到車頭 (m)',.20,2.,.70,'實測，預設值不是 Hunter 規格'),
            ('rear','base_link 到車尾 (m)',.20,2.,.50,'實測，包含外掛設備'),
            ('axle','base_link 在後軸前方 (m)',0.,2.,.35,'base_link 至後軸中心的前向偏移，需實測'),
            ('radius','後軸最小轉彎半徑 (m)',.50,5.,1.50,'以實測倒車轉向能力為準，不是原本 CSV 預檢限制'),
            ('distance','單次最大後退 (m)',.20,.80,.50,'限制單次脫困距離'),
            ('speed','最大後退速度 (m/s)',.05,.15,.10,'限制上限，實際速度受 NDT 與剩餘距離進一步限制'),
            ('margin','車身全周安全緩衝 (m)',.20,.60,.30,'包含車尾、車頭、兩側；不是安全認證'),
            ('attempts','每次受阻最多嘗試',1,3,2,'達上限停車；正常跟隨通過後續兩點才重設')]
        for i,(key,label,lo,hi,default,tip) in enumerate(fields):
            widget=QSpinBox() if key=='attempts' else QDoubleSpinBox()
            widget.setRange(lo,hi)
            if key!='attempts':widget.setDecimals(2);widget.setSingleStep(.05)
            try:
                saved=(int if key=='attempts' else float)(self.settings.value('nav_recovery_'+key,default))
            except (ValueError,TypeError):saved=default
            widget.setValue(saved)
            widget.valueChanged.connect(lambda v,k=key:self.settings.setValue('nav_recovery_'+k,v))
            widget.setToolTip(tip);self.nav_recovery_fields[key]=widget
            form.addWidget(QLabel(label),i//2,(i%2)*2)
            form.addWidget(widget,i//2,(i%2)*2+1)
        rec.addLayout(form)
        warning=QLabel('尺寸／半徑為待量測暫填值。僅隔離場地靜態障礙測試；無點雲、未知格、TF／NDT／心跳不符即禁止倒車。'
                       'RViz：青色=選定，綠色=可行，紅色=拒絕。脫困時凍結原 Waypoint，不會跳點。')
        warning.setWordWrap(True);rec.addWidget(warning)
        self.nav_recovery_enable.toggled.connect(self._recovery_controls_changed)
        self._recovery_controls_changed()
        layout.insertWidget(15,recovery)
        return page

    def _recovery_controls_changed(self,*args):
        locked=getattr(self,'simple_request',False) or self.is_running('nav_supervisor')
        enabled=self.nav_recovery_enable.isChecked()
        self.nav_recovery_enable.setEnabled(not locked)
        for widget in self.nav_recovery_fields.values():widget.setEnabled(enabled and not locked)

    def simple_start(self):
        if self.nav_recovery_enable.isChecked():
            from nav_recovery_planner import RecoveryConfig
            f=self.nav_recovery_fields
            try:
                RecoveryConfig(self.nav_vehicle_width.value(),f['front'].value(),f['rear'].value(),
                    f['axle'].value(),f['radius'].value(),f['distance'].value(),f['speed'].value(),
                    f['attempts'].value(),f['margin'].value())
            except ValueError as exc:
                self.log('Recovery 設定不合法：'+str(exc));return
        result=super().simple_start()
        self._recovery_controls_changed()
        return result

    def tick(self):
        super().tick()
        if hasattr(self,'nav_recovery_enable'):
            self._recovery_controls_changed()

    def recheck_saved_feasibility(self):
        if (self.is_running('nav_supervisor') or self.is_running('nav_bridge') or
            getattr(self,'simple_request',False) or self.is_running('csv_pose') or
            self.is_running('feasibility_preview')):
            self.log('請先停止導航、路線預覽及錄製，再重新預檢 CSV。');return
        source=self.vpath(self.nav_path)
        if not os.path.isfile(source):
            self.log('找不到 Waypoint CSV：'+source);return
        try:
            profile=self._current_feasibility_profile()
        except (TypeError,ValueError) as exc:
            self.log('預檢設定錯誤：'+str(exc));return
        script=os.path.join(BUNDLE,'recheck_waypoint_feasibility.py')
        required=(script,os.path.join(BUNDLE,'nav_trajectory_feasibility.py'),
                  os.path.join(BUNDLE,'route_safety.py'))
        missing=[os.path.basename(p) for p in required if not os.path.isfile(p)]
        if missing:
            self.log('重新預檢無法執行：GUI 安裝目錄缺少 {}；請安裝完整 v1.6.4 更新包。'.format(
                ', '.join(missing)))
            return
        cmd=['python3',script,'--input',source,'--profile-json',_json.dumps(profile)]
        def finished(message):
            self.log(message)
            if message.startswith('成功：'):
                import re
                found=re.search(r'NEW_CSV=(\S+)',message)
                if found:
                    self.nav_path.setText(found.group(1))
                    self.save_live_settings()
                self.log('新檔已產生；如 RViz 仍載入舊線，停止預覽後再按「只讀載入彩色 Waypoint CSV」。')
        self.async_task('Pure Pursuit CSV 重新預檢',lambda:self._run_waypoint_generator(cmd),finished)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    gui = OnlineGUIV164()
    gui.show()
    sys.exit(app.exec_())
