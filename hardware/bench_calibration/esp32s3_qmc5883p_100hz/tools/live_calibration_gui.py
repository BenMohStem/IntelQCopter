#!/usr/bin/env python3
"""IntelQCopter guided calibration GUI (binary v2/v3 and timing/LiDAR v4).

The firmware sends 500 packet/s; only fresh MAG/BMP packets count for slow
sensor phases. CSV stores only accepted calibration samples for the active
phase and the channels needed to calibrate that sensor; the live stream remains
available to plots but is not archived.
Run from tools/ or using start_calibration.ps1 in the project root.
"""
from __future__ import annotations

import argparse
import binascii
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
from collections import deque
import json
import math
import os
import re
import socket
import queue
import struct
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
from pathlib import Path
from zipfile import ZipFile

MAGIC = b'\xA5\x5A'
PACKET_SIZES = {2: 52, 3: 52, 4: 92}
BAUD = 921600
G = 9.80665
# Intended MPU6050 capture profile. The packet does not carry register readback,
# so these engineering-unit conversions assume the firmware configured these ranges.
MPU_ACCEL_FULL_SCALE_G = 8.0
MPU_ACCEL_COUNTS_PER_G = 4096.0
MPU_GYRO_FULL_SCALE_DPS = 500.0
MPU_GYRO_COUNTS_PER_DPS = 65.5
MPU_ACCEL_FULL_SCALE_MPS2 = MPU_ACCEL_FULL_SCALE_G * G
HMC_COUNTS_PER_UT = 1090.0 / 100.0
QMC_COUNTS_PER_UT = 3000.0 / 100.0  # QMC5883L +/-8 G
QMCP_COUNTS_PER_UT = 3750.0 / 100.0  # QMC5883P +/-8 G, QST datasheet Rev E
ROOT = Path(__file__).resolve().parent.parent
PROJECT_ROOT = ROOT.parents[2]
CALIBRATION_ROOT = PROJECT_ROOT/'measurements'/'calibration'/'esp32s3'
SESSION_COMPLETE_DIR = CALIBRATION_ROOT/'sessions'/'complete'
SESSION_INCOMPLETE_DIR = CALIBRATION_ROOT/'sessions'/'incomplete'
ANALYSIS_DIR = CALIBRATION_ROOT/'analysis'


def prepare_tk_runtime():
    """Provide Tcl/Tk script libraries when Python ships them in zip archives.

    Some Windows Python distributions, including the configured Python 3.15
    install, include Tcl/Tk library archives but do not unpack them. Tkinter
    needs the extracted scripts and its matching DLL before the GUI opens.
    """
    import tkinter

    base = Path(sys.base_prefix)
    tcl_root = base/'tcl'
    if (tcl_root/'tcl9.0'/'init.tcl').is_file():
        return

    tcl_archives = sorted(tcl_root.glob('libtcl*.zip'))
    tk_archives = sorted(tcl_root.glob('libtk*.zip'))
    if not tcl_archives or not tk_archives:
        return

    runtime = Path(tempfile.gettempdir())/(
        f'qcopter_tk_{sys.version_info.major}{sys.version_info.minor}')
    tcl_dest = runtime/'tcl'
    tk_dest = runtime/'tk'
    if not (tcl_dest/'tcl_library'/'init.tcl').is_file():
        tcl_dest.mkdir(parents=True, exist_ok=True)
        with ZipFile(tcl_archives[-1]) as archive:
            archive.extractall(tcl_dest)
    if not (tk_dest/'tk_library'/'tk.tcl').is_file():
        tk_dest.mkdir(parents=True, exist_ok=True)
        with ZipFile(tk_archives[-1]) as archive:
            archive.extractall(tk_dest)

    version_match = re.search(r'libtk([0-9.]+)\.zip$', tk_archives[-1].name)
    version = version_match.group(1) if version_match else str(tkinter.TkVersion)
    dll_version = ''.join(version.split('.')[:2])
    dll = base/'DLLs'/f'tcl9tk{dll_version}.dll'
    if not dll.is_file():
        matches = list((base/'DLLs').glob('tcl*tk*.dll'))
        if not matches:
            return
        dll = matches[0]

    package_dir = runtime/'package'
    package_dir.mkdir(parents=True, exist_ok=True)
    package_dir.joinpath('pkgIndex.tcl').write_text(
        f'package ifneeded tk {version} [list load [file normalize {{{dll.as_posix()}}}]]\n'
        f'package ifneeded Tk {version} [list package require -exact tk {version}]\n',
        encoding='utf-8')
    os.environ['TCL_LIBRARY'] = str(tcl_dest/'tcl_library')
    os.environ['TK_LIBRARY'] = str(tk_dest/'tk_library')
    os.environ['TCLLIBPATH'] = '{'+package_dir.as_posix()+'}'
    if hasattr(os, 'add_dll_directory'):
        os.add_dll_directory(str(base/'DLLs'))

CSV_COLUMNS = (
    'kind,seq,t_us,pose,mpu_ok,bmp_ok,mag_ok,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,'
    'mpu_temp_raw,ax_mps2,ay_mps2,az_mps2,gx_dps,gy_dps,gz_dps,mpu_temp_c,'
    'bmp_new,bmp_adc_p,bmp_adc_t,bmp_temp_c,pressure_pa,mag_new,mx_raw,my_raw,mz_raw,'
    'mx_uT,my_uT,mz_uT,host_utc_iso,mag_stage,late_us,missed_ticks,'
    'mag_board_x_uT,mag_board_y_uT,mag_board_z_uT,relative_altitude_m,host_monotonic_s,'
    'phase_accepted,phase_elapsed_s,mag_sensor_type,observed_packet_dt_us,seq_gap_before,'
    'gap_gt_5ms,gap_gt_10ms,phase_attempt,lidar_reference_m,lidar_distance_m,lidar_status,'
    'lidar_signal,lidar_sensor_time_ms,lidar_module_id,sensor_age_mpu_us,sensor_age_qmc_us,'
    'sensor_age_bmp_us,sensor_age_lidar_us,service_mpu_us,service_qmc_us,service_bmp_us,'
    'service_lidar_us,sensor_fresh_mask,mpu_errors_total,qmc_errors_total,bmp_errors_total,'
    'lidar_errors_total,deadline_overruns_total,session_started_utc,session_status,missing_stages'
).split(',')


@dataclass(slots=True)
class Frame:
    flags: int
    seq: int
    t_us: int
    late_us: int
    ax: int
    ay: int
    az: int
    gx: int
    gy: int
    gz: int
    temp: int
    mx: int
    my: int
    mz: int
    bmp_raw_p: int
    bmp_raw_t: int
    bmp_tc100: int
    bmp_pa100: int
    missed_ticks: int
    version: int = 2
    sensor_age_us: tuple[int, int, int, int] | None = None
    service_us: tuple[int, int, int, int] | None = None
    lidar_distance_mm: int | None = None
    lidar_sensor_time_ms: int | None = None
    lidar_signal_strength: int | None = None
    lidar_status: int | None = None
    sensor_valid_mask: int = 0
    sensor_fresh_mask: int = 0
    lidar_module_id: int | None = None
    sensor_error_counts: tuple[int, int, int, int] | None = None
    deadline_overruns: int | None = None

    @property
    def mag_sensor_type(self):
        if self.version >= 3 and self.flags & 0x40:
            return 'QMC5883P'
        return 'QMC5883L' if self.flags & 0x80 else 'HMC5883L' 
    @property
    def mpu_ok(self): return bool(self.flags & 0x01)
    @property
    def bmp_ok(self): return bool(self.flags & 0x02)
    @property
    def mag_ok(self): return bool(self.flags & 0x04)
    @property
    def bmp_new(self): return bool(self.flags & 0x08) and self.bmp_ok
    @property
    def mag_new(self): return bool(self.flags & 0x10) and self.mag_ok
    @property
    def lidar_new(self): return self.version >= 4 and bool(self.sensor_fresh_mask & 0x08)
    @property
    def lidar_valid(self):
        return (self.version >= 4 and bool(self.sensor_valid_mask & 0x08)
                and self.lidar_status == 0 and self.lidar_distance_mm is not None
                and 20 <= self.lidar_distance_mm <= 7800)

    def accel(self):
        """Convert raw counts using the configured ±8 g MPU6050 scale."""
        return tuple(x * G / MPU_ACCEL_COUNTS_PER_G for x in (self.ax,self.ay,self.az))

    def gyro(self):
        """Convert raw counts using the configured ±500 °/s MPU6050 scale."""
        return tuple(x / MPU_GYRO_COUNTS_PER_DPS for x in (self.gx,self.gy,self.gz))

    def mag(self):
        counts_per_ut = (QMCP_COUNTS_PER_UT if self.mag_sensor_type == 'QMC5883P'
                         else QMC_COUNTS_PER_UT if self.mag_sensor_type == 'QMC5883L'
                         else HMC_COUNTS_PER_UT)
        return tuple(x / counts_per_ut for x in (self.mx,self.my,self.mz))

    def bmp(self):
        """Return plausible pressure in Pa for both observed v3 wire encodings.

        The bench firmware emits integer Pa, while the DSH candidate emits
        centi-Pa in the same v3 field. Detect the scale from physical range;
        return NaN for invalid/zero values instead of plotting a false zero.
        """
        raw = float(self.bmp_pa100)
        if 30000.0 <= raw <= 120000.0:
            return raw
        scaled = raw/100.0
        if 30000.0 <= scaled <= 120000.0:
            return scaled
        return math.nan


class BinaryDecoder:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_crc = 0
        self.bad_sync = 0
        self.bad_version = 0
        self.total = 0

    def feed(self, data: bytes):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 4:
            pos = self.buffer.find(MAGIC)
            if pos < 0:
                self.bad_sync += len(self.buffer) - 1
                del self.buffer[:-1]
                break
            if pos:
                self.bad_sync += pos
                del self.buffer[:pos]
            version = self.buffer[2]
            size = PACKET_SIZES.get(version)
            if size is None:
                self.bad_version += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < size:
                break
            pkt = bytes(self.buffer[:size])
            crc_offset = size - 2
            got = int.from_bytes(pkt[crc_offset:size], 'little')
            if binascii.crc_hqx(pkt[:crc_offset], 0xFFFF) != got:
                self.bad_crc += 1
                del self.buffer[0]
                continue
            fields = struct.unpack_from('<IIH7h3hIIhIH', pkt, 4)
            extension = {}
            if version == 4:
                values = struct.unpack_from('<8HiIH4B5H', pkt, 50)
                extension = {
                    'sensor_age_us': tuple(None if v == 0xFFFF else v for v in values[0:4]),
                    'service_us': tuple(None if v == 0xFFFF else v for v in values[4:8]),
                    'lidar_distance_mm': values[8],
                    'lidar_sensor_time_ms': values[9],
                    'lidar_signal_strength': values[10],
                    'lidar_status': values[11],
                    'sensor_valid_mask': values[12],
                    'sensor_fresh_mask': values[13],
                    'lidar_module_id': values[14],
                    'sensor_error_counts': tuple(values[15:19]),
                    'deadline_overruns': values[19],
                }
            frames.append(Frame(pkt[3], *fields, version=version, **extension))
            self.total += 1
            del self.buffer[:size]
        # Prevent unbounded noise memory on malformed streams.
        if len(self.buffer) > 65536:
            del self.buffer[-91:]
        return frames


def make_test_packet(seq, ax=0, ay=0, az=int(MPU_ACCEL_COUNTS_PER_G), flags=0x1F, mag=(0,0,0), version=2):
    b = bytearray(MAGIC + bytes([version, flags]))
    b += struct.pack('<IIH7h3hIIhIH', seq, (seq*2000)&0xFFFFFFFF, 0,
        ax,ay,az,0,0,0,0,*mag,500000,500000,2350,10132500,0)
    assert len(b) == 50
    b += struct.pack('<H', binascii.crc_hqx(b, 0xFFFF))
    return bytes(b)


def selftest():
    d = BinaryDecoder()
    d.feed(b'noise')
    packet = make_test_packet(123, int(MPU_ACCEL_COUNTS_PER_G), 0, 0, mag=(100,200,-80))
    result = []
    for bit in (packet[:1], packet[1:17], packet[17:39], packet[39:]):
        result += d.feed(bit)
    assert len(result) == 1
    f = result[0]
    assert f.seq == 123 and f.ax == int(MPU_ACCEL_COUNTS_PER_G) and f.my == 200
    assert abs(f.accel()[0]-G)<1e-5
    assert f.mag_new and f.bmp_new
    bad = bytearray(make_test_packet(124)); bad[28] ^= 0x01
    d.feed(bytes(bad))
    assert d.bad_crc == 1
    assert d.feed(make_test_packet(125))[0].seq == 125
    print('PASS: binary framing, partial packets, CRC, field offsets, units and recovery')


def load_axis_map(path):
    obj = json.loads(Path(path).read_text(encoding='utf-8'))
    name = obj.get('magnetic_axis_mapping_name','unspecified')
    m = obj['mag_sensor_to_board_display_matrix']
    if not (isinstance(m, list) and len(m)==3 and all(isinstance(r,list) and len(r)==3 for r in m)):
        raise ValueError('mag_sensor_to_board_display_matrix must be a 3x3 array')
    if any(not isinstance(x,(int,float)) or not math.isfinite(x) for row in m for x in row):
        raise ValueError('Axis-map matrix must contain finite real numbers')
    return name, m


def map_mag(m, x):
    return tuple(sum(m[i][j]*x[j] for j in range(3)) for i in range(3))


def determinant(m):
    a,b,c = m
    return (a[0]*(b[1]*c[2]-b[2]*c[1]) -
            a[1]*(b[0]*c[2]-b[2]*c[0]) +
            a[2]*(b[0]*c[1]-b[1]*c[0]))


def magnetic_axis_limits(points, minimum_half_range=20.0):
    """Three equally scaled axes; keep the ellipsoid geometry undistorted.

    points are sensor display-frame XYZ in microtesla. Equal 3-D limits and
    box aspect avoid visually stretching a sphere into an ellipsoid.
    """
    if not points:
        return ((-60.0,60.0),)*3
    valid=[p for p in points if len(p)==3 and all(math.isfinite(v) for v in p)]
    if not valid:
        return ((-60.0,60.0),)*3
    low=[min(p[i] for p in valid) for i in range(3)]
    high=[max(p[i] for p in valid) for i in range(3)]
    half=max(float(minimum_half_range),max((hi-lo) for lo,hi in zip(low,high))*.60)
    return tuple(((lo+hi)*.5-half,(lo+hi)*.5+half) for lo,hi in zip(low,high))


def configure_magnetic_axes(ax, points, *, enlarged=False):
    """Calibrated µT scale, pale back-wall panes, visible scaled grid and axis triad.

    This changes the graphics ONLY. The sensor-to-board mapping is a separate
    configuration and must be checked for a proper rotation (determinant +1).
    """
    from matplotlib.ticker import MaxNLocator, FormatStrFormatter
    lims=magnetic_axis_limits(points)
    ax.set_xlim3d(*lims[0]);ax.set_ylim3d(*lims[1]);ax.set_zlim3d(*lims[2])
    ax.set_box_aspect((1,1,1))
    ax.set_proj_type('ortho')
    ax.set_xlabel('X (µT)',labelpad=8 if enlarged else 3)
    ax.set_ylabel('Y (µT)',labelpad=8 if enlarged else 3)
    ax.set_zlabel('Z (µT)',labelpad=8 if enlarged else 3)
    ax.tick_params(axis='both',which='major',labelsize=9 if enlarged else 6,pad=1)
    ax.xaxis.set_major_locator(MaxNLocator(nbins=5,steps=[1,2,2.5,5,10]))
    ax.yaxis.set_major_locator(MaxNLocator(nbins=5,steps=[1,2,2.5,5,10]))
    ax.zaxis.set_major_locator(MaxNLocator(nbins=5,steps=[1,2,2.5,5,10]))
    for axis,rgba in zip((ax.xaxis,ax.yaxis,ax.zaxis),
                         ((.94,.96,1,.85),(.96,.98,.96,.85),(1,.97,.95,.85))):
        axis.set_pane_color(rgba)
        axis.line.set_color('#677383')
        axis.set_major_formatter(FormatStrFormatter('%.0f'))
        # mplot3d has no public grid-line styling API at the individual-axis
        # level; use this guarded visual-only customization.
        if hasattr(axis,'_axinfo'):
            axis._axinfo['grid'].update({'color':(.55,.61,.69,.52),
                                         'linestyle':'-','linewidth':.65 if not enlarged else .85})
    ax.grid(True)
    # Draw positive X/Y/Z arrows in a low corner, INSIDE the data axes.
    # These are direction indicators; their base is not the magnetic origin.
    side=lims[0][1]-lims[0][0]
    origin=tuple(a+.11*side for a,b in lims)
    size=.30*side
    artists=[]
    for i,(name,color) in enumerate((('X+','#c73735'),('Y+','#26874d'),('Z+','#2762bb'))):
        delta=[0.0,0.0,0.0];delta[i]=size
        artists.append(ax.quiver(*origin,*delta,color=color,linewidth=2.5 if enlarged else 1.6,
                                 arrow_length_ratio=.17))
        tip=list(origin);tip[i]+=.38*side
        artists.append(ax.text(*tip,name,color=color,weight='bold',fontsize=11 if enlarged else 8))
    return artists


def live_plot_y_limits(values, minimum_span):
    """Return padded limits that remain readable for nearly steady sensors."""
    finite=[value for value in values if math.isfinite(value)]
    if not finite:
        return None
    low=min(finite)
    high=max(finite)
    span=max(high-low,float(minimum_span))
    center=(low+high)*.5
    margin=max(span*.10,float(minimum_span)*.02)
    half=span*.5+margin
    return center-half,center+half


@dataclass(frozen=True)
class Stage:
    pose: str
    mag_stage: str
    target: int
    sensor: str
    prompt: str


def stages(args):
    sequence = ([Stage('REST','STILL',args.gyro_samples,'mpu',
                  'Keep the drone completely still in one fixed orientation. Warm up electronics first.')]
        + [Stage(p,'STILL',args.accel_samples,'mpu',
            f'Place the MPU6050 {p} axis vertically up and hold steady.')
           for p in ('+X','-X','+Y','-Y','+Z','-Z')]
        + [Stage('MAG','YAW',args.mag_turn_samples,'mag',
                 'Hold roughly level; slowly turn 360° around vertical, adding gentle tilts.'),
           Stage('MAG','PITCH',args.mag_turn_samples,'mag',
                 'Slowly pitch through a full 360°: nose up, inverted, nose down, return.'),
           Stage('MAG','ROLL',args.mag_turn_samples,'mag',
                 'Slowly roll through a full 360°, including inverted orientations.'),
           Stage('MAG','TUMBLE',args.mag_tumble_samples,'mag',
                 'Slowly tumble through all orientations, including diagonal poses.')]
        + ([] if args.skip_bmp else [Stage('BMP','STILL',args.bmp_samples,'bmp',
                 'Keep still at one reference height, away from drafts and propeller wash.')])
)
    return sequence


class WifiPacketStream:
    """UDP client for the ESP32-S3 AP calibration stream.

    Accept concatenated v2/v3/v4 frames. Never substitute packet reception
    time for device sample timestamps or per-sensor ages.
    """
    HELLO = b'IQ_HELLO_V1'
    READY = b'IQ_READY_V1'

    def __init__(self, host='192.168.4.1', local_port=19005,
                 control_port=19006):
        self.host = str(host)
        self.control_port = int(control_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', int(local_port)))
        self.sock.settimeout(.06)
        self.last_hello = 0.0
        self.telemetry_datagrams = 0

    def __enter__(self): return self

    def __exit__(self, exc_type, exc, tb): self.sock.close()

    def read(self, _max_bytes=4096):
        now=time.monotonic()
        if now-self.last_hello>1.0:
            self.sock.sendto(self.HELLO, (self.host,self.control_port))
            self.last_hello=now
        try:
            data, sender=self.sock.recvfrom(4096)
        except socket.timeout:
            return b''
        # Avoid accepting messages from unrelated devices on the LAN.
        if sender[0] != self.host or data == self.READY:
            return b''
        if not data.startswith(MAGIC):
            return b''
        self.telemetry_datagrams += 1
        return data


class OptionalLidarSource(threading.Thread):
    """Read F2 Mini on a *separate* 3.3 V UART adapter; never block ESP32 read.

    Frames are decoded with the existing NLink parser. Each reading carries the
    module clock and host receipt time. No fictitious 500 Hz LiDAR values.
    """
    def __init__(self, port, messages, stop):
        super().__init__(daemon=True)
        self.port, self.messages, self.stop = port, messages, stop
        self.error = None
        self.connected = threading.Event()

    def run(self):
        try:
            import serial
            from lidar_calibration import NLinkDecoder
            decoder=NLinkDecoder()
            with serial.Serial(self.port,921600,timeout=0.04,rtscts=False,dsrdtr=False) as stream:
                self.connected.set()
                while not self.stop.is_set():
                    for sample in decoder.feed(stream.read(256)):
                        try:self.messages.put_nowait((time.monotonic(),sample))
                        except queue.Full:
                            try:self.messages.get_nowait()
                            except queue.Empty:pass
                            try:self.messages.put_nowait((time.monotonic(),sample))
                            except queue.Full:pass
        except Exception as exc:
            self.error=str(exc)


class Recorder(threading.Thread):
    def __init__(self, port, event_queue, axis_map, settle_seconds=3.0,
                 wifi=False, wifi_host="192.168.4.1", wifi_port=19005, lidar_port=""):
        super().__init__(daemon=True)
        self.port = port
        self.wifi,self.wifi_host,self.wifi_port=bool(wifi),str(wifi_host),int(wifi_port)
        self.events = event_queue
        self.lidar_port = str(lidar_port).strip()
        self.lidar_queue = queue.Queue(maxsize=1024)
        self.lidar_reader = None
        self.lidar_total = 0
        self.lidar_inband_seen = False
        self.axis_map = axis_map
        self.settle_seconds = float(settle_seconds)
        self.lock = threading.Lock()
        self.active = None
        self.sample_count = 0
        self.phase_start = 0.0
        self.stop_event = threading.Event()
        self.started = threading.Event()
        self.decoder = BinaryDecoder()
        self.latency_samples = queue.Queue(maxsize=2000)
        self.frames_total = 0
        self.bytes_total = 0
        self.last_byte_at = 0.0
        self.mpu_total = self.mag_total = self.bmp_total = 0
        self.mag_valid_total = 0
        self.pending_mag_plot = None
        self.last_frame = None
        self.last_seq = None
        self.last_mcu_timestamp = None
        self.seq_gaps = 0
        self.p0_samples = []
        self.p0 = None
        self.altitude_status = 'Waiting for valid BMP280 pressure (30–120 kPa).'
        self.last_emit = 0.0
        self.phase_last_progress = 0
        self.error = None
        self.paused = False
        self.last_accepted_at = 0.0
        self.stage_warning_issued = False
        self.stage_start_time = 0.0
        self.last_reject_reason = 'No samples checked yet'
        self.last_countdown_emit = 0.0
        self.capture_started_notified = False
        self.attempt_counter = 0
        self.current_attempt = 0
        self.commands = queue.Queue()
        self.session_path = None
        self.complete_path = None
        self.session_started_utc = ''
        self.stage_counts = {}
        self.session_rows = 0
        self.current_file = None
        self._fp = None
        self._csv_writer = None

    def connect_lidar(self, port):
        """Attach the single F2 Mini on its own COM port, without restarting ESP stream.

        No LiDAR is silently substituted from ESP packets: this reads the
        F2 Mini NLink frames on a separate USB-UART at 921600 baud.
        """
        port = str(port).strip()
        if not port:
            return False, 'Choose the LiDAR USB-UART COM port first.'
        if not self.wifi and port.casefold() == str(self.port).casefold():
            return False, 'LiDAR and ESP32 must use DIFFERENT COM ports.'
        if self.lidar_reader is not None and self.lidar_reader.is_alive():
            if port.casefold() == str(self.lidar_reader.port).casefold():
                return True, f'LiDAR is already listening on {port}.'
            return False, ('LiDAR is already using '+str(self.lidar_reader.port)
                           +'. Restart the GUI to switch its active port.')
        self.lidar_port = port
        self.lidar_reader = OptionalLidarSource(port, self.lidar_queue, self.stop_event)
        self.lidar_reader.start()
        return True, f'Connecting LiDAR on {port} at 921600 baud...'

    def start_session(self, incomplete_path, complete_path, started_utc):
        """Begin a new logical session; the CSV is opened on its first sample."""
        self.commands.put(('start', Path(incomplete_path), Path(complete_path), started_utc))

    def finalize_session(self, status, missing_stages=()):
        self.commands.put(('finalize', str(status), tuple(missing_stages)))

    def discard_session(self):
        self.commands.put(('discard',))

    def _write_row(self, row):
        """Lazily create the session file when the first accepted sample arrives."""
        if self.session_path is None:
            return
        if self._csv_writer is None:
            self.session_path.parent.mkdir(parents=True, exist_ok=True)
            self._fp = self.session_path.open('x', newline='', encoding='utf-8')
            self._csv_writer = csv.writer(self._fp)
            self._csv_writer.writerow(CSV_COLUMNS)
        self._csv_writer.writerow([row.get(column, '') for column in CSV_COLUMNS])
        self.session_rows += 1

    def _process_commands(self):
        while True:
            try: command = self.commands.get_nowait()
            except queue.Empty: return
            operation = command[0]
            if operation == 'start':
                _, self.session_path, self.complete_path, self.session_started_utc = command
                self.current_file = self.session_path
                self.session_rows = 0
                self.attempt_counter = 0
                self.current_attempt = 0
                with self.lock:
                    self.stage_counts = {}
                self.events.put(('session_started', self.session_path))
            elif operation == 'finalize':
                _, status, missing_stages = command
                path = None
                rows = self.session_rows
                if (rows and self._fp is None and self.session_path is not None
                        and self.session_path.exists()):
                    # Preserve the final session metadata if the sensor stream
                    # ended just before the GUI's finalize request was handled.
                    with self.session_path.open('a', newline='', encoding='utf-8') as stream:
                        writer=csv.writer(stream)
                        meta=dict.fromkeys(CSV_COLUMNS,'')
                        meta.update(kind='META',session_started_utc=self.session_started_utc,
                                    session_status=status,missing_stages='; '.join(missing_stages))
                        writer.writerow([meta.get(column,'') for column in CSV_COLUMNS])
                if self._fp is not None and self._csv_writer is not None:
                    if rows:
                        meta = dict.fromkeys(CSV_COLUMNS, '')
                        meta.update(kind='META', session_started_utc=self.session_started_utc,
                                    session_status=status,
                                    missing_stages='; '.join(missing_stages))
                        self._csv_writer.writerow([meta.get(column, '') for column in CSV_COLUMNS])
                    self._fp.flush()
                    self._fp.close()
                    self._fp = self._csv_writer = None
                if rows and self.session_path is not None:
                    path = self.session_path
                    if status == 'complete' and self.complete_path is not None:
                        self.complete_path.parent.mkdir(parents=True, exist_ok=True)
                        self.session_path.replace(self.complete_path)
                        path = self.complete_path
                    self.current_file = path
                else:
                    self.current_file = None
                self.events.put(('session_saved', (status, path, rows, missing_stages)))
                self.session_path = self.complete_path = None
                self.session_started_utc = ''
                self.session_rows = 0
                with self.lock:
                    self.stage_counts = {}
            elif operation == 'discard':
                path = self.session_path
                rows = self.session_rows
                if self._fp is not None:
                    self._fp.close()
                    self._fp = self._csv_writer = None
                if path is not None:
                    try: path.unlink()
                    except FileNotFoundError: pass
                self.current_file = None
                self.events.put(('session_discarded', (path, rows)))
                self.session_path = self.complete_path = None
                self.session_started_utc = ''
                self.session_rows = 0
                with self.lock:
                    self.stage_counts = {}

    def pause_stage(self):
        with self.lock:
            if self.active is None or self.paused:
                return False
            self.paused = True
            return True

    def resume_stage(self):
        with self.lock:
            if self.active is None or not self.paused:
                return False
            self.paused = False
            self.phase_start = time.monotonic() + self.settle_seconds
            self.last_countdown_emit = 0.0
            self.capture_started_notified = False
            self.last_accepted_at = time.monotonic()
            return True

    def start_stage(self, stage):
        with self.lock:
            if self.active is not None:
                return False
            self.active = stage
            self.attempt_counter += 1
            self.current_attempt = self.attempt_counter
            self.stage_counts[(stage.pose, stage.mag_stage)] = 0
            self.paused = False
            self.sample_count = 0
            self.last_accepted_at = time.monotonic()
            self.stage_start_time = self.last_accepted_at
            self.stage_warning_issued = False
            self.last_reject_reason = f'Starting after {self.settle_seconds:g} s preparation delay'
            self.phase_start = time.monotonic() + self.settle_seconds
            self.last_countdown_emit = 0.0
            self.capture_started_notified = False
            self.phase_last_progress = 0
            self.last_progress_emit_at = 0.0
            self.next_hint_at = self.phase_start + 5.0
            return True

    def end_stage(self):
        with self.lock:
            self.active = None

    def run(self):
        try:
            if self.wifi:
                stream = WifiPacketStream(host=self.wifi_host,local_port=self.wifi_port)
            else:
                import serial
                stream = serial.Serial(self.port, BAUD, timeout=0.06,
                                       rtscts=False, dsrdtr=False)
            if self.lidar_port:
                ok, note = self.connect_lidar(self.lidar_port)
                self.events.put(('lidar_status',note))
            with stream as ser:
                self.started.set()
                self.events.put(('status',
                    f'Wi-Fi UDP listening on :{self.wifi_port}; connect PC to IntelQCopter-Cal AP and wait for sensor packets'
                    if self.wifi else f'Connected to {self.port} at {BAUD} baud'))
                count_since_flush = 0
                while True:
                    self._process_commands()
                    if self.stop_event.is_set():
                        break
                    data = ser.read(4096)
                    # Drain separately received LiDAR frames without waiting for
                    # its slower UART; record each with its own sensor clock.
                    for _ in range(128):
                        try:received, sample = self.lidar_queue.get_nowait()
                        except queue.Empty:break
                        self._handle_lidar(sample,received)
                    if not data:
                        continue
                    self.bytes_total += len(data)
                    self.last_byte_at = time.monotonic()
                    frames = self.decoder.feed(data)
                    for frame in frames:
                        self._handle_frame(frame)
                        count_since_flush += 1
                    if count_since_flush >= 250:
                        if self._fp is not None:
                            self._fp.flush()
                        count_since_flush = 0
        except Exception as e:
            self.error = str(e)
            self.events.put(('error', str(e)))
        finally:
            if self._fp is not None:
                try:self._fp.flush();self._fp.close()
                except OSError:pass
                self._fp = self._csv_writer = None
            self.started.set()
            self.events.put(('stopped', None))

    def _handle_lidar(self,sample,received,*,source='serial'):
        self.lidar_total+=1
        self.events.put(('lidar_sample',{
            'distance_m':sample.distance_m,'valid':bool(sample.valid),
            'status':sample.status,'signal':sample.signal_strength,
        }))
        with self.lock:
            stage=self.active
            counting=bool(stage and stage.sensor=='lidar' and not self.paused and received>=self.phase_start)
            if source == 'serial' and self.lidar_inband_seen:
                counting=False
            attempt=self.current_attempt
            if not counting:return
            accepted=bool(sample.valid)
            if not self.capture_started_notified:
                self.capture_started_notified=True
                self.events.put(('capture_started',stage))
            if accepted:
                self.sample_count+=1
                self.last_accepted_at=received
            count=self.sample_count
            if (count != self.phase_last_progress and
                    (received-self.last_progress_emit_at >= 0.20 or count >= stage.target)):
                self.phase_last_progress=count
                self.last_progress_emit_at=received
                self.events.put(('progress',count))
            if count>=stage.target:
                self.active=None
                self.events.put(('stage_done',(stage,count)))
        ref=float(int(stage.mag_stage[1:]))/1000.0
        # Persist only valid calibration returns. Invalid ranges remain visible
        # in the live GUI but do not inflate a calibration session file.
        if accepted:
            row=dict.fromkeys(CSV_COLUMNS,'')
            row.update(kind='L',pose='LIDAR',mag_stage=stage.mag_stage,
                       host_utc_iso=datetime.now(timezone.utc).isoformat(timespec='milliseconds'),
                       host_monotonic_s=f'{received:.6f}',phase_accepted=1,
                       phase_attempt=attempt,
                       lidar_reference_m=f'{ref:.3f}',lidar_distance_m=f'{sample.distance_m:.6f}',
                       lidar_status=sample.status,lidar_signal=sample.signal_strength,
                       lidar_sensor_time_ms=sample.sensor_time_ms,lidar_module_id=sample.module_id)
            # These are asynchronous board temperatures for session context,
            # not temperatures measured by the LiDAR module.
            if self.last_frame is not None:
                f=self.last_frame
                row['mpu_temp_c']=f'{f.temp/340.0+36.53:.3f}' if f.mpu_ok else ''
                row['bmp_temp_c']=f'{f.bmp_tc100/100:.3f}' if f.bmp_ok else ''
            self._write_row(row)

    @staticmethod
    def _phase_rejection_reason(f, stage):
        """Only check whether a *real* sensor sample exists.

        No gravity orientation, motion, gyro magnitude or stillness gates.
        Pose quality is evaluated OFFLINE and can be reported as invalid.
        """
        if stage.sensor == 'mpu':
            return None if f.mpu_ok else 'No valid MPU6050 acquisition (I2C/status)'
        if stage.sensor == 'mag':
            if not f.mag_ok:
                return ('No magnetometer recognized: use Scan test for '
                        'HMC5883L=0x1E, QMC5883L=0x0D or QMC5883P=0x2C; '
                        'stop this run if the device is physically absent')
            return None if f.mag_new else 'Waiting for FRESH magnetometer reading'
        if stage.sensor == 'bmp':
            if not f.bmp_ok or not f.bmp_new:
                return 'Waiting for FRESH BMP280 reading'
            return None if math.isfinite(f.bmp()) else 'BMP pressure is invalid or has unknown wire scaling'
        return 'Unsupported calibration stage'

    @staticmethod
    def _accept_phase_frame(f, stage):
        return Recorder._phase_rejection_reason(f,stage) is None

    def _handle_frame(self, f):
        now = time.monotonic()
        self.frames_total += 1
        self.mpu_total += int(f.mpu_ok)
        self.mag_valid_total += int(f.mag_ok)
        self.mag_total += int(f.mag_new)
        self.bmp_total += int(f.bmp_new)
        seq_gap_before = 0
        if self.last_seq is not None:
            gap = (f.seq-self.last_seq)&0xFFFFFFFF
            if 1 < gap < 1000000:
                seq_gap_before = gap - 1
                self.seq_gaps += seq_gap_before
        self.last_seq = f.seq
        # Timestamp *from device*, not host reception time. A 10 ms hiccup
        # must show up as a 10000 us interval, not an invented 2000 us dt.
        dt_us = ((f.t_us - self.last_mcu_timestamp) & 0xFFFFFFFF) if self.last_mcu_timestamp is not None else 0
        self.last_mcu_timestamp = f.t_us
        dt_valid = 0 < dt_us < 1000000
        self.last_frame = f
        if f.version >= 4:
            snapshot=(now,f.sensor_age_us,f.service_us,f.sensor_fresh_mask,
                      f.sensor_error_counts,f.deadline_overruns)
            try:
                self.latency_samples.put_nowait(snapshot)
            except queue.Full:
                try:self.latency_samples.get_nowait()
                except queue.Empty:pass
                try:self.latency_samples.put_nowait(snapshot)
                except queue.Full:pass

        if f.lidar_new:
            self.lidar_inband_seen=True
            sample=SimpleNamespace(
                valid=f.lidar_valid,
                distance_m=(f.lidar_distance_mm or 0)/1000.0,
                status=f.lidar_status if f.lidar_status is not None else 255,
                signal_strength=f.lidar_signal_strength if f.lidar_signal_strength is not None else 0,
                sensor_time_ms=f.lidar_sensor_time_ms if f.lidar_sensor_time_ms is not None else 0,
                module_id=f.lidar_module_id if f.lidar_module_id is not None else 0,
            )
            self._handle_lidar(sample,now,source='telemetry')

        pressure = f.bmp()
        pressure_valid = f.bmp_ok and math.isfinite(pressure)
        if pressure_valid and f.bmp_new and self.p0 is None:
            # Build the reference only from actual BMP conversions; reused
            # cached pressure values must not masquerade as separate samples.
            self.p0_samples.append(pressure)
            if len(self.p0_samples) >= 30:
                self.p0 = sorted(self.p0_samples)[len(self.p0_samples)//2]
        if not f.bmp_ok:
            self.altitude_status = 'BMP280 status invalid; waiting for pressure.'
        elif not math.isfinite(pressure):
            self.altitude_status = 'BMP pressure invalid: expected 30,000–120,000 Pa.'
        elif self.p0 is None:
            self.altitude_status = f'Setting relative pressure reference: {len(self.p0_samples)}/30.'
        else:
            self.altitude_status = 'Relative barometric height from stream-start pressure.'
        altitude = math.nan
        if pressure_valid and self.p0 is not None and self.p0 > 0:
            ratio = pressure/self.p0
            if 0.2 < ratio < 2.0:
                altitude = 44330.0*(1.0-ratio**(1.0/5.255))
        mx = f.mag()
        mx_board = map_mag(self.axis_map, mx)
        if f.mag_new:
            # Keep only the newest fresh conversion until the next bounded Tk
            # plot snapshot. Do not enqueue hundreds of per-sample GUI events.
            self.pending_mag_plot = mx_board
        with self.lock:
            stage = self.active
            counting = bool(stage and not self.paused and now >= self.phase_start)
            preparing = bool(stage and not self.paused and now < self.phase_start)
            attempt = self.current_attempt
            stage_elapsed = now-self.stage_start_time
            valid = False
            if preparing and now-self.last_countdown_emit >= 0.20:
                self.last_countdown_emit = now
                self.events.put(('countdown', (stage, max(0.0, self.phase_start-now))))
            if counting and stage.sensor != 'lidar':
                if not self.capture_started_notified:
                    self.capture_started_notified = True
                    self.events.put(('capture_started', stage))
                reason = self._phase_rejection_reason(f, stage)
                valid = reason is None
                self.last_reject_reason = 'Accepted' if valid else reason
                if valid:
                    self.sample_count += 1
                    self.last_accepted_at = now
                    key=(stage.pose,stage.mag_stage)
                    self.stage_counts[key]=self.stage_counts.get(key,0)+1
                count = self.sample_count
                if (count != self.phase_last_progress and
                        (now-self.last_progress_emit_at >= 0.20 or count >= stage.target)):
                    self.phase_last_progress = count
                    self.last_progress_emit_at = now
                    self.events.put(('progress',count))
                if now >= self.next_hint_at and (now-self.last_accepted_at) > 5.0:
                    self.events.put(('hint',
                        f'No new {stage.sensor.upper()} samples for {stage.pose}/{stage.mag_stage}. '
                        f'Reason: {self.last_reject_reason}. '
                        'This phase is not counted until valid fresh sensor data arrives.'))
                    self.next_hint_at = now + 8.0
                expected_rate = {'mpu':500.0, 'mag':15.0, 'bmp':10.0, 'lidar':50.0}[stage.sensor]
                slow_deadline = max(30.0, stage.target / expected_rate * 3.0)
                if not self.stage_warning_issued and now-self.stage_start_time > slow_deadline:
                    self.stage_warning_issued = True
                    self.events.put(('hint',
                        f'Stage {stage.pose}/{stage.mag_stage} is unusually slow: '
                        f'{count}/{stage.target} real samples. Check sensor status and connections.'))
                if count >= stage.target:
                    self.active = None
                    self.events.put(('stage_done', (stage, count)))
            elif counting and stage.sensor == 'lidar':
                if now >= self.next_hint_at and now-self.last_accepted_at > 5.0:
                    self.events.put(('hint',
                        'Waiting for fresh valid F2 readings. Connect the separate LiDAR UART or use firmware v4 forwarding; invalid ranges do not count.'))
                    self.next_hint_at=now+8.0
                slow_deadline=max(30.0,stage.target/50.0*3.0)
                if not self.stage_warning_issued and now-self.stage_start_time>slow_deadline:
                    self.stage_warning_issued=True
                    self.events.put(('hint',
                        f'LiDAR range stage is unusually slow: {self.sample_count}/{stage.target} accepted readings. Check port, status and target setup.'))
            else:
                count = self.sample_count
            # Persist only accepted calibration samples. For slower MAG/BMP
            # stages this avoids writing every 500 Hz packet between fresh
            # conversions. Unrelated sensor channels and live timing telemetry
            # stay in memory. MAG/BMP sample rows intentionally omit packet
            # sequence IDs because skipped 500 Hz packets are expected there.
            capture = counting and valid and stage.sensor != 'lidar'
        if capture:
            row=dict.fromkeys(CSV_COLUMNS,'')
            row.update(kind='D',t_us=f.t_us,pose=stage.pose,
                       host_utc_iso=datetime.now(timezone.utc).isoformat(timespec='milliseconds'),
                       host_monotonic_s=f'{now:.6f}',phase_accepted=1,
                       phase_elapsed_s=f'{stage_elapsed:.3f}',phase_attempt=attempt,
                       mag_stage=stage.mag_stage)
            if stage.sensor=='mpu':
                a=f.accel()
                row.update(seq=f.seq,mpu_ok=1,ax_raw=f.ax,ay_raw=f.ay,az_raw=f.az,
                           mpu_temp_raw=f.temp,late_us=f.late_us,
                           missed_ticks=f.missed_ticks,
                           ax_mps2=f'{a[0]:.6f}',ay_mps2=f'{a[1]:.6f}',az_mps2=f'{a[2]:.6f}',
                           mpu_temp_c=f'{f.temp/340.0+36.53:.3f}',
                           observed_packet_dt_us=dt_us if dt_valid else '',
                           seq_gap_before=seq_gap_before,
                           gap_gt_5ms=int(dt_valid and dt_us>5000),
                           gap_gt_10ms=int(dt_valid and dt_us>10000))
                # Gyro calibration uses the stationary REST phase; face stages
                # only need accelerometer data and the shared temperature.
                if stage.pose=='REST':
                    g=f.gyro()
                    row.update(gx_raw=f.gx,gy_raw=f.gy,gz_raw=f.gz,
                               gx_dps=f'{g[0]:.6f}',gy_dps=f'{g[1]:.6f}',gz_dps=f'{g[2]:.6f}')
            elif stage.sensor=='mag':
                row.update(mag_ok=1,mag_new=1,mag_sensor_type=f.mag_sensor_type,
                           mx_raw=f.mx,my_raw=f.my,mz_raw=f.mz,
                           mx_uT=f'{mx[0]:.6f}',my_uT=f'{mx[1]:.6f}',mz_uT=f'{mx[2]:.6f}',
                           mag_board_x_uT=f'{mx_board[0]:.6f}',
                           mag_board_y_uT=f'{mx_board[1]:.6f}',
                           mag_board_z_uT=f'{mx_board[2]:.6f}')
            elif stage.sensor=='bmp':
                row.update(bmp_ok=1,bmp_new=1,bmp_adc_p=f.bmp_raw_p,bmp_adc_t=f.bmp_raw_t,
                           bmp_temp_c=f'{f.bmp_tc100/100:.2f}',pressure_pa=f'{f.bmp():.2f}')
            assert len(row)==len(CSV_COLUMNS)
            self._write_row(row)

        if now - self.last_emit >= 0.12:
            self.last_emit = now
            # Drop disposable plot snapshots if Tk falls behind; never drop
            # phase completion or alerts. Calibration sample capture is separate.
            if self.events.qsize() < 60:
                self.events.put(('plot', {
                't': now, 'a': f.accel() if f.mpu_ok else (math.nan,)*3,
                'g': f.gyro() if f.mpu_ok else (math.nan,)*3,
                'm': mx_board if f.mag_ok else (math.nan,)*3,
                'p': pressure if pressure_valid else math.nan, 'h': altitude,
                'seq': f.seq, 'drop': self.seq_gaps, 'crc': self.decoder.bad_crc,
                'mpu': f.mpu_ok, 'mag': f.mag_ok, 'bmp': f.bmp_ok,
                'rate': self.frames_total/max(.001,(now-self.run_started)) if hasattr(self,'run_started') else 0,
                'mag_fresh':self.mag_total, 'bmp_fresh':self.bmp_total,
                'mag_sensor_type': f.mag_sensor_type,
                'mpu_temp': f.temp/340.0+36.53 if f.mpu_ok else math.nan,
                'bmp_temp': f.bmp_tc100/100.0 if f.bmp_ok else math.nan,
                'bmp_new': f.bmp_new,
                'altitude_status': self.altitude_status,
                'mag_point': self.pending_mag_plot,
                'version':f.version,
            }))
                self.pending_mag_plot = None

class Application:
    def __init__(self, tk, args, stage_list, axis_name, axis_matrix, output):
        from tkinter import ttk, messagebox
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        self.tk, self.ttk, self.msg = tk, ttk, messagebox
        self.args, self.stages, self.output = args, stage_list, output
        self.axis_name=axis_name
        self.default_storage = output is None
        self.output_dir = (SESSION_INCOMPLETE_DIR if self.default_storage else Path(output).parent)
        self.output_prefix = 'calibration' if self.default_storage else Path(output).stem
        self.output = None
        self.root=tk.Tk()
        self.root.title('IntelQCopter | 500 Hz IMU / 200 Hz QMC calibration')
        self.root.geometry('1240x800')
        self.root.minsize(1024,680)
        self.events=queue.Queue()
        self.worker=None
        self.stage_index=0
        self.finished=False
        self.capture_active=False
        self.session_active=False
        self.session_ready=False
        self.pending_begin=False
        self.analysis_running=False
        self.start_time=time.monotonic()
        self.lines=[]
        self.times=deque(maxlen=500)
        self.mag_points=deque(maxlen=8000)
        self.mag_plot_total=0
        self.series={key:deque(maxlen=500) for key in ('ax','ay','az','gx','gy','gz','mx','my','mz','pressure','altitude','mpu_temp','bmp_temp')}
        self.latency_times=deque(maxlen=7500)
        self.latency_series={f'{sensor}_{metric}':deque(maxlen=7500)
                             for sensor in ('MPU','QMC','BMP','LiDAR')
                             for metric in ('age','service')}
        self.last_plot=0
        self.live_plot_enabled=True
        self.last_3d_update=0.0
        self.last_magnetic_count=0
        self.mag_popup=None
        self.mag_popup_last_draw=0.0
        self.info=tk.StringVar(value='Connecting sensor telemetry...')
        self.saved_info=tk.StringVar(value=(
            'Accepted calibration samples only; complete sessions auto-save. LiDAR is excluded for this run.'))
        self.headline=tk.StringVar(value='')
        self.instruction=tk.StringVar(value='')
        self.capture_label=tk.StringVar(value=(
            f'New sessions: {SESSION_INCOMPLETE_DIR.relative_to(PROJECT_ROOT)}' if self.default_storage
            else f'New sessions: {self.output_dir}')+f'  ·  MAG axes: {axis_name}')
        self.altitude_status='Waiting for valid BMP280 pressure (30–120 kPa).'
        self.progress=tk.StringVar(value='')
        self.paused_status=tk.StringVar(value='')
        self.health=tk.StringVar(value='Opening sensor stream…')
        self.latency_status=tk.StringVar(value='Waiting for sensor telemetry.')
        self.latency_summary=tk.StringVar(value='UNAVAILABLE — per-sensor age and service time require measured firmware v4 fields.')
        self._latency_status_last=''
        self.lidar_port_var=tk.StringVar(value='')
        self.lidar_status_var=tk.StringVar(value=(
            'LiDAR is excluded from this session.' if self.args.skip_lidar else
            'Connect the TOFSense-F2 Mini for range phases 13–16.'))
        self._lidar_last_status=''
        self.geometry_config_path=ROOT/'lidar_geometry_config.json'
        self.geometry_vars={
            'rx':tk.StringVar(value='0.000000'), 'ry':tk.StringVar(value='0.000000'),
            'rz':tk.StringVar(value='-0.026658'), 'roll':tk.StringVar(value='0'),
            'pitch':tk.StringVar(value='0'), 'yaw':tk.StringVar(value='0'),
            'distance':tk.StringVar(value=''),
        }
        self.geometry_result=tk.StringVar(value='Enter a measured slant range and attitude, then calculate.')
        self._load_geometry_config()

        top=ttk.Frame(self.root,padding=8);top.pack(fill='x')
        ttk.Label(top,text='IntelQCopter | 500 Hz IMU · 200 Hz QMC calibration',font=('Segoe UI',15,'bold')).pack(anchor='w')
        ttk.Label(top,textvariable=self.capture_label,font=('Segoe UI',8)).pack(anchor='w')
        ttk.Label(top,textvariable=self.info,font=('Segoe UI',8)).pack(anchor='w',pady=(3,0))
        ttk.Separator(self.root,orient='horizontal').pack(fill='x')
        phase=ttk.Frame(self.root,padding=8);phase.pack(fill='x')
        ttk.Label(phase,textvariable=self.headline,font=('Segoe UI',12,'bold')).pack(anchor='w')
        ttk.Label(phase,textvariable=self.instruction,justify='left',wraplength=1100,font=('Segoe UI',8)).pack(anchor='w',pady=3)
        self.bar=ttk.Progressbar(phase,orient='horizontal',mode='determinate',maximum=100,length=900)
        self.bar.pack(fill='x',pady=4)
        ttk.Label(phase,textvariable=self.progress,font=('Consolas',10)).pack(anchor='w')
        buttons=ttk.Frame(phase);buttons.pack(fill='x',pady=8)
        self.begin=ttk.Button(buttons,text='ENTER — Start current phase',command=self.begin_phase)
        self.begin.pack(side='left',padx=(0,8))
        self.pause=ttk.Button(buttons,text='Pause',command=self.toggle_pause,state='disabled')
        self.pause.pack(side='left',padx=8)
        self.redo=ttk.Button(buttons,text='Redo phase',command=self.redo_phase)
        self.redo.pack(side='left',padx=5)
        self.save_now=ttk.Button(buttons,text='Save & begin new calibration',command=self.save_current)
        self.save_now.pack(side='left',padx=5)
        self.stop=ttk.Button(buttons,text='Stop / close',command=self.close)
        self.stop.pack(side='left',padx=8)
        self.calc=ttk.Button(buttons,text='Calibrate selected CSV files...',command=self.calculate)
        self.calc.pack(side='left',padx=8)
        self.graph_button=ttk.Button(buttons,text='Pause live graphs',command=self.toggle_live_graphs)
        self.graph_button.pack(side='left',padx=8)
        self.large_3d_button=ttk.Button(buttons,text='Enlarge 3D magnetometer',command=self.open_3d)
        self.large_3d_button.pack(side='left',padx=5)
        extra=ttk.Frame(phase);extra.pack(fill='x',pady=(0,4))
        # Magnetometer quality assessment (previously separate button) is now
        # handled through the full integrated calibration; the isolated MAG-only
        # probe is deprecated per user direction.
        self.combine_button=ttk.Button(extra,text='Combine CSV + calibration JSON...',command=self.combine_calibrations)
        self.combine_button.pack(side='left',padx=(0,8))
        self.lidar_analysis_button=ttk.Button(extra,text='Analyze LiDAR range CSV...',command=self.analyze_lidar_csv)
        if not self.args.skip_lidar:
            self.lidar_analysis_button.pack(side='left',padx=(0,8))
        ttk.Label(extra,text='Withheld-data fit, 3D coverage and bootstrap; superseded redo attempts are excluded.',
                  foreground='#5A6470').pack(side='left')
        lidar_row=ttk.Frame(phase)
        ttk.Label(lidar_row,text='F2 adapter COM (one TOFSense-F2 Mini):').pack(side='left')
        self.lidar_port_combo=ttk.Combobox(lidar_row,textvariable=self.lidar_port_var,
                                           width=10,state='readonly')
        self.lidar_port_combo.pack(side='left',padx=4)
        ttk.Button(lidar_row,text='Refresh',command=self.refresh_lidar_ports).pack(side='left',padx=3)
        ttk.Button(lidar_row,text='Connect F2',command=self.connect_lidar_from_gui).pack(side='left',padx=3)
        ttk.Label(lidar_row,textvariable=self.lidar_status_var,foreground='#285884',
                  wraplength=620,justify='left').pack(side='left',padx=8)
        if not self.args.skip_lidar:
            lidar_row.pack(fill='x',pady=(0,3))
        ttk.Label(phase,textvariable=self.health,foreground='#285884').pack(anchor='w')
        ttk.Label(phase,textvariable=self.saved_info,foreground='#186A58').pack(anchor='w')

        # Calling constrained_layout on every draw is expensive with a 3D axis.
        # Static layout drastically reduces GUI latency on Windows laptops.
        notebook=ttk.Notebook(self.root)
        notebook.pack(fill='both',expand=True,padx=10,pady=6)
        self.tab_live=ttk.Frame(notebook)
        self.tab_thermal=ttk.Frame(notebook)
        self.tab_latency=ttk.Frame(notebook)
        self.tab_geometry=ttk.Frame(notebook,padding=10)
        notebook.add(self.tab_live,text='Live motion, magnetic field & altitude')
        notebook.add(self.tab_thermal,text='MPU6050 / BMP280 temperatures')
        notebook.add(self.tab_latency,text='Sensor latency')
        if not self.args.skip_lidar:
            notebook.add(self.tab_geometry,text='LiDAR geometry')
        self.notebook=notebook
        fig=Figure(figsize=(14,8.0),dpi=95,constrained_layout=False)
        fig.subplots_adjust(left=0.07,right=0.97,bottom=0.12,top=0.91,wspace=0.30,hspace=0.48)
        ax_acc=fig.add_subplot(2,3,1)
        ax_gyro=fig.add_subplot(2,3,2)
        ax_3d=fig.add_subplot(2,3,3,projection='3d')
        ax_mag=fig.add_subplot(2,3,4)
        ax_pressure=fig.add_subplot(2,3,5)
        ax_alt=fig.add_subplot(2,3,6)
        self.mag_3d=ax_3d
        ax_3d.set_title('3D MAG cloud',fontsize=10,pad=4)
        for axis in ('X','Y','Z'):
            getattr(ax_3d,'set_'+axis.lower()+'label')(axis+' µT',fontsize=9)
        self.mag_scatter=ax_3d.scatter([],[],[],s=6,alpha=.48,c='#6243B7',depthshade=False,edgecolors='w',linewidth=.3)
        self._mag_triads=configure_magnetic_axes(ax_3d,[],enlarged=False)
        self.charts=[]
        for ax,title,keys,unit,minimum_span,initial_limits in [
            (ax_acc,'Accel · ±8 g expected',('ax','ay','az'),'m/s²',2.0,
             (-MPU_ACCEL_FULL_SCALE_MPS2,MPU_ACCEL_FULL_SCALE_MPS2)),
            (ax_gyro,'Gyro · ±500°/s expected',('gx','gy','gz'),'°/s',10.0,
             (-MPU_GYRO_FULL_SCALE_DPS,MPU_GYRO_FULL_SCALE_DPS)),
            (ax_mag,'MAG · raw axes',('mx','my','mz'),'µT',10.0,(-100,100)),
            (ax_pressure,'BMP pressure',('pressure',),'Pa',100.0,(30000,120000)),
            (ax_alt,'Relative barometric height',('altitude',),'m',0.20,(-1,1))]:
            ax.set_title(title,fontsize=9,pad=3)
            ax.set_ylabel(unit,fontsize=8,labelpad=2)
            ax.set_xlabel('' if ax in (ax_acc,ax_gyro) else 'Time (s)',fontsize=8,labelpad=1)
            ax.tick_params(labelsize=7,pad=1)
            ax.grid(alpha=0.25)
            ax.set_ylim(*initial_limits)
            ax.ticklabel_format(axis='y',style='plain',useOffset=False)
            lines=[]
            for key in keys:
                line,=ax.plot([],[],label=key,linewidth=1.1)
                lines.append((key,line))
            ax.legend(loc='upper right',fontsize=8)
            self.charts.append((ax,lines,minimum_span,initial_limits))
        # Show the sensor's configured measurement range after data arrives.
        # Dynamic autoscaling made out-of-range conversion errors look plausible.
        self.fixed_sensor_limits={
            ax_acc:(-MPU_ACCEL_FULL_SCALE_MPS2,MPU_ACCEL_FULL_SCALE_MPS2),
            ax_gyro:(-MPU_GYRO_FULL_SCALE_DPS,MPU_GYRO_FULL_SCALE_DPS),
        }
        self.altitude_annotation=ax_alt.text(
            .5,.5,self.altitude_status,transform=ax_alt.transAxes,
            ha='center',va='center',fontsize=8,color='#7A4B00',wrap=True)
        self.canvas=FigureCanvasTkAgg(fig,master=self.tab_live)
        self.canvas.draw(); self.canvas.get_tk_widget().pack(fill='both',expand=True,padx=3,pady=3)
        # Temperature is recorded at native sensor rates; thermal graphs use
        # a separate tab so the original six plots remain readable.
        ftemp=Figure(figsize=(12,5),dpi=90)
        ftemp.subplots_adjust(left=.075,right=.96,bottom=.12,top=.90,wspace=.27)
        self.temp_plot=ftemp.add_subplot(121)
        self.temp_plot.set_title('MPU6050 and BMP280 temperatures')
        self.temp_plot.set_xlabel('Elapsed (s)');self.temp_plot.set_ylabel('Temperature (°C)')
        self.temp_plot.grid(alpha=.25)
        self.temp_lines=[]
        for key,label,color in [('mpu_temp','MPU die','tab:red'),('bmp_temp','BMP die','tab:blue')]:
            line,=self.temp_plot.plot([],[],label=label,color=color,lw=1.5)
            self.temp_lines.append((key,line))
        self.temp_plot.legend(loc='upper right',fontsize=9)
        self.thermal_scatter=ftemp.add_subplot(122)
        self.thermal_scatter.set_title('BMP pressure vs sensor temperature (diagnostic)')
        self.thermal_scatter.set_xlabel('BMP temperature (°C)')
        self.thermal_scatter.set_ylabel('Pressure (Pa)')
        self.thermal_scatter.grid(alpha=.25)
        self.thermal_pressure_t=[]
        self.thermal_pressure_p=[]
        self.temperature_canvas=FigureCanvasTkAgg(ftemp,master=self.tab_thermal)
        self.temperature_canvas.draw()
        self.temperature_canvas.get_tk_widget().pack(fill='both',expand=True,padx=3,pady=3)
        ttk.Label(self.tab_latency,textvariable=self.latency_status,justify='left',
                  wraplength=1150,foreground='#285884').pack(fill='x',padx=8,pady=(8,2))
        ttk.Label(self.tab_latency,textvariable=self.latency_summary,justify='left',
                  wraplength=1150,font=('Consolas',8)).pack(fill='x',padx=8,pady=(0,3))
        latency_fig=Figure(figsize=(12,7),dpi=90)
        latency_fig.subplots_adjust(left=.08,right=.97,bottom=.09,top=.92,wspace=.30,hspace=.38)
        self.latency_axes=[]
        self.latency_lines={}
        for index,sensor in enumerate(('MPU','QMC','BMP','LiDAR'),start=1):
            ax=latency_fig.add_subplot(2,2,index)
            ax.set_title(f'{sensor}: sample age and per-tick service',fontsize=10)
            ax.set_xlabel('Elapsed time (s)',fontsize=8)
            ax.set_ylabel('Microseconds (µs)',fontsize=8)
            ax.tick_params(labelsize=8)
            ax.grid(alpha=.25)
            age,=ax.plot([],[],label='sample age',lw=1.1)
            service,=ax.plot([],[],label='sensor service',lw=1.1)
            empty=ax.text(.5,.5,'WAITING FOR MEASURED TIMING DATA',
                          transform=ax.transAxes,ha='center',va='center',
                          fontsize=9,color='#6b4f24',wrap=True)
            ax.legend(loc='upper right',fontsize=7)
            self.latency_axes.append(ax)
            self.latency_lines[sensor]=(age,service,empty)
        self.latency_canvas=FigureCanvasTkAgg(latency_fig,master=self.tab_latency)
        self.latency_canvas.draw()
        self.latency_canvas.get_tk_widget().pack(fill='both',expand=True,padx=5,pady=5)
        self._build_geometry_tab()
        self.root.bind('<Return>',self.on_enter)
        self.root.protocol('WM_DELETE_WINDOW',self.close)
        self.update_stage()
        self.refresh_lidar_ports()
        self.root.after(120,self.pump)

    def on_enter(self,event):
        self.begin_phase()

    def _available_lidar_ports(self):
        try:
            from serial.tools import list_ports
            return [x.device for x in list_ports.comports()
                    if x.device.casefold() != str(self.args.port).casefold()]
        except ImportError:
            return []

    def _load_geometry_config(self):
        if not self.geometry_config_path.exists():
            return
        try:
            obj=json.loads(self.geometry_config_path.read_text(encoding='utf-8'))
            vector=obj.get('sensor_position_from_com_body_m',{})
            for key,axis in (('rx','x'),('ry','y'),('rz','z')):
                value=float(vector[axis])
                if math.isfinite(value):
                    self.geometry_vars[key].set(f'{value:.6f}')
        except (OSError,ValueError,KeyError,TypeError,json.JSONDecodeError):
            self.geometry_result.set('Geometry config could not be read; using preliminary defaults.')

    def _build_geometry_tab(self):
        from tkinter import ttk
        ttk.Label(self.tab_geometry,
            text='Body axes: +X forward, +Y left, +Z up. LiDAR beam: body −Z.',
            font=('Segoe UI',10,'bold')).pack(anchor='w')
        ttk.Label(self.tab_geometry,
            text='The default COM lever arm (0, 0, −0.026658) m is a preliminary estimate. Measure the assembled aircraft. Range D is slant distance from the optical reference; this worksheet is not an altitude estimator and assumes the entered attitude is body-to-world ZYX.',
            wraplength=1100,justify='left').pack(anchor='w',pady=(3,10))
        frame=ttk.Frame(self.tab_geometry);frame.pack(anchor='w',fill='x')
        fields=(('Sensor x from COM (m)','rx'),('Sensor y from COM (m)','ry'),
                ('Sensor z from COM (m)','rz'),('Roll (deg)','roll'),
                ('Pitch (deg)','pitch'),('Yaw (deg)','yaw'),('Slant range D (m)','distance'))
        for index,(label,key) in enumerate(fields):
            row=index//4;col=(index%4)*2
            ttk.Label(frame,text=label).grid(row=row,column=col,sticky='w',padx=(0,4),pady=5)
            ttk.Entry(frame,textvariable=self.geometry_vars[key],width=12).grid(row=row,column=col+1,sticky='w',padx=(0,14),pady=5)
        actions=ttk.Frame(self.tab_geometry);actions.pack(anchor='w',pady=8)
        ttk.Button(actions,text='Calculate geometry',command=self.calculate_geometry).pack(side='left',padx=(0,8))
        ttk.Button(actions,text='Save offset configuration',command=self.save_geometry_config).pack(side='left')
        ttk.Label(self.tab_geometry,textvariable=self.geometry_result,justify='left',
                  wraplength=1100,font=('Consolas',10)).pack(anchor='w',pady=8)
        ttk.Label(self.tab_geometry,
            text='Ground-plane, slope, obstacle, footprint, attitude uncertainty, timestamp and innovation checks remain estimator responsibilities. Do not zero navigation altitude from a new range or treat a beam hit as terrain under COM when tilted.',
            wraplength=1100,justify='left',foreground='#7A4B00').pack(anchor='w',pady=(4,0))

    def calculate_geometry(self):
        try:
            values={key:float(var.get()) for key,var in self.geometry_vars.items()}
            if not all(math.isfinite(v) for v in values.values()):
                raise ValueError('all entries must be finite')
            if values['distance'] < 0:
                raise ValueError('slant range must be non-negative')
        except ValueError as exc:
            self.geometry_result.set(f'Invalid geometry input: {exc}')
            return
        roll,pitch,yaw=(math.radians(values[key]) for key in ('roll','pitch','yaw'))
        cr,sr=math.cos(roll),math.sin(roll)
        cp,sp=math.cos(pitch),math.sin(pitch)
        cy,sy=math.cos(yaw),math.sin(yaw)
        rotation=((cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr),
                  (sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr),
                  (-sp,cp*sr,cp*cr))
        def rotate(vector):
            return tuple(sum(rotation[i][j]*vector[j] for j in range(3)) for i in range(3))
        r_sensor=(values['rx'],values['ry'],values['rz'])
        beam=(0.0,0.0,-1.0)
        rotated_sensor=rotate(r_sensor)
        rotated_beam=rotate(beam)
        h_sensor=-values['distance']*rotated_beam[2]
        h_com=-rotated_sensor[2]+h_sensor
        footprint=rotate(tuple(r_sensor[i]+values['distance']*beam[i] for i in range(3)))
        self.geometry_result.set(
            f'H_sensor = {h_sensor:.6f} m\n'
            f'H_COM above the beam-hit point = {h_com:.6f} m\n'
            f'Beam-hit displacement from COM: x={footprint[0]:+.6f}, '
            f'y={footprint[1]:+.6f}, z={footprint[2]:+.6f} m\n'
            f'Formula: H_COM = −(R r_s)_z − D(R d_s)_z; D is slant range.')

    def save_geometry_config(self):
        try:
            vector={axis:float(self.geometry_vars[key].get())
                    for key,axis in (('rx','x'),('ry','y'),('rz','z'))}
            if not all(math.isfinite(v) for v in vector.values()):
                raise ValueError('offset values must be finite')
            obj={'schema':'IntelQCopter LiDAR mounting geometry v1',
                 'body_axes':{'x':'forward','y':'left','z':'up'},
                 'sensor_position_from_com_body_m':vector,
                 'measurement_state':'preliminary_unverified',
                 'source':'user-provided preliminary F450 geometry; verify by physical measurement'}
            self.geometry_config_path.write_text(json.dumps(obj,indent=2)+'\n',encoding='utf-8')
            self.geometry_result.set(f'Saved configurable preliminary offset to {self.geometry_config_path.name}; verify the physical dimensions before estimator use.')
        except (OSError,ValueError) as exc:
            self.geometry_result.set(f'Could not save geometry configuration: {exc}')

    def refresh_lidar_ports(self):
        if self.args.skip_lidar:
            self.lidar_status_var.set('LiDAR is excluded from this calibration session.')
            return
        choices = self._available_lidar_ports()
        self.lidar_port_combo['values'] = choices
        if not self.lidar_port_var.get().strip() and len(choices) == 1:
            self.lidar_port_var.set(choices[0])
            self.args.lidar_port = choices[0]
        elif self.lidar_port_var.get().strip() not in choices:
            self.lidar_port_var.set('')
            self.args.lidar_port = ''
        if choices:
            self.lidar_status_var.set(
                'Single TOFSense-F2 Mini; choose its separate USB-UART port: '+', '.join(choices))
        else:
            self.lidar_status_var.set(
                'F2 required for phases 13–16; no separate USB-UART port found. '
                'Connect the F2 adapter or use firmware v4 forwarding.')

    def connect_lidar_from_gui(self):
        port = self.lidar_port_var.get().strip()
        if not self.worker or not self.worker.started.is_set() or self.worker.error:
            self.lidar_status_var.set('Start the ESP sensor stream first, then connect a separate LiDAR UART if needed.')
            return
        ok, note = self.worker.connect_lidar(port)
        self.lidar_status_var.set(note)
        if ok:
            self.args.lidar_port = port
        else:
            self.msg.showwarning('TOFSense-F2 connection',note)

    def _draw_magnetic_axes(self, points):
        """Update light grid panes, numeric µT ticks and the positive-axis triad."""
        for artist in self._mag_triads:
            try: artist.remove()
            except ValueError: pass
        self._mag_triads=configure_magnetic_axes(self.mag_3d,points,enlarged=False)

    def open_3d(self):
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg,NavigationToolbar2Tk
        if self.mag_popup is not None and self.mag_popup.winfo_exists():
            self.mag_popup.lift()
            return
        win=self.tk.Toplevel(self.root)
        win.title('IntelQCopter — QMC5883P 3D magnetic cloud | X red · Y green · Z blue')
        win.geometry('860x740')
        fig=Figure(figsize=(9,7),dpi=100)
        ax=fig.add_subplot(111,projection='3d')
        ax.set_title('Magnetic measurements in display frame (uncalibrated)')
        ax.set_xlabel('X (µT)');ax.set_ylabel('Y (µT)');ax.set_zlabel('Z (µT)')
        ax.set_box_aspect((1,1,1))
        canvas=FigureCanvasTkAgg(fig,master=win)
        canvas.get_tk_widget().pack(fill='both',expand=True)
        toolbar=NavigationToolbar2Tk(canvas,win,pack_toolbar=False)
        toolbar.update();toolbar.pack(fill='x')
        self.mag_popup=win
        self.big_mag_axis=ax
        self.big_mag_canvas=canvas
        self.mag_popup_last_draw=0.0
        def close_window():
            self.mag_popup=None
            win.destroy()
        win.protocol('WM_DELETE_WINDOW',close_window)
        self.redraw_big3d()

    def redraw_big3d(self):
        if self.mag_popup is None or not self.mag_popup.winfo_exists():return
        ax=self.big_mag_axis
        elev,azim=ax.elev,ax.azim
        ax.clear()
        ax.view_init(elev=elev,azim=azim)
        ax.set_title('Magnetic field: raw rotated-sensor measurements | µT grid')
        pts=list(self.mag_points)
        stride=max(1,len(pts)//3500)
        pts=pts[::stride]
        if pts:
            x,y,z=zip(*pts)
            ax.scatter(x,y,z,s=5,c='#6243B7',alpha=.42,depthshade=False)
        configure_magnetic_axes(ax,pts,enlarged=True)
        self.big_mag_canvas.draw_idle()

    def save_current(self):
        if not self.worker or not self.worker.started.is_set() or self.worker.error:
            self.msg.showwarning('Recorder unavailable','Connect the serial recorder first.')
            return
        if self.finished:
            self._start_new_session()
            return
        if not self.session_active:
            self._start_new_session()
            return
        missing=self._missing_stage_descriptions()
        if not missing:
            self.worker.end_stage()
            self.stage_index=len(self.stages)
            self.update_stage()
            return
        choice=self.msg.askyesnocancel(
            'Begin a new calibration?',
            'The current calibration is incomplete.\n\n'
            'Yes: save it as incomplete, with missing phases listed, then begin a new session.\n'
            'No: delete this session data and begin a new session.\n'
            'Cancel: return to the current calibration.')
        if choice is None:
            return
        self.worker.end_stage()
        self.capture_active=False
        self.pause.config(text='Pause',state='disabled')
        if choice:
            self.worker.finalize_session('incomplete',missing)
        else:
            self.worker.discard_session()
        self._start_new_session()

    def _new_session_paths(self):
        stamp=datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')
        stem=f'{self.output_prefix}_{stamp}'
        if self.default_storage:
            incomplete=SESSION_INCOMPLETE_DIR/f'{stem}_incomplete.csv'
            complete=SESSION_COMPLETE_DIR/f'{stem}_complete.csv'
        else:
            incomplete=self.output_dir/f'{stem}_incomplete.csv'
            complete=self.output_dir/f'{stem}_complete.csv'
        return incomplete,complete

    def _reset_session_view(self):
        self.stage_index=0
        self.finished=False
        self.capture_active=False
        self.mag_points.clear()
        self.times.clear()
        self.latency_times.clear()
        for values in self.series.values():values.clear()
        for values in self.latency_series.values():values.clear()
        self.thermal_pressure_t.clear()
        self.thermal_pressure_p.clear()
        self.mag_plot_total=0
        self.start_time=time.monotonic()
        self.bar['value']=0

    def _start_new_session(self):
        started=datetime.now(timezone.utc).isoformat(timespec='milliseconds')
        incomplete,complete=self._new_session_paths()
        self.worker.start_session(incomplete,complete,started)
        self.output=incomplete
        self.session_active=True
        self.session_ready=False
        self._reset_session_view()
        self.capture_label.set(f'Session: {incomplete}  ·  MAG axes: {self.axis_name}')
        self.saved_info.set('New session ready. A CSV is created only after its first accepted sample.')
        self.info.set('New calibration session ready. Position the board and press ENTER to begin.')
        self.update_stage()

    def _missing_stage_descriptions(self):
        with self.worker.lock:
            counts=dict(self.worker.stage_counts)
        return [f'{stage.pose}/{stage.mag_stage} {counts.get((stage.pose,stage.mag_stage),0)}/{stage.target}'
                for stage in self.stages
                if counts.get((stage.pose,stage.mag_stage),0)<stage.target]

    def update_stage(self):
        if self.stage_index>=len(self.stages):
            self.finished=True
            self.headline.set('COMPLETED — all calibration phases collected')
            self.instruction.set('The session is being finalized automatically. Review the saved CSV before using its calibration.')
            self.progress.set('Saving complete session…')
            self.bar['value']=100
            self.begin.config(state='disabled')
            self.worker.finalize_session('complete')
            self.info.set('All phases complete. Saving the calibration session automatically.')
            return
        s=self.stages[self.stage_index]
        self.headline.set(f'{self.stage_index+1}/{len(self.stages)}  ·  {s.pose} / {s.mag_stage}   ·   {s.target:,} valid {s.sensor.upper()} readings')
        self.instruction.set(s.prompt+'\nSet position, press ENTER, then hold steady after the '+f'{self.args.settle_seconds:g}'+' s countdown.')
        self.progress.set('0 / {:,} — WAITING FOR ENTER'.format(s.target))
        self.bar['value']=0
        self.begin.config(state='normal' if self.worker and self.worker.started.is_set() and not self.worker.error else 'disabled')
        self.capture_active=False
        self.pause.config(text='Pause',state='disabled')

    def redo_phase(self):
        """Restart the current stage, or return to the last completed stage.

        A new phase_attempt is allocated on Enter. Offline analysis retains
        only the latest attempt for each pose in the same session, so previously
        recorded bad poses are never blended in.
        """
        if not self.worker or self.finished:
            return
        index = self.stage_index if self.capture_active or self.stage_index == 0 else self.stage_index-1
        if not self.msg.askyesno('Redo calibration stage',
             f'Record {self.stages[index].pose}/{self.stages[index].mag_stage} again?\n'
             'Earlier samples remain in CSV for auditing but are superseded during analysis.\n'
             f'A new {self.args.settle_seconds:g}-second countdown begins after Enter.'):
            return
        self.worker.end_stage()
        self.stage_index=index
        self.capture_active=False
        self.finished=False
        self.update_stage()
        self.info.set('READY TO REDO: position the drone; press ENTER for a new attempt.')

    def toggle_live_graphs(self):
        self.live_plot_enabled=not self.live_plot_enabled
        self.graph_button.config(text=('Pause live graphs' if self.live_plot_enabled else 'Resume live graphs'))
        if self.live_plot_enabled:
            self.last_plot=0.0

    def toggle_pause(self):
        if not self.capture_active or not self.worker:
            return
        if not self.worker.paused and self.worker.pause_stage():
            self.pause.config(text='Resume')
            self.info.set('PAUSED: sensor stream continues; CSV capture and sample counter are suspended.')
        elif self.worker.resume_stage():
            self.pause.config(text='Pause')
            self.info.set('RESUMED: 3-second countdown before counting real sensor samples. No pose gate.')

    def begin_phase(self):
        if self.finished or self.capture_active or not self.worker or not self.worker.started.is_set() or self.worker.error:
            return
        if not self.session_active:
            self.pending_begin=True
            self._start_new_session()
            return
        if not self.session_ready:
            self.pending_begin=True
            self.info.set('Preparing a new session; the phase will start as soon as its session is ready.')
            return
        s=self.stages[self.stage_index]
        # Always honor ENTER: start a countdown, then count only actual sensor
        # acquisitions. Never block because of attitude, stillness, or missing
        # sensor. Missing sensors remain zero-count (never manufacture data).
        if self.worker.start_stage(s):
            self.capture_active=True
            self.pause.config(text='Pause',state='normal')
            self.begin.config(state='disabled')
            self.info.set(f'PREPARE {s.pose} / {s.mag_stage}: {self.args.settle_seconds:g}-second countdown started. Record begins automatically.')
            self.progress.set(f'0 / {s.target:,} — PREPARING ({self.args.settle_seconds:g} s)')

    def _set_latency_status(self,text):
        if text != self._latency_status_last:
            self._latency_status_last=text
            self.latency_status.set(text)

    def pump(self):
        if self.worker:
            drained=0
            while drained<2000:
                try:sample_time,ages,service,fresh_mask,errors,deadline_count=self.worker.latency_samples.get_nowait()
                except queue.Empty:break
                drained+=1
                self.latency_times.append(sample_time-self.start_time)
                for index,sensor in enumerate(('MPU','QMC','BMP','LiDAR')):
                    age=ages[index] if ages else None
                    duration=service[index] if service else None
                    self.latency_series[f'{sensor}_age'].append(float(age) if age is not None else math.nan)
                    self.latency_series[f'{sensor}_service'].append(float(duration) if duration is not None else math.nan)
                has_measurements=any(value is not None for group in (ages,service)
                                     if group for value in group)
                if has_measurements:
                    self._set_latency_status(
                        f'Measured sensor age/service: MPU={ages[0] if ages and ages[0] is not None else "--"}µs, '
                        f'QMC={ages[1] if ages and ages[1] is not None else "--"}µs, '
                        f'BMP={ages[2] if ages and ages[2] is not None else "--"}µs, '
                        f'LiDAR={ages[3] if ages and ages[3] is not None else "--"}µs; '
                        f'errors={errors}, deadline_overruns={deadline_count}.')
                else:
                    self._set_latency_status(
                        'NO SENSOR TIMING DATA: firmware does not send per-sensor age/service fields. '
                        'Age/service plots will remain empty until firmware v4 is installed.')
            frame=self.worker.last_frame
            if frame is not None and frame.version < 4:
                self._set_latency_status(
                    f'NOT MEASURED: firmware v{frame.version} lacks per-sensor age/service fields. '
                    'The 2 ms control loop period is NOT sensor latency; latency plots stay empty.')
        if self.worker and self.worker.lidar_reader is not None:
            reader=self.worker.lidar_reader
            state=('ERROR: '+reader.error if reader.error else
                   f'LiDAR connected on {reader.port} at 921600 baud' if reader.connected.is_set() else
                   f'Opening LiDAR serial {reader.port}...')
            if state != self._lidar_last_status:
                self._lidar_last_status=state
                self.lidar_status_var.set(state)
        elif self.worker and self.worker.lidar_inband_seen:
            state='LiDAR samples received via ESP32-S3 UART telemetry (firmware v4).'
            if state != self._lidar_last_status:
                self._lidar_last_status=state
                self.lidar_status_var.set(state)
        elif self.worker and self.worker.lidar_port:
            state=f'LiDAR port configured: {self.worker.lidar_port} — not yet connected.'
            if state != self._lidar_last_status:
                self._lidar_last_status=state
                self.lidar_status_var.set(state)
        processed=0
        while processed<1000:
            try:kind,payload=self.events.get_nowait()
            except queue.Empty:break
            processed+=1
            if kind=='status':
                self.info.set(payload)
                self.begin.config(state='normal')
            elif kind=='error':
                self.info.set('SERIAL ERROR: '+payload)
                self.begin.config(state='disabled')
                self.msg.showerror('Serial connection error',payload)
            elif kind=='stopped':
                self.begin.config(state='disabled')
            elif kind=='countdown':
                stage, remaining = payload
                if self.capture_active and not self.finished and stage == self.stages[self.stage_index]:
                    self.progress.set(f'0 / {stage.target:,} — STARTING IN {remaining:.1f} s')
            elif kind=='capture_started':
                stage = payload
                if self.capture_active and not self.finished and stage == self.stages[self.stage_index]:
                    self.info.set(f'NOW RECORDING {stage.pose}/{stage.mag_stage}: hold pose steady; rotate only for MAG turns.')
                    self.progress.set(f'0 / {stage.target:,} — RECORDING')
                    try: self.root.bell()
                    except Exception: pass
            elif kind=='hint':
                if self.capture_active:
                    self.info.set(payload)
            elif kind=='progress':
                if self.capture_active and not self.finished:
                    s=self.stages[self.stage_index]
                    self.progress.set(f'{payload:,} / {s.target:,} fresh valid samples')
                    self.bar['value']=min(100,100*payload/s.target)
                    self.root.update_idletasks()
                    self.root.update_idletasks()
            elif kind=='stage_done':
                s,count=payload
                # Ignore duplicate/stale completion events from an older phase.
                if self.finished or s != self.stages[self.stage_index]:
                    continue
                # Match the existing countdown-finished cue: make each
                # successfully completed calibration phase audible.
                try: self.root.bell()
                except Exception: pass
                self.capture_active=False
                self.pause.config(text='Pause',state='disabled')
                self.stage_index+=1
                self.update_stage()
                if not self.finished:
                    self.info.set(f'Completed {s.pose}/{s.mag_stage}: {count:,} valid readings. Position for next phase and press Enter.')
                else:
                    self.info.set(f'All {len(self.stages)} phases complete. Finalizing the session CSV.')
            elif kind=='lidar_status':
                self.lidar_status_var.set(payload)
            elif kind=='lidar_sample':
                if payload.get('valid') and math.isfinite(payload.get('distance_m',math.nan)):
                    self.geometry_vars['distance'].set(f"{payload['distance_m']:.6f}")
                    self.calculate_geometry()
            elif kind=='session_saved':
                status,path,rows,missing=payload
                if path is None:
                    self.saved_info.set('No CSV created: this session contained no accepted samples.')
                else:
                    self.saved_info.set(f'{status.upper()} session saved: {path} ({rows:,} samples + session metadata).')
                    if self.output is not None and self.output.name.replace('_incomplete.csv','_complete.csv') == path.name:
                        self.output=path
                        self.capture_label.set(f'Saved: {path}')
                if status=='complete':
                    self.info.set(f'Calibration complete and saved: {path}' if path else
                                  'Calibration completed with no accepted samples; no CSV was created.')
            elif kind=='session_started':
                self.session_active=True
                self.session_ready=True
                if self.pending_begin:
                    self.pending_begin=False
                    self.begin_phase()
            elif kind=='session_discarded':
                path,rows=payload
                self.saved_info.set('Previous incomplete session data discarded; the new session is ready.')
            elif kind=='analysis_done':
                ok, report=payload
                self.analysis_running=False
                for button in (self.calc,self.combine_button,self.lidar_analysis_button):
                    button.config(state='normal')
                if ok:self.msg.showinfo('Analysis finished',report[-5000:])
                else:self.msg.showerror('Calibration failed',report[-5000:])
            elif kind=='plot':
                self.add_point(payload)
        now=time.monotonic()
        if (self.mag_popup is not None and self.mag_popup.winfo_exists() and
                now-self.mag_popup_last_draw >= 2.0 and self.mag_points):
            self.mag_popup_last_draw=now
            self.redraw_big3d()
        if self.live_plot_enabled and now-self.last_plot>=0.90:
            self.last_plot=now
            selected=self.notebook.index('current')
            if selected == 0 and self.times:
                self.redraw()
            elif selected == 1 and self.times:
                self.redraw_thermal()
            elif selected == 2:
                self.redraw_latency()
            if self.worker:
                w=self.worker
                frame=w.last_frame
                age=(time.monotonic()-w.last_byte_at) if w.last_byte_at else time.monotonic()-self.start_time
                if not w.bytes_total and age>3:
                    self.health.set(f'NO SENSOR DATA on {w.port if not w.wifi else "Wi-Fi"}. Confirm the ESP firmware stream and transport.')
                elif w.bytes_total and not w.frames_total:
                    self.health.set(f'Bytes={w.bytes_total:,}, valid packets=0 | sync={w.decoder.bad_sync:,} CRC={w.decoder.bad_crc:,} version={w.decoder.bad_version:,}. Check transport, packet size/version, and CRC.')
                elif frame:
                    self.health.set(f'{w.frames_total:,} packets | MPU {w.mpu_total:,} | MAG ok {w.mag_valid_total:,} / fresh {w.mag_total:,} / plotted {self.mag_plot_total:,} | BMP {w.bmp_total:,} | flags 0x{frame.flags:02X} | CRC {w.decoder.bad_crc:,} | gaps {w.seq_gaps:,}')
                elif w.bytes_total:
                    self.health.set(f'UART receiving {w.bytes_total:,} bytes; waiting for a complete valid packet…')
        self.root.after(120,self.pump)

    def add_point(self,p):
        now=p['t']-self.start_time
        self.altitude_status=p.get('altitude_status',self.altitude_status)
        mag_point=p.get('mag_point')
        if mag_point is not None:
            self.mag_points.append(mag_point)
            self.mag_plot_total+=1
        # An ESP32 can send valid packets containing all-invalid sensors.
        # Do not advance blank axes when there are no measurements.
        if not any((p['mpu'],p['mag'],p['bmp'])):
            return
        self.times.append(p['t']-self.start_time)
        values={**dict(zip(('ax','ay','az'),p['a'])),**dict(zip(('gx','gy','gz'),p['g'])),
                **dict(zip(('mx','my','mz'),p['m'])), 'pressure':p['p'],'altitude':p['h'],
                'mpu_temp':p['mpu_temp'],'bmp_temp':p['bmp_temp']}
        for key,d in self.series.items():
            d.append(values[key])
        if p.get('bmp_new') and math.isfinite(p['bmp_temp']) and math.isfinite(p['p']):
            self.thermal_pressure_t.append(p['bmp_temp'])
            self.thermal_pressure_p.append(p['p'])
            if len(self.thermal_pressure_t)>2000:
                del self.thermal_pressure_t[:500]
                del self.thermal_pressure_p[:500]

    def redraw(self):
        ts=list(self.times)
        if not ts:return
        x_right=max(30,ts[-1]+0.1)
        x_left=max(0,ts[-1]-30)
        for ax,lines,minimum_span,initial_limits in self.charts:
            for key,line in lines:
                line.set_data(ts,list(self.series[key]))
            ax.set_xlim(x_left,x_right)
            visible_values=(value for key,_line in lines
                            for sample_time,value in zip(ts,self.series[key])
                            if sample_time>=x_left)
            if ax in self.fixed_sensor_limits:
                ax.set_ylim(*self.fixed_sensor_limits[ax])
                continue
            limits=live_plot_y_limits(visible_values,minimum_span)
            if limits is not None:
                ax.set_ylim(*limits)
            else:
                ax.set_ylim(*initial_limits)
        if self.mag_points:
            pts=list(self.mag_points)
            stride=max(1,len(pts)//1800)
            thin=pts[::stride]
            xx,yy,zz=zip(*thin)
            self.mag_scatter._offsets3d=(xx,yy,zz)
            self._draw_magnetic_axes(thin)
        else:
            self._draw_magnetic_axes([])
        ready=self.altitude_status.startswith('Relative barometric height')
        self.altitude_annotation.set_text('' if ready else self.altitude_status)
        self.altitude_annotation.set_visible(not ready)
        self.canvas.draw_idle()

    def redraw_thermal(self):
        ts=list(self.times)
        if not ts:return
        for key,line in self.temp_lines:
            line.set_data(ts,list(self.series[key]))
        self.temp_plot.set_xlim(max(0,ts[-1]-60),max(60,ts[-1]+1))
        self.temp_plot.set_ylim(-40,85)
        finite=[v for key,line in self.temp_lines for v in self.series[key] if math.isfinite(v)]
        if finite:
            lo,hi=min(finite),max(finite)
            margin=max(0.3,(hi-lo)*0.12)
            self.temp_plot.set_ylim(lo-margin,hi+margin)
        self.thermal_scatter.clear()
        self.thermal_scatter.set_title('BMP pressure vs temperature (NOT corrected)')
        self.thermal_scatter.set_xlabel('BMP die temperature (°C)')
        self.thermal_scatter.set_ylabel('Pressure (Pa)')
        self.thermal_scatter.grid(alpha=.25)
        if self.thermal_pressure_t:
            temperatures=self.thermal_pressure_t[::3]
            pressures=self.thermal_pressure_p[::3]
            self.thermal_scatter.scatter(temperatures,pressures,s=7,alpha=.45)
            self.thermal_scatter.set_xlim(-40,85)
            limits=live_plot_y_limits(pressures,100.0)
            if limits is not None:
                self.thermal_scatter.set_ylim(*limits)
        else:
            self.thermal_scatter.set_xlim(-40,85)
            self.thermal_scatter.set_ylim(30000,120000)
        self.thermal_scatter.ticklabel_format(axis='y',style='plain',useOffset=False)
        self.temperature_canvas.draw_idle()

    def redraw_latency(self):
        ts=list(self.latency_times)
        summary=[]
        any_measurements=False
        x_left=max(0,ts[-1]-30) if ts else 0
        for index,sensor in enumerate(('MPU','QMC','BMP','LiDAR')):
            ax=self.latency_axes[index]
            age_line,service_line,empty_label=self.latency_lines[sensor]
            ages=list(self.latency_series[f'{sensor}_age'])
            service=list(self.latency_series[f'{sensor}_service'])
            age_line.set_data(ts,ages)
            service_line.set_data(ts,service)
            if ts:
                ax.set_xlim(x_left,max(30,ts[-1]+0.1))
            visible_ages=[value for sample_time,value in zip(ts,ages)
                          if sample_time>=x_left and math.isfinite(value)]
            visible_service=[value for sample_time,value in zip(ts,service)
                             if sample_time>=x_left and math.isfinite(value)]
            finite=[*visible_ages,*visible_service]
            if finite:
                any_measurements=True
                empty_label.set_visible(False)
                ax.set_ylim(0,max(1.0,max(finite)*1.08))
            else:
                empty_label.set_text(
                    'NO MEASURED DATA\nFirmware v4 age/service fields required')
                empty_label.set_visible(True)
                ax.set_ylim(0,1)
            def percentile_stats(values):
                clean=sorted(value for value in values if math.isfinite(value))
                if not clean:return 'n/a'
                def at(q):return clean[round((len(clean)-1)*q)]
                return '/'.join(str(round(at(q))) for q in (0.50,0.95,0.99,1.0))
            summary.append(f'{sensor} age P50/P95/P99/max {percentile_stats(visible_ages)} µs; '
                           f'service {percentile_stats(visible_service)} µs')
        if any_measurements:
            self.latency_summary.set('Displayed 30 s window, measured P50/P95/P99/max: '+' | '.join(summary))
        else:
            self.latency_summary.set(
                'UNAVAILABLE — no measured per-sensor age or service-time values have arrived. '
                'The 2 ms control period is not a latency measurement.')
        self.latency_canvas.draw_idle()

    def calculate(self):
        from tkinter import filedialog
        if self.analysis_running:
            self.msg.showinfo('Analysis already running','Please finish the current analysis first.')
            return
        # Live capture can continue while analysis runs, provided the user
        # selects only closed session files, not the active writer.
        names=filedialog.askopenfilenames(parent=self.root,
            title='Select one or more completed calibration CSV files (Ctrl+click)',
            initialdir=str(SESSION_COMPLETE_DIR if SESSION_COMPLETE_DIR.exists() else SESSION_INCOMPLETE_DIR),
            filetypes=[('Calibration CSV','*.csv')])
        if not names:
            return
        paths=[Path(p).resolve() for p in names]
        if (self.worker and self.worker.is_alive() and self.worker._fp is not None
                and self.worker.current_file is not None
                and self.worker.current_file.resolve() in paths):
            self.msg.showwarning('Active CSV cannot be analyzed safely',
                'The current session CSV is still being written. Finish it or start a new session before analysis.')
            return
        self.analysis_running=True
        self.calc.config(state='disabled')
        self.info.set(f'Analyzing {len(paths)} selected CSV file(s) in background; GUI remains responsive.')
        ANALYSIS_DIR.mkdir(parents=True,exist_ok=True)
        output_prefix=ANALYSIS_DIR/('combined_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
        def job():
            try:
                command=[sys.executable,str(ROOT/'tools/calibrate.py'),*[str(p) for p in paths],
                         '--output-prefix',str(output_prefix)]
                result=subprocess.run(command,capture_output=True,text=True,timeout=360)
                report=(result.stdout+'\n'+result.stderr).strip()
                self.events.put(('analysis_done',(result.returncode==0,report)))
            except Exception as exc:
                self.events.put(('analysis_done',(False,str(exc))))
        threading.Thread(target=job,daemon=True).start()

    def analyze_lidar_csv(self):
        from tkinter import filedialog
        if self.analysis_running:
            self.msg.showinfo('Analysis running','Finish the current analysis first.')
            return
        names=filedialog.askopenfilenames(parent=self.root,
            title='Select completed calibration CSV file(s) containing the four LiDAR references',
            initialdir=str(SESSION_COMPLETE_DIR if SESSION_COMPLETE_DIR.exists() else SESSION_INCOMPLETE_DIR),
            filetypes=[('Calibration CSV','*.csv')])
        if not names:return
        paths=[Path(name).resolve() for name in names]
        if (self.worker and self.worker.is_alive() and self.worker._fp is not None
                and self.worker.current_file is not None and self.worker.current_file.resolve() in paths):
            self.msg.showwarning('Active CSV','Save or close the current CSV segment before analyzing it.')
            return
        self.analysis_running=True
        self.lidar_analysis_button.config(state='disabled')
        self.calc.config(state='disabled')
        self.info.set('LiDAR range calibration analysis running; incomplete references will be reported.')
        def job():
            try:
                command=[sys.executable,str(ROOT/'tools/lidar_calibration.py'),
                         '--analyze',*map(str,paths),'--min-samples',str(self.args.lidar_samples),
                         '--required-distances','0.25','0.50','1.00','2.00']
                result=subprocess.run(command,capture_output=True,text=True,timeout=180)
                self.events.put(('analysis_done',(result.returncode==0,(result.stdout+'\n'+result.stderr).strip())))
            except Exception as exc:
                self.events.put(('analysis_done',(False,str(exc))))
        threading.Thread(target=job,daemon=True).start()

    def combine_calibrations(self):
        """Combine completed raw CSVs and saved result JSONs without pausing acquisition."""
        from tkinter import filedialog
        if self.analysis_running:
            self.msg.showinfo('Analysis running','Finish the current analysis first.')
            return
        names=filedialog.askopenfilenames(parent=self.root,
            title='Select calibration CSV files (Ctrl+click for multiple). '
                  'Results .json files are optional. '
                  'Example: measurements/calibration/esp32s3/sessions/complete/calibration_<timestamp>_complete.csv',
            initialdir=str(SESSION_COMPLETE_DIR if SESSION_COMPLETE_DIR.exists() else SESSION_INCOMPLETE_DIR),
            filetypes=[('Calibration data','*.csv *.json'),('CSV recordings','*.csv'),('Saved reports','*.json')])
        if not names:
            # Show a help dialog explaining which files to select
            self.msg.showinfo('How to combine calibration data',
                'To combine CSV calibration results:\n\n'
                '1. Select a completed or explicitly saved incomplete CSV session.\n'
                '2. Select the .csv file(s) from the calibration run.\n'
                '3. Optionally include saved *_calibration.json result files.\n'
                '4. Click OK; the combined output is saved under measurements/calibration/esp32s3/analysis/.')
            return
        paths=[Path(n).resolve() for n in names]
        if (self.worker and self.worker.is_alive() and self.worker._fp is not None
                and self.worker.current_file is not None and self.worker.current_file.resolve() in paths):
            self.msg.showwarning('Active CSV','Finish the current session or begin a new one before analysis.')
            return
        self.analysis_running=True
        self.calc.config(state='disabled')
        self.combine_button.config(state='disabled')
        ANALYSIS_DIR.mkdir(parents=True,exist_ok=True)
        target=ANALYSIS_DIR/('merged_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
        self.info.set(f'Merging {len(paths)} completed calibration inputs with provenance checks...')
        def job():
            try:
                command=[sys.executable,str(ROOT/'tools/combine_calibrations.py'),
                         *map(str,paths),'--output-prefix',str(target)]
                proc=subprocess.run(command,capture_output=True,text=True,timeout=600)
                self.events.put(('analysis_done',(proc.returncode==0,(proc.stdout+'\n'+proc.stderr).strip())))
            except Exception as exc:
                self.events.put(('analysis_done',(False,str(exc))))
        threading.Thread(target=job,daemon=True).start()

    def close(self):
        if self.worker:
            if self.session_active and not self.finished:
                choice=self.msg.askyesnocancel(
                    'Close calibration GUI?',
                    'The current session is incomplete.\n\n'
                    'Yes: save it with an incomplete tag and missing-phase details.\n'
                    'No: delete its captured samples.\n'
                    'Cancel: return to the GUI.')
                if choice is None:
                    return
                self.worker.end_stage()
                if choice:
                    missing=self._missing_stage_descriptions()
                    self.worker.finalize_session('complete' if not missing else 'incomplete',missing)
                else:
                    self.worker.discard_session()
            self.worker.stop_event.set()
            self.worker.join(timeout=3)
            if not self.worker.is_alive():
                self.worker._process_commands()
        self.root.destroy()

    def launch(self,port,axis_matrix):
        self.worker=Recorder(port,self.events,axis_matrix,
                             settle_seconds=self.args.settle_seconds,
                             wifi=self.args.wifi, wifi_host=self.args.wifi_host,
                             wifi_port=self.args.wifi_port, lidar_port=self.args.lidar_port)
        self.worker.run_started=time.monotonic()
        self.worker.start()
        self.root.mainloop()


def main():
    ap=argparse.ArgumentParser(description='IntelQCopter interactive calibration GUI')
    ap.add_argument('--port',default='',help='USB serial COM5; ignored in Wi-Fi mode')
    ap.add_argument('--wifi',action='store_true',help='Use ESP32-S3 Wi-Fi AP + UDP, not USB serial')
    ap.add_argument('--wifi-host',default='192.168.4.1',help='ESP32-S3 access-point IP')
    ap.add_argument('--wifi-port',type=int,default=19005,help='local UDP telemetry port')
    ap.add_argument('--output',type=Path,default=None)
    ap.add_argument('--config',type=Path,default=ROOT/'sensor_config.json')
    ap.add_argument('--gyro-samples',type=int,default=30000)
    ap.add_argument('--accel-samples',type=int,default=5000)
    ap.add_argument('--mag-turn-samples',type=int,default=2000)
    ap.add_argument('--mag-tumble-samples',type=int,default=8000)
    ap.add_argument('--bmp-samples',type=int,default=1000)
    ap.add_argument('--skip-bmp',action='store_true')
    ap.add_argument('--mag-only',action='store_true',help='Record only the four QMC/HMC 3D rotation stages; combine with full six-face CSV later')
    ap.add_argument('--settle-seconds',type=float,default=3.0,help='Preparation delay after Enter before recording (default 3s)')
    ap.add_argument('--self-test',action='store_true')
    ns=ap.parse_args()
    # LiDAR calibration is deliberately held until after the competition.
    ns.skip_lidar=True
    ns.lidar_port=''
    ns.lidar_samples=300
    if ns.self_test:
        selftest()
        return
    if any(getattr(ns,n)<1 for n in ('gyro_samples','accel_samples','mag_turn_samples','mag_tumble_samples','bmp_samples')):
        ap.error('All sample targets must be positive')
    if not (0.0 <= ns.settle_seconds <= 30.0) or not math.isfinite(ns.settle_seconds):
        ap.error('--settle-seconds must be between 0 and 30')
    if not ns.port and not ns.wifi:
        try:
            from serial.tools import list_ports
            ports=list(list_ports.comports())
            if len(ports)!=1:
                ap.error('Specify --port COM5. Ports: '+', '.join(p.device for p in ports))
            ns.port=ports[0].device
        except ImportError:
            ap.error('pyserial is missing. Install packages from requirements.txt')
    axis_name, axis_matrix=load_axis_map(ns.config)
    if abs(abs(determinant(axis_matrix))-1)>1e-6:
        ap.error('Mag axis mapping must have determinant +/-1')
    out=Path(ns.output).resolve() if ns.output else None
    prepare_tk_runtime()
    import tkinter as tk
    ui=Application(tk,ns,stages(ns),axis_name,axis_matrix,out)
    ui.launch(ns.port,axis_matrix)


if __name__=='__main__':
    main()
