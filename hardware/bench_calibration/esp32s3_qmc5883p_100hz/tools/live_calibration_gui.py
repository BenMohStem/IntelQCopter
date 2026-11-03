#!/usr/bin/env python3
"""IntelQCopter 500 Hz guided calibration GUI (binary v2/v3 / 52-byte packets).

The firmware sends 500 packet/s; only fresh MAG/BMP packets count for slow
sensor phases. All raw samples are saved to CSV; graphing is decimated.
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
import socket
import queue
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

MAGIC = b'\xA5\x5A'
SIZE = 52
BAUD = 921600
G = 9.80665
HMC_COUNTS_PER_UT = 1090.0 / 100.0
QMC_COUNTS_PER_UT = 3000.0 / 100.0  # QMC5883L +/-8 G
QMCP_COUNTS_PER_UT = 3750.0 / 100.0  # QMC5883P +/-8 G, QST datasheet Rev E
ROOT = Path(__file__).resolve().parent.parent

CSV_COLUMNS = ('kind,seq,t_us,pose,mpu_ok,bmp_ok,mag_ok,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,'
    'mpu_temp_raw,ax_mps2,ay_mps2,az_mps2,gx_dps,gy_dps,gz_dps,mpu_temp_c,'
    'bmp_new,bmp_adc_p,bmp_adc_t,bmp_temp_c,pressure_pa,mag_new,mx_raw,my_raw,mz_raw,'
    'mx_uT,my_uT,mz_uT,host_utc_iso,mag_stage,late_us,missed_ticks,'
    'mag_board_x_uT,mag_board_y_uT,mag_board_z_uT,relative_altitude_m,host_monotonic_s,phase_accepted,phase_elapsed_s,mag_sensor_type,observed_packet_dt_us,seq_gap_before,gap_gt_5ms,gap_gt_10ms,phase_attempt,lidar_reference_m,lidar_distance_m,lidar_status,lidar_signal,lidar_sensor_time_ms,lidar_module_id').split(',')


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

    def accel(self):
        return tuple(x * G / 16384.0 for x in (self.ax,self.ay,self.az))

    def gyro(self):
        return tuple(x / 131.0 for x in (self.gx,self.gy,self.gz))

    def mag(self):
        counts_per_ut = (QMCP_COUNTS_PER_UT if self.mag_sensor_type == 'QMC5883P'
                         else QMC_COUNTS_PER_UT if self.mag_sensor_type == 'QMC5883L'
                         else HMC_COUNTS_PER_UT)
        return tuple(x / counts_per_ut for x in (self.mx,self.my,self.mz))

    def bmp(self):
        return self.bmp_pa100 / 100.0


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
        while len(self.buffer) >= SIZE:
            pos = self.buffer.find(MAGIC)
            if pos < 0:
                self.bad_sync += len(self.buffer) - 1
                del self.buffer[:-1]
                break
            if pos:
                self.bad_sync += pos
                del self.buffer[:pos]
            if len(self.buffer) < SIZE:
                break
            if self.buffer[2] not in (2,3):
                self.bad_version += 1
                del self.buffer[0]
                continue
            pkt = bytes(self.buffer[:SIZE])
            got = int.from_bytes(pkt[50:52], 'little')
            if binascii.crc_hqx(pkt[:50], 0xFFFF) != got:
                self.bad_crc += 1
                del self.buffer[0]
                continue
            fields = struct.unpack_from('<IIH7h3hIIhIH', pkt, 4)
            frames.append(Frame(pkt[3], *fields, version=pkt[2]))
            self.total += 1
            del self.buffer[:SIZE]
        # Prevent unbounded noise memory on malformed streams.
        if len(self.buffer) > 65536:
            del self.buffer[:-SIZE]
        return frames


def make_test_packet(seq, ax=0, ay=0, az=16384, flags=0x1F, mag=(0,0,0), version=2):
    b = bytearray(MAGIC + bytes([version, flags]))
    b += struct.pack('<IIH7h3hIIhIH', seq, (seq*2000)&0xFFFFFFFF, 0,
        ax,ay,az,0,0,0,0,*mag,500000,500000,2350,10132500,0)
    assert len(b) == 50
    b += struct.pack('<H', binascii.crc_hqx(b, 0xFFFF))
    return bytes(b)


def selftest():
    d = BinaryDecoder()
    d.feed(b'noise')
    packet = make_test_packet(123, 16384, 0, 0, mag=(100,200,-80))
    result = []
    for bit in (packet[:1], packet[1:17], packet[17:39], packet[39:]):
        result += d.feed(bit)
    assert len(result) == 1
    f = result[0]
    assert f.seq == 123 and f.ax == 16384 and f.my == 200
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


@dataclass(frozen=True)
class Stage:
    pose: str
    mag_stage: str
    target: int
    sensor: str
    prompt: str


def stages(args):
    sequence = ([Stage('REST','STILL',args.gyro_samples,'mpu',
                  'Leave the drone COMPLETELY STATIONARY, level or at any fixed orientation.\n'
                  'Do not touch the drone during this stationary gyro calibration. Warm up electronics first.')]
        + [Stage(p,'STILL',args.accel_samples,'mpu',
            f'Place the MPU6050 {p} SENSOR axis vertically UP.\n'
            f'Hold this pose steady during sampling. No software pose gate; accurate orientation is your responsibility.')
           for p in ('+X','-X','+Y','-Y','+Z','-Z')]
        + [Stage('MAG','YAW',args.mag_turn_samples,'mag',
                 'Keep drone approximately HORIZONTAL and slowly rotate it 360 degrees around vertical (yaw).\n'
                 'Repeat at several small tilts. Press Enter, wait for countdown, then START rotating.'),
           Stage('MAG','PITCH',args.mag_turn_samples,'mag',
                 'Slowly PITCH entire assembled drone 360 degrees: nose up, inverted, nose down, return.\n'
                 'Wait for the 3-second countdown, then perform the full turn. Use a clear open space; no motors or steel table.'),
           Stage('MAG','ROLL',args.mag_turn_samples,'mag',
                 'Wait for the 3-second countdown, then slowly ROLL entire drone 360 degrees through inverted poses.'),
           Stage('MAG','TUMBLE',args.mag_tumble_samples,'mag',
                 'Wait for the 3-second countdown, then slow unrestricted 3D tumbling. Cover EVERY orientation and diagonals.\n'
                 'This stage is essential for a well-conditioned ellipsoid fit.')]
        + ([] if args.skip_bmp else [Stage('BMP','STILL',args.bmp_samples,'bmp',
                 'Keep drone still at one known reference height, with no propeller wash or moving air.\n'
                 'A barometer does not provide a true altitude reference by itself.')])
)
    return ([x for x in sequence if x.sensor == 'mag']
            if getattr(args, 'mag_only', False) else sequence)


class WifiPacketStream:
    """UDP client for the ESP32-S3 AP calibration stream.

    Same 52-byte frames as USB. Receive and compute timestamps on the PC
    WITHOUT substituting network-reception time for the device sample time.
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
        if not data.startswith(MAGIC) or len(data) % SIZE != 0:
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
    def __init__(self, port, file, event_queue, axis_map, settle_seconds=3.0,
                 wifi=False, wifi_host="192.168.4.1", wifi_port=19005, lidar_port=""):
        super().__init__(daemon=True)
        self.port, self.file = port, Path(file)
        self.wifi,self.wifi_host,self.wifi_port=bool(wifi),str(wifi_host),int(wifi_port)
        self.events = event_queue
        self.lidar_port = str(lidar_port).strip()
        self.lidar_queue = queue.Queue(maxsize=1024)
        self.lidar_reader = None
        self.lidar_total = 0
        self.axis_map = axis_map
        self.settle_seconds = float(settle_seconds)
        self.lock = threading.Lock()
        self.active = None
        self.sample_count = 0
        self.phase_start = 0.0
        self.stop_event = threading.Event()
        self.started = threading.Event()
        self.decoder = BinaryDecoder()
        self.frames_total = 0
        self.bytes_total = 0
        self.last_byte_at = 0.0
        self.mpu_total = self.mag_total = self.bmp_total = 0
        self.last_frame = None
        self.last_seq = None
        self.last_mcu_timestamp = None
        self.seq_gaps = 0
        self.p0_samples = []
        self.p0 = None
        self.last_emit = 0.0
        self.phase_last_progress = 0
        self.error = None
        self.paused = False
        self.last_accepted_at = 0.0
        self.stage_warning_issued = False
        self.stage_start_time = 0.0
        self.last_reject_reason = 'No samples checked yet'
        self.skipped = []
        self.last_countdown_emit = 0.0
        self.capture_started_notified = False
        self.checkpoint_request = threading.Event()
        self.attempt_counter = 0
        self.current_attempt = 0
        self.segment_number = 1
        self.current_file = self.file
        self.completed_files = []

    def connect_lidar(self, port):
        """Attach optional LiDAR on its own COM port, without restarting ESP stream.

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

    def save_checkpoint(self):
        """Request a completed, closed CSV segment while acquisition continues."""
        self.checkpoint_request.set()

    def _segment_filename(self, number):
        return self.file.with_name(self.file.stem + f'_part{number:02d}' + self.file.suffix)

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
            self.next_hint_at = self.phase_start + 5.0
            return True

    def end_stage(self):
        with self.lock:
            self.active = None

    def skip_stage(self):
        """Explicitly omit a stage; never silently relabel data as calibrated."""
        with self.lock:
            stage = self.active
            self.active = None
            self.paused = False
            if stage:
                self.skipped.append({'pose':stage.pose,'mag_stage':stage.mag_stage,
                                     'accepted_before_skip':self.sample_count,
                                     'target':stage.target})
            return stage


    def run(self):
        fp = None
        try:
            self.file.parent.mkdir(parents=True, exist_ok=True)
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
                fp = self.current_file.open('x', newline='', encoding='utf-8')
                csvw = csv.writer(fp)
                csvw.writerow(CSV_COLUMNS)
                fp.flush()
                self.started.set()
                self.events.put(('status',
                    f'Wi-Fi UDP listening on :{self.wifi_port}; connect PC to IntelQCopter-Cal AP and wait for sensor packets'
                    if self.wifi else f'Connected to {self.port} at {BAUD} baud'))
                count_since_flush = 0
                while not self.stop_event.is_set():
                    # The I/O thread alone owns the CSV writer. Closing/rotating
                    # here guarantees each saved segment is complete and can be
                    # analyzed while a new one keeps recording.
                    if self.checkpoint_request.is_set():
                        self.checkpoint_request.clear()
                        fp.flush()
                        fp.close()
                        old = self.current_file
                        self.completed_files.append(old)
                        self.segment_number += 1
                        self.current_file = self._segment_filename(self.segment_number)
                        fp = self.current_file.open('x', newline='', encoding='utf-8')
                        csvw = csv.writer(fp)
                        csvw.writerow(CSV_COLUMNS)
                        fp.flush()
                        count_since_flush = 0
                        self.events.put(('saved_checkpoint', (old, self.current_file)))
                    data = ser.read(4096)
                    # Drain separately received LiDAR frames without waiting for
                    # its slower UART; record each with its own sensor clock.
                    for _ in range(128):
                        try:received, sample = self.lidar_queue.get_nowait()
                        except queue.Empty:break
                        self._handle_lidar(sample,received,csvw)
                    if not data:
                        continue
                    self.bytes_total += len(data)
                    self.last_byte_at = time.monotonic()
                    frames = self.decoder.feed(data)
                    for frame in frames:
                        self._handle_frame(frame, csvw)
                        count_since_flush += 1
                    if count_since_flush >= 250:
                        fp.flush()
                        count_since_flush = 0
        except Exception as e:
            self.error = str(e)
            self.events.put(('error', str(e)))
        finally:
            if fp is not None:
                try:fp.flush();fp.close()
                except OSError:pass
            self.started.set()
            self.events.put(('stopped', None))

    def _handle_lidar(self,sample,received,writer):
        self.lidar_total+=1
        with self.lock:
            stage=self.active
            counting=bool(stage and stage.sensor=='lidar' and not self.paused and received>=self.phase_start)
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
            if count>=stage.target:
                self.active=None
                self.events.put(('stage_done',(stage,count)))
        ref=float(int(stage.mag_stage[1:]))/1000.0
        row=dict.fromkeys(CSV_COLUMNS,'')
        row.update(kind='L',pose='LIDAR',mag_stage=stage.mag_stage,
                   host_utc_iso=datetime.now(timezone.utc).isoformat(timespec='milliseconds'),
                   host_monotonic_s=f'{received:.6f}',phase_accepted=int(accepted),
                   phase_attempt=attempt,
                   lidar_reference_m=f'{ref:.3f}',lidar_distance_m=f'{sample.distance_m:.6f}',
                   lidar_status=sample.status,lidar_signal=sample.signal_strength,
                   lidar_sensor_time_ms=sample.sensor_time_ms,lidar_module_id=sample.module_id)
        # Temperatures are the latest ESP32 observations, asynchronous to LiDAR;
        # they are *not* measurements of the LiDAR die temperature.
        if self.last_frame is not None:
            f=self.last_frame
            row['mpu_temp_c']=f'{f.temp/340.0+36.53:.3f}' if f.mpu_ok else ''
            row['bmp_temp_c']=f'{f.bmp_tc100/100:.3f}' if f.bmp_ok else ''
        writer.writerow([row[c] for c in CSV_COLUMNS])
        if accepted and (count%10==0 or count==stage.target):
            self.events.put(('progress',count))

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
                        'Skip phase if device is physically absent')
            return None if f.mag_new else 'Waiting for FRESH magnetometer reading'
        if stage.sensor == 'bmp':
            return None if (f.bmp_ok and f.bmp_new) else 'Waiting for FRESH BMP280 reading'
        return 'Unsupported calibration stage'

    @staticmethod
    def _accept_phase_frame(f, stage):
        return Recorder._phase_rejection_reason(f,stage) is None

    def _handle_frame(self, f, writer):
        now = time.monotonic()
        self.frames_total += 1
        self.mpu_total += int(f.mpu_ok)
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

        if f.bmp_new and f.bmp() > 10000:
            if self.p0 is None:
                self.p0_samples.append(f.bmp())
                if len(self.p0_samples) >= 30:
                    self.p0 = sorted(self.p0_samples)[len(self.p0_samples)//2]
        altitude = (44330.0*(1.0-(f.bmp()/self.p0)**(1.0/5.255))) if f.bmp_ok and self.p0 else math.nan
        mx = f.mag()
        mx_board = map_mag(self.axis_map, mx)
        if f.mag_new:
            self.events.put(('mag_point', mx_board))
        with self.lock:
            stage = self.active
            counting = bool(stage and not self.paused and now >= self.phase_start)
            preparing = bool(stage and not self.paused and now < self.phase_start)
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
                count = self.sample_count
                if now >= self.next_hint_at and (now-self.last_accepted_at) > 5.0:
                    self.events.put(('hint',
                        f'No new {stage.sensor.upper()} samples for {stage.pose}/{stage.mag_stage}. '
                        f'Reason: {self.last_reject_reason}. '
                        'This stage does not check orientation; if sensor missing, use Skip phase.'))
                    self.next_hint_at = now + 8.0
                expected_rate = {'mpu':500.0, 'mag':15.0, 'bmp':10.0, 'lidar':50.0}[stage.sensor]
                slow_deadline = max(30.0, stage.target / expected_rate * 3.0)
                if not self.stage_warning_issued and now-self.stage_start_time > slow_deadline:
                    self.stage_warning_issued = True
                    self.events.put(('hint',
                        f'Stage {stage.pose}/{stage.mag_stage} is unusually slow: '
                        f'{count}/{stage.target} real samples. Use Skip phase if sensor unavailable.'))
                if count >= stage.target:
                    self.active = None
                    self.events.put(('stage_done', (stage, count)))
            else:
                count = self.sample_count
            capture = counting and stage.sensor != 'lidar'
        if capture:
            a = f.accel() if f.mpu_ok else (math.nan,)*3
            g = f.gyro() if f.mpu_ok else (math.nan,)*3
            m = mx if f.mag_ok else (math.nan,)*3
            mb = mx_board if f.mag_ok else (math.nan,)*3
            row = [
                'D', f.seq, f.t_us, stage.pose, int(f.mpu_ok),int(f.bmp_ok),int(f.mag_ok),
                f.ax,f.ay,f.az,f.gx,f.gy,f.gz,f.temp,
                *[f'{v:.6f}' for v in a], *[f'{v:.6f}' for v in g],
                f'{f.temp/340.0+36.53:.3f}' if f.mpu_ok else 'nan',
                int(f.bmp_new), f.bmp_raw_p, f.bmp_raw_t,
                f'{f.bmp_tc100/100:.2f}' if f.bmp_ok else 'nan',
                f'{f.bmp():.2f}' if f.bmp_ok else 'nan',
                int(f.mag_new), f.mx,f.my,f.mz,
                *[f'{v:.6f}' for v in m],
                datetime.now(timezone.utc).isoformat(timespec='milliseconds'),
                stage.mag_stage, f.late_us, f.missed_ticks,
                *[f'{v:.6f}' for v in mb],
                f'{altitude:.4f}', f'{now:.3f}', int(valid),
                f'{now-self.stage_start_time:.3f}', f.mag_sensor_type,
                dt_us if dt_valid else '', seq_gap_before, int(dt_valid and dt_us>5000), int(dt_valid and dt_us>10000), self.current_attempt,
                '', '', '', '', '', '',
            ]
            assert len(row)==len(CSV_COLUMNS), (len(row),len(CSV_COLUMNS))
            writer.writerow(row)

        if now - self.last_emit >= 0.12:
            self.last_emit = now
            # Drop disposable plot snapshots if Tk falls behind; never drop
            # phase completion or alerts. Raw CSV recording is unaffected.
            if self.events.qsize() < 60:
                self.events.put(('plot', {
                't': now, 'a': f.accel() if f.mpu_ok else (math.nan,)*3,
                'g': f.gyro() if f.mpu_ok else (math.nan,)*3,
                'm': mx_board if f.mag_ok else (math.nan,)*3,
                'p': f.bmp() if f.bmp_ok else math.nan, 'h': altitude,
                'seq': f.seq, 'drop': self.seq_gaps, 'crc': self.decoder.bad_crc,
                'mpu': f.mpu_ok, 'mag': f.mag_ok, 'bmp': f.bmp_ok,
                'rate': self.frames_total/max(.001,(now-self.run_started)) if hasattr(self,'run_started') else 0,
                'mag_fresh':self.mag_total, 'bmp_fresh':self.bmp_total,
                'mag_sensor_type': f.mag_sensor_type,
                'mpu_temp': f.temp/340.0+36.53 if f.mpu_ok else math.nan,
                'bmp_temp': f.bmp_tc100/100.0 if f.bmp_ok else math.nan,
                'bmp_new': f.bmp_new,
            }))
        if capture and count != self.phase_last_progress and (count%25==0 or count==stage.target):
            self.phase_last_progress=count
            self.events.put(('progress', count))


class Application:
    def __init__(self, tk, args, stage_list, axis_name, axis_matrix, output):
        from tkinter import ttk, messagebox
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        self.tk, self.ttk, self.msg = tk, ttk, messagebox
        self.args, self.stages, self.output = args, stage_list, output
        self.root=tk.Tk()
        self.root.title('IntelQCopter | 500 Hz guided sensor calibration')
        self.root.geometry('1240x890')
        self.events=queue.Queue()
        self.worker=None
        self.stage_index=0
        self.finished=False
        self.capture_active=False
        self.analysis_running=False
        self.skipped_stages=[]
        self.start_time=time.monotonic()
        self.lines=[]
        self.times=deque(maxlen=500)
        self.mag_points=deque(maxlen=8000)
        self.series={key:deque(maxlen=500) for key in ('ax','ay','az','gx','gy','gz','mx','my','mz','pressure','altitude','mpu_temp','bmp_temp')}
        self.last_plot=0
        self.live_plot_enabled=True
        self.last_3d_update=0.0
        self.last_magnetic_count=0
        self.mag_popup=None
        self.mag_popup_last_draw=0.0
        self.info=tk.StringVar(value='Connecting sensor telemetry...')
        self.saved_info=tk.StringVar(value='Capture file is automatically written during each phase.')
        self.headline=tk.StringVar(value='')
        self.instruction=tk.StringVar(value='')
        self.progress=tk.StringVar(value='')
        self.paused_status=tk.StringVar(value='')
        self.health=tk.StringVar(value='Opening sensor stream…')

        top=ttk.Frame(self.root,padding=12);top.pack(fill='x')
        ttk.Label(top,text='IntelQCopter | 500 Hz sensor calibration',font=('Segoe UI',17,'bold')).pack(anchor='w')
        ttk.Label(top,text=f'Output: {output}',font=('Segoe UI',9)).pack(anchor='w')
        ttk.Label(top,text=f'MAG mapping: {axis_name}; det={determinant(axis_matrix):+.0f}. Raw chip axes remain unchanged in CSV.',foreground='#A35A00').pack(anchor='w')
        ttk.Label(top,textvariable=self.info).pack(anchor='w',pady=(6,0))
        ttk.Separator(self.root,orient='horizontal').pack(fill='x')
        phase=ttk.Frame(self.root,padding=12);phase.pack(fill='x')
        ttk.Label(phase,textvariable=self.headline,font=('Segoe UI',14,'bold')).pack(anchor='w')
        ttk.Label(phase,textvariable=self.instruction,justify='left',wraplength=1100,font=('Segoe UI',9)).pack(anchor='w',pady=5)
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
        self.skip=ttk.Button(buttons,text='Skip phase (incomplete)',command=self.skip_phase)
        self.skip.pack(side='left',padx=8)
        self.save_now=ttk.Button(buttons,text='Save CSV segment / continue',command=self.save_current)
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
        self.mag_quality_button=ttk.Button(extra,
            text='Assess MAG precision / coverage from CSVs...',
            command=self.assess_mag_quality)
        self.mag_quality_button.pack(side='left',padx=(0,8))
        self.combine_button=ttk.Button(extra,text='Combine CSV + calibration JSON...',command=self.combine_calibrations)
        self.combine_button.pack(side='left',padx=(0,8))
        ttk.Label(extra,text='Withheld-data fit, 3D coverage and bootstrap; superseded redo attempts are excluded.',
                  foreground='#5A6470').pack(side='left')
        ttk.Label(phase,textvariable=self.health,foreground='#285884').pack(anchor='w')
        ttk.Label(phase,textvariable=self.saved_info,foreground='#186A58').pack(anchor='w')

        # Calling constrained_layout on every draw is expensive with a 3D axis.
        # Static layout drastically reduces GUI latency on Windows laptops.
        notebook=ttk.Notebook(self.root)
        notebook.pack(fill='both',expand=True,padx=10,pady=6)
        self.tab_live=ttk.Frame(notebook)
        self.tab_thermal=ttk.Frame(notebook)
        notebook.add(self.tab_live,text='Live motion, magnetic field & altitude')
        notebook.add(self.tab_thermal,text='MPU6050 / BMP280 temperatures')
        self.notebook=notebook
        fig=Figure(figsize=(13,6.4),dpi=90,constrained_layout=False)
        fig.subplots_adjust(left=0.07,right=0.98,bottom=0.11,top=0.93,wspace=0.34,hspace=0.88)
        ax_acc=fig.add_subplot(2,3,1)
        ax_gyro=fig.add_subplot(2,3,2)
        ax_3d=fig.add_subplot(2,3,3,projection='3d')
        ax_mag=fig.add_subplot(2,3,4)
        ax_pressure=fig.add_subplot(2,3,5)
        ax_alt=fig.add_subplot(2,3,6)
        self.mag_3d=ax_3d
        ax_3d.set_title('3D MAG cloud',fontsize=9,pad=3)
        for axis in ('X','Y','Z'):
            getattr(ax_3d,'set_'+axis.lower()+'label')(axis+' µT',fontsize=8)
        self.mag_scatter=ax_3d.scatter([],[],[],s=4,alpha=.43,c='#6243B7',depthshade=False)
        self._mag_triads=configure_magnetic_axes(ax_3d,[],enlarged=False)
        self.charts=[]
        for ax,title,keys,unit in [
            (ax_acc,'Accel · 500 Hz',('ax','ay','az'),'m/s²'),
            (ax_gyro,'Gyro · 500 Hz',('gx','gy','gz'),'°/s'),
            (ax_mag,'MAG · display axes',('mx','my','mz'),'µT'),
            (ax_pressure,'BMP pressure',('pressure',),'Pa'),
            (ax_alt,'Relative height',('altitude',),'m')]:
            ax.set_title(title,fontsize=9,pad=3)
            ax.set_ylabel(unit,fontsize=8,labelpad=2)
            ax.set_xlabel('' if ax in (ax_acc,ax_gyro) else 'Time (s)',fontsize=8,labelpad=1)
            ax.tick_params(labelsize=7,pad=1)
            ax.grid(alpha=0.25)
            lines=[]
            for key in keys:
                line,=ax.plot([],[],label=key,linewidth=1.1)
                lines.append((key,line))
            ax.legend(loc='upper right',fontsize=8)
            self.charts.append((ax,lines))
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
        self.root.bind('<Return>',self.on_enter)
        self.root.protocol('WM_DELETE_WINDOW',self.close)
        self.update_stage()
        self.root.after(120,self.pump)

    def on_enter(self,event):
        # Do not start an MPU/gyro phase while editing the LiDAR COM field.
        if event.widget == self.lidar_port_combo:
            self.connect_lidar_from_gui()
        else:
            self.begin_phase()

    def _available_lidar_ports(self):
        try:
            from serial.tools import list_ports
            return [x.device for x in list_ports.comports()
                    if x.device.casefold() != str(self.args.port).casefold()]
        except ImportError:
            return []

    def refresh_lidar_ports(self):
        choices = self._available_lidar_ports()
        self.lidar_port_combo['values'] = choices
        if not self.lidar_port_var.get().strip() and len(choices) == 1:
            self.lidar_port_var.set(choices[0])
        self.lidar_status_var.set('Available LiDAR ports: '+(', '.join(choices) if choices else
                                  'none; connect a separate USB-UART adapter'))

    def connect_lidar_from_gui(self):
        port = self.lidar_port_var.get().strip()
        if not self.worker or not self.worker.started.is_set() or self.worker.error:
            self.lidar_status_var.set('ESP serial connection must start before connecting LiDAR.')
            return
        ok, note = self.worker.connect_lidar(port)
        self.lidar_status_var.set(note)
        if ok:
            self.args.lidar_port = port
        else:
            self.msg.showwarning('Optional LiDAR connection',note)

    def skip_all_lidar_from_gui(self):
        pending = [x for x in self.stages[self.stage_index:] if x.sensor == 'lidar']
        if not pending:
            self.lidar_status_var.set('No remaining LiDAR ranges to skip.')
            return
        if not self.msg.askyesno('Skip optional LiDAR calibration?',
                                'Skip all remaining LiDAR distance stages?\n'
                                'MPU6050, magnetometer and BMP280 calibration will be preserved.\n'
                                'LiDAR will be marked as SKIPPED (not calibrated).'):
            return
        if (self.stage_index < len(self.stages) and
                self.stages[self.stage_index].sensor == 'lidar' and self.worker):
            self.worker.skip_stage()
            self.capture_active = False
        for x in pending:
            self.skipped_stages.append({'pose':x.pose, 'mag_stage':x.mag_stage,
                                        'reason':'optional_skipped_in_gui'})
        self.stages = [x for x in self.stages if x.sensor != 'lidar']
        self._save_skip_manifest()
        self.lidar_status_var.set('LiDAR skipped in GUI; missing calibration is recorded in .skipped.json')
        if not self.capture_active:
            self.update_stage()

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
        self.worker.save_checkpoint()
        self.saved_info.set('Saving completed CSV segment; continuing in a new file…')

    def update_stage(self):
        if self.stage_index>=len(self.stages):
            self.finished=True
            self.headline.set('COMPLETED — all scheduled captures saved')
            self.instruction.set('Press Calculate Calibration. Check JSON warnings and validate each measured sensor.')
            self.progress.set(f'Saved to: {self.output}')
            self.bar['value']=100
            self.begin.config(state='disabled')
            self.info.set('Finished. The serial connection can be closed with Stop / save.')
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
        only the latest attempt for each pose in the same session, across
        checkpoints, so previously recorded bad poses are never blended in.
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

    def skip_phase(self):
        if self.finished or not self.worker:
            return
        stage=self.stages[self.stage_index]
        if not self.msg.askyesno('Skip uncalibrated phase?',
            f'Skip {stage.pose}/{stage.mag_stage}?\nThis stage will be MISSING or INCOMPLETE in the calibration report.\n'
            'If LiDAR is selected, all optional ranges are skipped. No missing constants are invented.'):
            return
        self.worker.skip_stage()
        if stage.sensor == 'lidar':
            # One skip omits all optional LiDAR references without four dialogs.
            while self.stage_index<len(self.stages) and self.stages[self.stage_index].sensor=='lidar':
                x=self.stages[self.stage_index]
                self.skipped_stages.append({'pose':x.pose,'mag_stage':x.mag_stage,'reason':'optional_not_connected'})
                self.stage_index+=1
        else:
            self.skipped_stages.append({'pose':stage.pose,'mag_stage':stage.mag_stage})
            self.stage_index+=1
        self.capture_active=False
        self.update_stage()
        self._save_skip_manifest()
        self.info.set('Stage SKIPPED. Missing or incomplete calibration will be reported, not treated as valid.')

    def _save_skip_manifest(self):
        manifest=self.output.with_suffix('.skipped.json')
        manifest.parent.mkdir(parents=True,exist_ok=True)
        manifest.write_text(json.dumps({'skipped_stages':self.skipped_stages},indent=2),encoding='utf-8')

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
        s=self.stages[self.stage_index]
        if s.sensor == 'lidar':
            if (not self.worker.lidar_reader or self.worker.lidar_reader.error
                    or not self.worker.lidar_reader.connected.is_set()):
                self.msg.showinfo('Optional LiDAR not connected',
                   'To calibrate LiDAR, connect its SEPARATE USB-UART COM port above.\n'
                   'Or click Skip ALL LiDAR ranges. No range data is invented.\n'
                   'The ESP32 COM port cannot simultaneously be the raw LiDAR COM port.')
                return
        # Always honor ENTER: start a countdown, then count only actual sensor
        # acquisitions. Never block because of attitude, stillness, or missing
        # sensor. Missing sensors remain zero-count (never manufacture data).
        if self.worker.start_stage(s):
            self.capture_active=True
            self.pause.config(text='Pause',state='normal')
            self.begin.config(state='disabled')
            self.info.set(f'PREPARE {s.pose} / {s.mag_stage}: {self.args.settle_seconds:g}-second countdown started. Record begins automatically.')
            self.progress.set(f'0 / {s.target:,} — PREPARING ({self.args.settle_seconds:g} s)')

    def pump(self):
        if self.worker and self.worker.lidar_reader is not None:
            reader=self.worker.lidar_reader
            state=('ERROR: '+reader.error if reader.error else
                   f'LiDAR connected on {reader.port} at 921600 baud' if reader.connected.is_set() else
                   f'Opening LiDAR serial {reader.port}...')
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
            elif kind=='stage_done':
                s,count=payload
                # Ignore a queued completion after a user-initiated skip.
                if self.finished or s != self.stages[self.stage_index]:
                    continue
                self.capture_active=False
                self.pause.config(text='Pause',state='disabled')
                self.stage_index+=1
                self.update_stage()
                if not self.finished:
                    self.info.set(f'Completed {s.pose}/{s.mag_stage}: {count:,} valid readings. Position for next phase and press Enter.')
                else:
                    self.info.set(f'All {len(self.stages)} phases complete. Saved to {self.output}')
            elif kind=='lidar_status':
                self.info.set(payload)
            elif kind=='saved_checkpoint':
                saved, new_file=payload
                self.saved_info.set(f'SAVED: {saved.name} | now recording {new_file.name}')
                self.info.set('CSV segment safely closed. The GUI and active capture continue.')
            elif kind=='analysis_done':
                ok, report=payload
                self.analysis_running=False
                self.calc.config(state='normal')
                self.combine_button.config(state='normal')
                self.mag_quality_button.config(state='normal')
                if ok:self.msg.showinfo('Analysis finished',report[-5000:])
                else:self.msg.showerror('Calibration failed',report[-5000:])
            elif kind=='plot':
                self.add_point(payload)
            elif kind=='mag_point':
                self.mag_points.append(payload)
        now=time.monotonic()
        if (self.mag_popup is not None and self.mag_popup.winfo_exists() and
                now-self.mag_popup_last_draw >= 2.0 and self.mag_points):
            self.mag_popup_last_draw=now
            self.redraw_big3d()
        if self.live_plot_enabled and now-self.last_plot>=0.90:
            self.last_plot=now
            if self.times:
                if self.notebook.index('current') == 0:
                    self.redraw()
                else:
                    self.redraw_thermal()
            if self.worker:
                w=self.worker
                frame=w.last_frame
                age=(time.monotonic()-w.last_byte_at) if w.last_byte_at else time.monotonic()-self.start_time
                if not w.bytes_total and age>3:
                    self.health.set(f'NO UART DATA on {w.port} at {BAUD} baud. Close other serial apps; confirm ESP firmware streams 52-byte v2/v3 packets.')
                elif w.bytes_total and not w.frames_total:
                    self.health.set(f'UART bytes={w.bytes_total:,}, valid packets=0 | sync={w.decoder.bad_sync:,} CRC={w.decoder.bad_crc:,} version={w.decoder.bad_version:,}. Wrong baud/protocol or incompatible firmware.')
                elif frame:
                    self.health.set(f'{w.frames_total:,} packets | MPU {w.mpu_total:,} | MAG {w.mag_total:,} | BMP {w.bmp_total:,} | flags 0x{frame.flags:02X} | CRC {w.decoder.bad_crc:,} | gaps {w.seq_gaps:,}')
                elif w.bytes_total:
                    self.health.set(f'UART receiving {w.bytes_total:,} bytes; waiting for a complete valid packet…')
        self.root.after(120,self.pump)

    def add_point(self,p):
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
        for ax,lines in self.charts:
            for key,line in lines:
                line.set_data(ts,list(self.series[key]))
            ax.set_xlim(max(0,ts[-1]-30),max(30,ts[-1]+0.1))
            finite=[v for key,line in lines for v in self.series[key] if math.isfinite(v)]
            if finite:
                lo,hi=min(finite),max(finite)
                margin=max((hi-lo)*0.12,1e-3)
                ax.set_ylim(lo-margin,hi+margin)
        if self.mag_points:
            pts=list(self.mag_points)
            stride=max(1,len(pts)//1800)
            thin=pts[::stride]
            xx,yy,zz=zip(*thin)
            self.mag_scatter._offsets3d=(xx,yy,zz)
            self._draw_magnetic_axes(thin)
        else:
            self._draw_magnetic_axes([])
        self.canvas.draw_idle()

    def redraw_thermal(self):
        ts=list(self.times)
        if not ts:return
        for key,line in self.temp_lines:
            line.set_data(ts,list(self.series[key]))
        self.temp_plot.set_xlim(max(0,ts[-1]-60),max(60,ts[-1]+1))
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
            self.thermal_scatter.scatter(self.thermal_pressure_t[::3],self.thermal_pressure_p[::3],s=7,alpha=.45)
        self.temperature_canvas.draw_idle()

    def calculate(self):
        from tkinter import filedialog
        if self.analysis_running:
            self.msg.showinfo('Analysis already running','Please finish the current analysis first.')
            return
        # Live capture can continue while analysis runs, provided the user
        # selects only fully closed checkpoint segments, not the active writer.
        names=filedialog.askopenfilenames(parent=self.root,
            title='Select one or more completed calibration CSV files (Ctrl+click)',
            initialdir=str(ROOT/'captures'),filetypes=[('Calibration CSV','*.csv')])
        if not names:
            return
        paths=[Path(p).resolve() for p in names]
        if self.worker and self.worker.is_alive() and self.worker.current_file.resolve() in paths:
            self.msg.showwarning('Active CSV cannot be analyzed safely',
                'The current CSV is still being written. Click "Save CSV segment / continue" '
                'and select only the closed segment(s), or finish the current phase first.')
            return
        self.analysis_running=True
        self.calc.config(state='disabled')
        self.info.set(f'Analyzing {len(paths)} selected CSV file(s) in background; GUI remains responsive.')
        output_prefix=ROOT/'captures'/('combined_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
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

    def combine_calibrations(self):
        """Combine completed raw CSVs and saved result JSONs without pausing acquisition."""
        from tkinter import filedialog
        if self.analysis_running:
            self.msg.showinfo('Analysis running','Finish the current analysis first.')
            return
        names=filedialog.askopenfilenames(parent=self.root,
            title='Choose completed CSV and/or *_calibration.json files (Ctrl+click)',
            initialdir=str(ROOT/'captures'),
            filetypes=[('Calibration data and results','*.csv *.json'),('CSV recordings','*.csv'),('Saved calibration reports','*.json')])
        if not names:return
        paths=[Path(n).resolve() for n in names]
        if self.worker and self.worker.is_alive() and self.worker.current_file.resolve() in paths:
            self.msg.showwarning('Active CSV','Click Save CSV segment / continue, and choose closed CSV files.')
            return
        self.analysis_running=True
        self.calc.config(state='disabled')
        self.combine_button.config(state='disabled')
        target=ROOT/'captures'/('merged_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
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

    def assess_mag_quality(self):
        """Assess true held-out magnetic radial error without interrupting acquisition."""
        from tkinter import filedialog
        if self.analysis_running:
            self.msg.showinfo('Analysis running','Finish the current analysis first.')
            return
        names=filedialog.askopenfilenames(parent=self.root,
            title='Select one or more COMPLETED magnetometer CSV sessions',
            initialdir=str(ROOT/'captures'),filetypes=[('Calibration CSV','*.csv')])
        if not names:return
        paths=[Path(n).resolve() for n in names]
        if self.worker and self.worker.is_alive() and self.worker.current_file.resolve() in paths:
            self.msg.showwarning('Active recording','Save CSV segment / continue first, then select closed CSV files.')
            return
        self.analysis_running=True
        self.calc.config(state='disabled')
        self.mag_quality_button.config(state='disabled')
        self.info.set('Magnetometer quality study running: held-out fit, 3D coverage, bootstrap...')
        target=ROOT/'captures'/('mag_quality_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
        def job():
            try:
                cmd=[sys.executable,str(ROOT/'tools/magnetometer_quality.py'),
                     *[str(p) for p in paths],'--output-prefix',str(target)]
                res=subprocess.run(cmd,capture_output=True,text=True,timeout=360)
                self.events.put(('analysis_done',(res.returncode==0,(res.stdout+'\n'+res.stderr).strip())))
            except Exception as exc:
                self.events.put(('analysis_done',(False,str(exc))))
        threading.Thread(target=job,daemon=True).start()

    def close(self):
        if self.worker:
            self._save_skip_manifest()
            self.worker.stop_event.set()
            self.worker.join(timeout=3)
        self.root.destroy()

    def launch(self,port,axis_matrix):
        self.worker=Recorder(port,self.output,self.events,axis_matrix,
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
    ap.add_argument('--skip-lidar',action='store_true',help='Omit optional LiDAR stages entirely')
    ap.add_argument('--lidar-port',default='',help='Optional 3.3V USB-UART for TOFSense F2 Mini (not ESP32 COM port)')
    ap.add_argument('--lidar-samples',type=int,default=300,help='Fresh valid range readings per reference distance')
    ap.add_argument('--mag-only',action='store_true',help='Record only the four QMC/HMC 3D rotation stages; combine with full six-face CSV later')
    ap.add_argument('--settle-seconds',type=float,default=3.0,help='Preparation delay after Enter before recording (default 3s)')
    ap.add_argument('--self-test',action='store_true')
    ns=ap.parse_args()
    if ns.self_test:
        selftest()
        return
    if any(getattr(ns,n)<1 for n in ('gyro_samples','accel_samples','mag_turn_samples','mag_tumble_samples','bmp_samples','lidar_samples')):
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
    out=ns.output or ROOT/'captures'/('iq_500hz_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.csv')
    out=Path(out).resolve()
    import tkinter as tk
    ui=Application(tk,ns,stages(ns),axis_name,axis_matrix,out)
    ui.launch(ns.port,axis_matrix)


if __name__=='__main__':
    main()
