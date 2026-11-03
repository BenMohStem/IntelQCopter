"""Offline-only v18 tests. No real ESP32/USB/supply is exercised."""
import contextlib
from io import StringIO
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import auto_tests as a


def simulated_lines(flags, hz=None, with_wifi=False, status_fail=0):
    hz=hz or flags['IQ_MAG_BOOT_PROFILE']
    params={'fw':18,**{k:flags[v] for k,v in a.STATE_KEYS.items()}}
    state='STATE '+' '.join(f'{k}={v}' for k,v in params.items())
    lines=['=== IQ v18 AUTOMATED SYSTEM DIAGNOSTIC ===',state,
           f'BUILD rate={flags["IQ_MAG_BOOT_PROFILE"]} serial={flags["IQ_SERIAL_BAUD"]} I2C={flags["IQ_I2C_HZ"]}']
    for n in range(1,36):
        active=int(with_wifi and n>=15)
        lines.append(f'HEALTH selected={hz} measured_hz_window={hz * 0.96:.2f} fresh={n*hz} '
                     f'status_fail={status_fail} xyz_fail=0 verify_fail=0 reset_signature=0 '
                     f'recoveries=0 regs_ok=1 fallback=0 soft_reset_attempted={flags["IQ_MAG_SOFT_RESET"]} '
                     f'soft_reset_ok={flags["IQ_MAG_SOFT_RESET"]} wifi_active={active} imu_ok=800 bmp_ok=80')
        lines.append(f'CSV,{n*1000000},100,200,300,0x19,1,{hz}')
        if n==15 and with_wifi:
            lines.append('WIFI_EVENT stage=READY mode=1 active=1 t_ms=15000')
    if flags['IQ_MAG_SOFT_RESET']:
        lines.append('MAG_SOFT_RESET stage=CONFIRMED ack=1 id=0x80 C1=0x00 C2=0x00')
    return lines


class Configuration(unittest.TestCase):
    def test_every_preset_is_documented_and_renderable(self):
        ini=(ROOT/'platformio.ini').read_text()
        for pid in a.PRESETS:
            flags=a.flags_for(pid,123456789)
            changed=a.render_ini(ini,flags)
            for key in a.BUILD_KEYS:
                self.assertIn(f'-D{key}={flags[key]}',changed,pid)
            self.assertIn('monitor_speed = '+str(flags['IQ_SERIAL_BAUD']),changed)
            self.assertEqual(flags['IQ_RUN_NONCE'],123456789)
        self.assertGreaterEqual(len(a.PRESETS),30)
    def test_powershell_launcher_splats_arguments_not_automatic_args(self):
        ps=(ROOT/'tools'/'run_auto_tests.ps1').read_text(encoding='utf-8')
        self.assertNotIn('@Args',ps)
        self.assertIn('& $VenvPython @RunArguments',ps)
        self.assertIn("'softreset'",ps)
        self.assertIn('& py -3 @RunArguments',ps)
        self.assertIn('& python @RunArguments',ps)

    def test_soft_reset_pair_stays_controlled(self):
        for base,reset in [('A0','G0'),('A1','G1'),('A2','G2'),('B1','G4'),('F1','G5'),('A3','G3')]:
            b=a.flags_for(base,55);r=a.flags_for(reset,55)
            self.assertEqual(r['IQ_MAG_SOFT_RESET'],1)
            self.assertEqual(b['IQ_MAG_SOFT_RESET'],0)
            r['IQ_MAG_SOFT_RESET']=0
            self.assertEqual(r,b)

    def test_presets_contrast_exact_variables(self):
        self.assertEqual(a.flags_for('A0')['IQ_MAG_BOOT_PROFILE'],50)
        self.assertEqual(a.flags_for('A1')['IQ_MAG_BOOT_PROFILE'],100)
        self.assertEqual(a.flags_for('A2')['IQ_MAG_BOOT_PROFILE'],200)
        self.assertEqual(a.flags_for('B1')['IQ_WIFI_STRESS_MODE'],1)
        self.assertEqual(a.flags_for('B2')['IQ_WIFI_STRESS_MODE'],2)
        self.assertEqual(a.flags_for('E1')['IQ_SERIAL_BAUD'],230400)
        self.assertEqual(a.flags_for('E2')['IQ_SERIAL_BAUD'],460800)
        self.assertEqual(a.flags_for('F1')['IQ_MAG_READ_STOP'],1)
    def test_ini_missing_flag_rejected(self):
        with self.assertRaisesRegex(ValueError,'Expected exactly one'):
            a.render_ini('hello',a.flags_for('A0'))
    def test_dry_run_no_files_or_ports(self):
        with patch.object(a,'choose_device',side_effect=AssertionError('should not touch ports')):
            with contextlib.redirect_stdout(StringIO()) as out:
                self.assertEqual(a.main(['--tests','A0,F1,E2','--dry-run']),0)
            self.assertIn('STOP+START',out.getvalue())
            self.assertIn('460800',out.getvalue())
    def test_ykush_command_off_on_and_restore(self):
        args=SimpleNamespace(power_mode='ykush',ykush_exe='/bin/echo',ykush_serial='',
                             ykush_port=1,off_detection_sec=4,off_seconds=10)
        calls=[]
        with patch.object(a.subprocess,'run',side_effect=lambda c,**kw: (calls.append(c) or SimpleNamespace(returncode=0,stdout='',stderr=''))), \
             patch.object(a,'locate_device',return_value=None), \
             patch.object(a.time,'sleep',return_value=None):
            ctrl=a.PowerController(args,lambda *args,**kw:None)
            ctrl.cold_boot(SimpleNamespace(), 'COM3')
            self.assertFalse(ctrl.switch_off)
            self.assertEqual(calls[0][-2:],['-d','1'])
            self.assertEqual(calls[1][-2:],['-u','1'])
    def test_ykush_port_does_not_disappear_aborts_and_rescues(self):
        args=SimpleNamespace(power_mode='ykush',ykush_exe='/bin/echo',ykush_serial='',
                             ykush_port=1,off_detection_sec=.1,off_seconds=1)
        calls=[]
        with patch.object(a.subprocess,'run',side_effect=lambda c,**kw: (calls.append(c) or SimpleNamespace(returncode=0,stdout='',stderr=''))), \
             patch.object(a,'locate_device',return_value='COM3'), \
             patch.object(a.time,'sleep',return_value=None):
            ctrl=a.PowerController(args,lambda *args,**kw:None)
            with self.assertRaises(a.PowerSwitchError):
                ctrl.cold_boot(SimpleNamespace(),'COM3')
            ctrl.rescue()
            self.assertEqual([c[-2] for c in calls],['-d','-u'])
    def test_power_json_validation(self):
        with tempfile.TemporaryDirectory() as td:
            fp=Path(td)/'commands.json'
            fp.write_text('{"off": "wrong", "on":["tool","on"]}')
            args=SimpleNamespace(power_mode='command',power_commands=str(fp))
            with self.assertRaises(ValueError):a.PowerController(args,lambda *args,**kwargs:None)


class LogAnalysis(unittest.TestCase):
    def cap(self,lines):
        return a.Capture('COM3',35,'ADAPTIVE_EVIDENCE_COLLECTED',False,len('\n'.join(lines)),lines)
    def test_clean_100_is_provisional_warm_only(self):
        flags=a.flags_for('A1',12345)
        lines=simulated_lines(flags)
        result=a.assess(lines,flags,'warm',self.cap(lines))
        self.assertIn('PROVISIONAL PASS',result['verdict'])
        self.assertIn('WARM-BOOT ONLY',result['verdict'])
        self.assertEqual(result['median_last_8_health_hz'],96.0)
        self.assertEqual(result['xyz_diagnostics']['count'],35)
        self.assertEqual(result['xyz_diagnostics']['x_y_z_mean_counts'],[100.,200.,300.])
    def test_config_nonce_mismatch_fails(self):
        flags=a.flags_for('A0',31337)
        lines=simulated_lines(flags)
        result=a.assess(lines,a.flags_for('A0',98989),'ykush',self.cap(lines))
        self.assertTrue(any('run_nonce' in s for s in result['errors']))
    def test_200_zero_data_is_not_pass(self):
        flags=a.flags_for('A2',500)
        lines=simulated_lines(flags,hz=0.01,status_fail=2291)
        lines=[s for s in lines if not s.startswith('CSV,')]
        result=a.assess(lines,flags,'ykush',self.cap(lines))
        self.assertIn('FAIL',result['verdict'])
        self.assertTrue(any('status_fail' in s for s in result['errors']))
    def test_wifi_before_after_collected(self):
        flags=a.flags_for('B1',909090)
        lines=simulated_lines(flags,with_wifi=True)
        result=a.assess(lines,flags,'ykush',self.cap(lines))
        self.assertIn('PROVISIONAL PASS',result['verdict'])
        self.assertGreater(result['xyz_diagnostics']['wifi_before']['count'],0)
        self.assertGreater(result['xyz_diagnostics']['wifi_after']['count'],0)
        self.assertEqual(result['xyz_diagnostics']['wifi_magnitude_delta_counts'],0)
    def test_soft_reset_confirmed_and_missing(self):
        flags=a.flags_for('G2',333)
        lines=simulated_lines(flags)
        good=a.assess(lines,flags,'warm',self.cap(lines))
        self.assertIn('PROVISIONAL PASS',good['verdict'])
        broken=[l.replace('soft_reset_ok=1','soft_reset_ok=0') for l in lines
                if not l.startswith('MAG_SOFT_RESET')]
        bad=a.assess(broken,flags,'warm',self.cap(broken))
        self.assertTrue(any('soft reset not confirmed' in e.lower() for e in bad['errors']))

    def test_fallback_disqualifies_200(self):
        flags=a.flags_for('A2',1)
        lines=simulated_lines(flags)
        lines+=['FALLBACK_100HZ: 200Hz NOT WORKING']
        r=a.assess(lines,flags,'warm',self.cap(lines))
        self.assertNotIn('PROVISIONAL PASS',r['verdict'])
    def test_csv_decimation_not_used_as_rate(self):
        flags=a.flags_for('A1',1)
        lines=simulated_lines(flags)
        lines=[l for i,l in enumerate(lines) if not l.startswith('CSV,') or i%2==1]
        r=a.assess(lines,flags,'warm',self.cap(lines))
        self.assertEqual(r['median_last_8_health_hz'],96.0)


class Regression18(unittest.TestCase):
    def test_prestart_parked_is_not_falsely_failed_for_zero_samples(self):
        f=a.flags_for('T0',333)
        lines=simulated_lines(f)
        lines += ['CONFIG_PARKED prestart_ok=1 chip_id=0x80 c1=0x00 c2=0x08']
        lines=[l for l in lines if not l.startswith('CSV,')]
        cap=a.Capture('COM3',6,'VERIFIED_PRESTART_ONLY',False,1000,lines)
        result=a.assess(lines,f,'warm',cap)
        self.assertTrue(result['verdict'].startswith('PRESTART_ONLY PASS'))
        self.assertIsNone(result['median_last_8_health_hz'])
        self.assertEqual(result['csv_rows'],0)

    def test_firmware_v18_signature_and_diagnostic_code(self):
        cpp=(ROOT/'src/main.cpp').read_text()
        self.assertIn('STATE fw=18',cpp)
        self.assertIn('POST_START_BUS',cpp)
        self.assertIn('CONFIG_PARKED',cpp)
        self.assertIn('readByteMode(MAG,ID,true)',cpp)
        self.assertIn('reason=IDENTITY_NOT_CONFIRMED',cpp)

    def test_continuous_mode_does_not_claim_fifty_hz(self):
        f=a.flags_for('H0',100)
        lines=simulated_lines(f,hz=250)
        cap=a.Capture('COM3',35,'ADAPTIVE_EVIDENCE_COLLECTED',False,1000,lines)
        result=a.assess(lines,f,'warm',cap)
        self.assertTrue(any('Continuous mode' in w for w in result['warnings']))
        self.assertFalse(any('82% of 50Hz' in x for x in result['errors']))

    def test_upload_port_busy_is_bounded_and_raises_fatal(self):
        import tempfile
        def fake_pio(cmd,**kwargs):
            kwargs['stdout'].write('Could not open COM3: PermissionError(13, Access is denied)\n')
            return SimpleNamespace(returncode=1)
        events=[]
        with tempfile.TemporaryDirectory() as d, \
             patch.object(a.subprocess,'run',side_effect=fake_pio), \
             patch.object(a.time,'sleep',return_value=None):
            with self.assertRaises(a.UploadPortBusyError):
                a.execute_pio('pio','COM3',Path(d)/'platformio.log',
                              lambda name,**kw:events.append(name),attempts=3)
        self.assertEqual(events.count('UPLOAD_START'),3)
        self.assertEqual(events.count('PORT_BUSY_UPLOAD_RETRY'),2)

    def test_port_busy_aborts_rest_of_suite_and_keeps_error_archive(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)
            original=(ROOT/'platformio.ini').read_text(encoding='utf-8')
            (root/'platformio.ini').write_text(original)
            board=SimpleNamespace(device='COM3',vid=0x1a86,pid=0x55d3,
                                  serial_number='CH343',location='1-2')
            uploads=[]
            def blocked(*args,**kwargs):
                uploads.append(1)
                raise a.UploadPortBusyError('COM3 PermissionError(13) Access is denied')
            with patch.multiple(a, ROOT=root, INI=root/'platformio.ini', RESULTS=root/'results'), \
                 patch.object(a,'platformio_cmd',return_value='pio'), \
                 patch.object(a,'choose_device',return_value=board), \
                 patch.object(a,'execute_pio',side_effect=blocked), \
                 patch.dict(sys.modules,{'serial':SimpleNamespace()}), \
                 contextlib.redirect_stdout(StringIO()):
                self.assertEqual(a.main(['--tests','A0,A1,A2','--port','COM3']),1)
            self.assertEqual(len(uploads),1)
            archives=list((root/'results').glob('*_SEND_THIS.zip'))
            self.assertEqual(len(archives),1)
            sessions=[x for x in (root/'results').glob('SESSION_*') if x.is_dir()]
            self.assertEqual(len(sessions),1)
            import json
            manifest=json.loads((sessions[0]/'manifest.json').read_text())
            self.assertFalse(manifest['complete'])
            self.assertEqual(len(manifest['runs']),1)
            self.assertEqual((root/'platformio.ini').read_text(),original)

    def test_non_busy_compile_error_not_retried(self):
        def fake_pio(cmd,**kwargs):
            kwargs['stdout'].write('error: ESP32 undeclared identifier\n')
            return SimpleNamespace(returncode=1)
        events=[]
        with tempfile.TemporaryDirectory() as d, patch.object(a.subprocess,'run',side_effect=fake_pio):
            with self.assertRaisesRegex(RuntimeError,'non-port error'):
                a.execute_pio('pio','COM3',Path(d)/'platformio.log',
                              lambda name,**kw:events.append(name))
        self.assertEqual(events.count('UPLOAD_START'),1)


class Integration(unittest.TestCase):
    def test_fake_bench_produces_zip_and_restores_ini(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)
            (root/'platformio.ini').write_text((ROOT/'platformio.ini').read_text(encoding='utf-8'))
            original=(root/'platformio.ini').read_bytes()
            board=SimpleNamespace(device='COM3',vid=0x1a86,pid=0x55d3,
                                  serial_number='CH343',location='1-2')
            fake_serial=SimpleNamespace()
            def fake_capture(conn, port, path, args, wifi, event):
                actual=a.render_ini((root/'platformio.ini').read_text(),a.flags_for('A0'))
                nonce=int(__import__('re').search(r'IQ_RUN_NONCE=(\d+)',(root/'platformio.ini').read_text()).group(1))
                chosen='A0' if wifi==0 else 'B1'
                flags=a.flags_for(chosen,nonce)
                lines=simulated_lines(flags,with_wifi=bool(wifi))
                path.parent.mkdir(parents=True,exist_ok=True)
                path.write_text('\n'.join(lines))
                return a.Capture(port,35,'ADAPTIVE_EVIDENCE_COLLECTED',False,len('\n'.join(lines)),lines)
            with patch.multiple(a,ROOT=root,INI=root/'platformio.ini',RESULTS=root/'results'), \
                 patch.object(a,'platformio_cmd',return_value='pio'), \
                 patch.object(a,'choose_device',return_value=board), \
                 patch.object(a,'wait_for_device',return_value=(fake_serial,'COM3')), \
                 patch.object(a,'execute_pio',return_value=None), \
                 patch.object(a,'capture_stream',side_effect=fake_capture), \
                 patch.dict(sys.modules,{'serial':SimpleNamespace()}):
                with contextlib.redirect_stdout(StringIO()):
                    self.assertEqual(a.main(['--tests','A0,B1','--port','COM3','--power-mode','warm']),0)
                self.assertEqual((root/'platformio.ini').read_bytes(),original)
                packs=list((root/'results').glob('*_SEND_THIS.zip'))
                self.assertEqual(len(packs),1)
                import zipfile, json
                with zipfile.ZipFile(packs[0]) as z:
                    self.assertTrue(any(x.endswith('SUMMARY.csv') for x in z.namelist()))
                    self.assertTrue(any(x.endswith('uart_raw.log') for x in z.namelist()))
                sessions=list((root/'results').glob('SESSION_*'))
                sessions=[p for p in sessions if p.is_dir()]
                self.assertEqual(len(sessions),1)
                manifest=json.loads((sessions[0]/'manifest.json').read_text())
                self.assertEqual(len(manifest['runs']),2)
                self.assertTrue(manifest['complete'])
                for r in manifest['runs']:
                    self.assertIn('WARM-BOOT ONLY',r['summary']['verdict'])


if __name__=='__main__':
    unittest.main()
