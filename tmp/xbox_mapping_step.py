from pathlib import Path
import json
r=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise')
p=r/'lessons/xbox-test-menu/main.cpp'
s=p.read_text()
s=s.replace('    bool menu_open = true;\n','').replace('            menu_open = true;\n','').replace('            menu_open = false;\n','')
s=s.replace('    uint32_t last_status = 0;', '    uint32_t last_status = 0;\n    uint32_t last_input = 0;\n    bool inputs_only = false;')
s=s.replace('buttons = (xbox.xboxNotif.btnStart ? 1 : 0)', '// Operator observed these two library fields exchanged on this controller.\n            buttons = (xbox.xboxNotif.btnY ? 1 : 0)')
s=s.replace('| (xbox.xboxNotif.btnY ? 8 : 0)', '| (xbox.xboxNotif.btnStart ? 8 : 0)')
s=s.replace('        if (pressed & 2)', '        if ((pressed & 2) && !inputs_only)')
s=s.replace('        if (pressed & 4)', '        if ((pressed & 4) && !inputs_only)')
needle='''            if (command == '?')'''
s=s.replace(needle,'''            if (command == 'i' || command == 't')
            {
                inputs_only = command == 'i';
                Serial.printf("INPUT_MODE,%s\\n", inputs_only ? "MONITOR_ONLY_NO_TESTS" : "TEST_MENU");
            }
'''+needle)
s=s.replace("if (command == 'd' || command == 'f' || command == 'D' || command == 'F')", "if (!inputs_only && (command == 'd' || command == 'f' || command == 'D' || command == 'F'))")
needle='''        if (!dumping && (uint32_t)(now - last_status) >= 5000)'''
s=s.replace(needle,'''        if (inputs_only && input_ready && (uint32_t)(now - last_input) >= 200)
        {
            // Library field names are raw decoder labels, not yet physical acceptance.
            const auto &input = xbox.xboxNotif;
            Serial.printf("INPUT,rawA=%u,rawB=%u,rawX=%u,rawY=%u,rawStart=%u,LB=%u,RB=%u,UP=%u,DOWN=%u,LEFT=%u,RIGHT=%u,LX=%u,LY=%u,RX=%u,RY=%u,LT=%u,RT=%u\\n",
                input.btnA, input.btnB, input.btnX, input.btnY, input.btnStart,
                input.btnLB, input.btnRB, input.btnDirUp, input.btnDirDown,
                input.btnDirLeft, input.btnDirRight, input.joyLHori, input.joyLVert,
                input.joyRHori, input.joyRVert, input.trigLT, input.trigRT);
            last_input = now;
        }
'''+needle)
p.write_text(s,encoding='utf-8',newline='\n')
p=r/'tools/validation/record_xbox_menu.py'
s=p.read_text().replace("    args=parser.parse_args()", "    parser.add_argument('--inputs-only',action='store_true',help='Show raw controller inputs; disable test starts')\n    args=parser.parse_args()\n    if args.inputs_only and args.run_all:parser.error('Choose inputs-only or run-all')")
s=s.replace("        if pending:device.write(pending.pop(0).encode())", "        if args.inputs_only:\n            device.write(b'i')\n            print('INPUT MONITOR ONLY: tests disabled. Press each button separately, then move sticks/triggers.',flush=True)\n        if pending:device.write(pending.pop(0).encode())")
s=s.replace("            elif fields[0]=='ERROR':raise ValueError(value)", "            elif fields[0] in ('INPUT','INPUT_MODE'):\n                print(value,flush=True)\n            elif fields[0]=='ERROR':raise ValueError(value)")
s=s.replace("        device.close();log.close()", "        if device.is_open and args.inputs_only:device.write(b't')\n        device.close();log.close()")
p.write_text(s,encoding='utf-8',newline='\n')
out=r/'measurements/software/2026-10-30-xbox-mapping'
out.mkdir(exist_ok=True)
(out/'README.md').write_text('''# Next small step: verify physical controller inputs

The user reports physical Menu shows information and physical Y advances selection. The two library fields are therefore exchanged in the bench application's action mapping for this controller. This is operator evidence, not proof of a universal Xbox mapping. A/B and all analog axes still need observation. Upstream decoder is unchanged.

Latest operator session 20261030T185510Z contains menu events and connected status, but no START/DATA/DONE captures at inspection time. Earlier closed operator sessions contain zero runs. The recorder could still have been open; absence of completed results is not an accepted acquisition test. Original logs are preserved privately. The four previous automatic trials remain separate hardware evidence from an older source.

## Why these tests existed

Direct/FIFO normal tests establish acquisition throughput and observed service time. Artificial-pause tests ask whether delayed work loses readable samples or queues older samples. Xbox selection checks the human-to-BLE-to-request-queue path. Together these address missing/delayed PID inputs, not PID gains, attitude correctness or flight stability. No need to repeat artificial-pause trials as a learning exercise now.

## One short input check before continuing

```powershell
.\\qcopter.ps1 record-xbox-menu --port COM3 --inputs-only
```

No tests start in this mode. Press and release A, B, X, Y, Menu separately; then D-pad, LB/RB, sticks and triggers one at a time. Observe which raw field changes. rawY/rawStart are intentionally library labels: physical Menu is currently mapped from rawY, physical Y from rawStart. Report mismatches rather than assuming A/B are correct. Sticks should be near 32767 at rest; exact direction and endpoint limits require observation. Trigger endpoints are library/device dependent and must be measured. Ctrl+C saves original logs and returns to test-menu mode. Values print at 5 Hz; BLE input processing continues independently.

After this, validate gyro rotation directions with known gentle propeller-free rotations, then the attitude estimator, then PID calculations. No altitude acceptance until the barometer is available. No motors/PWM/PID enabled here. Current production firmware/calibration unchanged; its existing stale fingerprint is preserved.
''',encoding='utf-8')
event={'schema':2,'id':'2026-10-30-xbox-mapping','date':'2026-10-30','domains':['acquisition-performance'],'summary':'Operator reports Menu/Y roles exchanged and tests completed; local latest log has menu events but no acquisition records.','outcome':'Swap reported Menu/Y action fields locally; add optional raw input monitor with test starts disabled. Build/upload and physical validation tracked in report. No acquisition or flight acceptance inferred.'}
(r/'knowledge/events/2026-10-30-xbox-mapping.json').write_text(json.dumps(event,indent=2)+'\n')
state_path=r/'knowledge/state.json'
state=json.loads(state_path.read_text())
state['current_stage']='Xbox Menu/Y mismatch reported; next step is raw button/axis mapping in monitor-only mode, then physical gyro direction checks. Operator acquisition completion not evidenced in latest inspected log. See 2026-10-30-xbox-mapping report for new firmware validation.'
state_path.write_text(json.dumps(state,indent=2)+'\n')
with (r/'important_sources/README.md').open('a',encoding='utf-8') as f:
    f.write('\n[Next step: physical Xbox input mapping and why acquisition tests were needed](../measurements/software/2026-10-30-xbox-mapping/README.md). Includes monitor-only command; Menu/Y correction is controller-specific.\n')
