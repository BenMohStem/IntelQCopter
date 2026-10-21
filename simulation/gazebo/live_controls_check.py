"""Deterministic XInput-to-HID encoding checks; no flight or Bluetooth claim."""
import ctypes,json,struct
from xinput_controller import Gamepad,State,encode

def main():
    checks=[]
    def check(name,condition):
        checks.append(dict(name=name,passed=bool(condition)))
        print(('PASS ' if condition else 'FAIL ')+name)
    g=Gamepad();p=encode(g);v=struct.unpack('<6H4B',p)
    check('Windows XINPUT_STATE structure layout',ctypes.sizeof(State)==16 and State.gamepad.offset==4)
    check('Neutral report matches FC centre',v==(32768,32768,32768,32768,0,0,0,0,0,0))
    g.lx=-32768;g.ly=32767;g.rx=32767;g.ry=-32768;g.lt=255;g.rt=255;g.buttons=2|0x1000|0x2000
    v=struct.unpack('<6H4B',encode(g))
    check('Stick endpoints and Y inversion',v[:4]==(0,1,65535,65535))
    check('Triggers span FC 0..1023',v[4:6]==(1023,1023))
    check('Down plus A/B gestures retain button bits',v[6:8]==(5,3))
    for mask,hat in [(1,1),(9,2),(8,3),(10,4),(2,5),(6,6),(4,7),(5,8),(0,0)]:
        g.buttons=mask;check('D-pad mask '+str(mask),encode(g)[12]==hat)
    print(json.dumps(dict(scope=__doc__,checks=checks,passed=all(c['passed'] for c in checks))))
    return 0 if all(c['passed'] for c in checks) else 2
if __name__=='__main__':raise SystemExit(main())
