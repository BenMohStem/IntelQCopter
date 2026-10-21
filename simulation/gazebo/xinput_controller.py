"""Windows XInput -> the pinned FC's 16-byte Xbox report. No synthetic fallback."""
import ctypes as c
import json,struct,time

class Gamepad(c.Structure):
    _fields_=[('buttons',c.c_ushort),('lt',c.c_ubyte),('rt',c.c_ubyte),('lx',c.c_short),('ly',c.c_short),('rx',c.c_short),('ry',c.c_short)]
class State(c.Structure):
    _fields_=[('packet',c.c_uint32),('gamepad',Gamepad)]

def encode(g):
    # XInput Y is positive up; the BLE report uses the opposite sign.
    axis=lambda v:max(0,min(65535,int(v)+32768))
    hats={1:1,9:2,8:3,10:4,2:5,6:6,4:7,5:8}
    return struct.pack('<6H4B',axis(g.lx),axis(-g.ly),axis(g.rx),axis(-g.ry),round(g.lt*1023/255),round(g.rt*1023/255),hats.get(g.buttons&15,0),int(bool(g.buttons&0x1000))|2*int(bool(g.buttons&0x2000)),0,0)

class Controller:
    def __init__(self,index=None):
        self.index=index;self.dll=None
        for name in ['xinput1_4.dll','xinput1_3.dll','xinput9_1_0.dll']:
            try:self.dll=c.WinDLL(name);break
            except OSError:pass
        if self.dll is None:raise RuntimeError('Windows XInput runtime unavailable')
        self.dll.XInputGetState.argtypes=[c.c_uint32,c.POINTER(State)];self.dll.XInputGetState.restype=c.c_uint32
    def poll(self):
        for index in ([self.index] if self.index is not None else range(4)):
            state=State()
            if self.dll.XInputGetState(index,c.byref(state))==0:
                return encode(state.gamepad),dict(connected=True,index=index,packet=int(state.packet),buttons=int(state.gamepad.buttons),LT=int(state.gamepad.lt),RT=int(state.gamepad.rt))
        return None,dict(connected=False,index=self.index)

if __name__=='__main__':
    controller=Controller()
    for _ in range(5):
        raw,info=controller.poll();print(json.dumps(dict(**info,raw_hid_hex=raw.hex() if raw else None)),flush=True);time.sleep(.2)
