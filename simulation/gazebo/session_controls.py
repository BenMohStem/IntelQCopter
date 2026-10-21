"""Explicit PC simulator actions encoded as ordinary Xbox HID gestures.

Never used by physical firmware. Real controller connection is still required.
Injected mode/disarm reports are labelled separately in the transport evidence.
"""
import json,struct,time

def shape_report(raw,profile):
    if profile!='gentle' or len(raw)!=16:return raw
    values=list(struct.unpack('<6H4B',raw))
    for i,gain in enumerate([.25,.25,5/35,5/35]):
        q=(values[i]-32768)/32768
        z=min(1,max(0,(abs(q)-.12)/.88))**2*gain
        shaped=0 if z==0 else (.12+.88*z)*(1 if q>=0 else -1)
        values[i]=max(0,min(65535,round(32768+32768*shaped)))
    if not 4<=values[6]<=6:values[4]=round(values[4]*.2)
    values[5]=round(values[5]*.2)
    return struct.pack('<6H4B',*values)

class SessionControls:
    def __init__(self,viewer):self.viewer=viewer;self.active=None;self.started=0;self.last=''
    def step(self,hid,now,info):
        if not info.get('connected') or (info.get('wall_time_s') and time.time()-info['wall_time_s']>.5):
            if self.active:self.last='Cancelled: Xbox disconnected'
            self.active=None
            self.viewer.next_action()
            return [],dict(active=False,message=self.last or 'Connect Xbox to continue',progress=0),False
        if self.active and any(len(p)==16 and 4<=p[12]<=6 and int.from_bytes(p[8:10],'little')>=921 for p in hid):
            self.active=None;self.last='PC gesture cancelled by Xbox disarm input'
            return hid,dict(active=False,message=self.last,progress=0),False
        if self.active is None:
            action=self.viewer.next_action()
            if action:self.active=action;self.started=now
        if self.active is None:return [shape_report(p,getattr(self.viewer,'profile','normal')) for p in hid],dict(active=False,message=self.last,progress=0),False
        duration=2.12 if self.active=='disarm' else 5.12;elapsed=now-self.started
        if elapsed>=duration:
            self.last={'manual':'Stable selected — gently push LEFT stick up to take off','navigation':'Navigation selected — waiting for an approved route','disarm':'Disarmed — restart session to fly again'}[self.active]
            self.active=None
            # Release the gesture before forwarding real controller input again.
            return [struct.pack('<6H4B',32768,32768,32768,32768,0,0,0,0,0,0)],dict(active=False,message=self.last,progress=1),True
        action=self.active;raw=struct.pack('<6H4B',32768,32768,32768,32768,1023 if action=='disarm' else 0,0,5,1 if action=='manual' else 2 if action=='navigation' else 0,0,0)
        return [raw],dict(active=True,action=action,message='Sending simulator '+action+' gesture',progress=min(1,elapsed/duration),remaining_s=max(0,duration-elapsed)),True
