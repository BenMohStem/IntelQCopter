import socket,struct
import numpy as np

class RosFlight:
    def __init__(self):self.stream=socket.create_connection(('127.0.0.1',9241),timeout=30)
    def exact(self,n):
        data=b''
        while len(data)<n:
            part=self.stream.recv(n-len(data))
            if not part:raise EOFError('ROS transport closed')
            data+=part
        return data
    def call(self,op,values):
        a=np.asarray(values,dtype='<f4');self.stream.sendall(struct.pack('<II',op,len(a))+a.tobytes())
        count=struct.unpack('<I',self.exact(4))[0]
        return np.frombuffer(self.exact(count*4),dtype='<f4')
    def qc_sitl_reset(self,hover,height,gains):self.call(1,np.r_[hover,height,gains])
    def qc_sitl_step(self,sensor,command,dt,flags,motors,diag):
        reply=self.call(2,np.r_[sensor,command,dt,flags]);motors[:]=reply[:4];diag[:]=reply[4:]
    def qc_sitl_height_observation(self,height,variance):self.call(3,[height,variance])
    def selected_mission(self):return self.call(4,[])
    def close(self):self.stream.sendall(struct.pack('<II',9,0));self.stream.close()
