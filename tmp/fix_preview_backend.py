from pathlib import Path
import json
r=Path(r'C:\Users\Mohamed\Desktop\Work\QCopter-Stepwise')
p=r/'tools/visualization/attitude_preview.py'
s=p.read_text().replace('import serial\nimport matplotlib.pyplot as plt','''import serial
import os
import matplotlib
try:
    import PyQt5
except ImportError as error:
    raise SystemExit('Interactive plotting needs PyQt5: install PyQt5==5.15.11 in the Python environment used by qcopter.ps1') from error
# Use this interpreter's Qt plugins, even when started from a ROS terminal.
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH']=str(Path(PyQt5.__file__).parent/'Qt5/plugins/platforms')
os.environ['QT_API']='pyqt5'
matplotlib.use('QtAgg',force=True)
import matplotlib.pyplot as plt''')
p.write_text(s,encoding='utf-8',newline='\n')
event={'schema':2,'id':'2026-10-30-preview-interactive-backend','date':'2026-10-30','domains':['operator-revision'],'summary':'User upload succeeds but preview exits with FigureCanvasAgg noninteractive.','outcome':'Viewer now explicitly selects QtAgg/PyQt5 before pyplot and uses its Qt plugin path. Missing PyQt5 installed in existing runner environment. No firmware upload needed.'}
(r/'knowledge/events/2026-10-30-preview-interactive-backend.json').write_text(json.dumps(event,indent=2)+'\n')
