#!/usr/bin/env python3
"""Original illustrative figures: NEVER imply that timing traces are measured."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
out=Path(__file__).resolve().parent.parent/'analysis'
out.mkdir(exist_ok=True)
fig,ax=plt.subplots(figsize=(10,3.65))
nominal=np.arange(0,52,2.)
actual=nominal+np.where((nominal>=18)&(nominal<28),10.,0.)
# An exceptional 10-ms obstruction: old deadlines are dropped, scheduler phase remains tied to t0.
actual=np.array([0,2,4,6,8,10,12,14,16,18,28,30,32,34,36,38,40,42,44,46,48,50],float)
ax.vlines(nominal,1.65,2.05,color='#A7B7C9',linewidth=1,label='Desired 2 ms deadlines')
ax.scatter(actual,np.full(len(actual),1.25),s=26,color='#1662A2',label='Acquisition start (synthetic)')
ax.annotate('10 ms interruption\n(5 nominal intervals)',xy=(25,1.55),xytext=(19,.34),
            arrowprops=dict(arrowstyle='->',color='#A33A3A'),color='#A33A3A',fontsize=9)
ax.set_yticks([1.25,1.85],['Actual read start','Ideal deadlines'])
ax.set_ylim(.1,2.3);ax.set_xlim(-1,52);ax.set_xlabel('Microseconds × 1000 (illustrative)')
ax.grid(axis='x',alpha=.2);ax.legend(loc='upper right',fontsize=8)
ax.set_title('Absolute deadlines preserve phase; measured dt preserves physical elapsed time')
fig.tight_layout();fig.savefig(out/'synthetic_absolute_deadlines.png',dpi=190);plt.close(fig)
print(out/'synthetic_absolute_deadlines.png')
