"""Explicit synthetic depth unit conversion; never change capture time.

Fortress RGBD emits float32 metres, while the production D435 validity gate
expects uint16 millimetres. Invalid/no-return samples remain invalid zero.
"""
import numpy as np


def millimetres(encoding,width,height,step,bigendian,data):
    if encoding!='32FC1' or width<=0 or height<=0:
        raise ValueError('expected nonempty Fortress32FC1 metre image')
    if step<width*4 or step%4 or len(data)!=height*step:
        raise ValueError('malformed float depth buffer')
    values=np.frombuffer(data,dtype='>f4' if bigendian else '<f4').reshape(height,step//4)[:,:width]
    rounded=np.rint(values.astype(np.float64)*1000)
    valid=np.isfinite(values)&(rounded>0)&(rounded<=65535)
    result=np.zeros(values.shape,dtype='<u2')
    result[valid]=rounded[valid].astype(np.uint16)
    return result.tobytes()
