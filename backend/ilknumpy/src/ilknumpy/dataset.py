import math, sys, struct
import numpy as np
from ilknumpy import backend as utils




class NaiveBinDataset :
    sizeOfFloatInDataset = 4
    
    def __init__(self, filePath):
        self.f = open( filePath, mode='rb' )
        self._eof = False
        self.buf  = np.empty(512, np.float32)

    @property
    def eof(self) :
        return self._eof

    def foo(self):
        while not self.eof :
            self.readVector(6)
            #self.readPose()
        print("done")

    '''
     A pose must be stored as a 12 element vector, first the three elements of the
     position vector, than the 9 elements of the rotation matrix, row wise.
    '''
    def readPose(self) :
        if not self._readBuff(12) :
            return None
        pose = np.identity(4)
        R = utils.rotationView(pose);
        p = utils.positionView(pose);
        p[0] = self.buf[0];
        p[1] = self.buf[1];
        p[2] = self.buf[2];

        # expect the rotation matrix elements in row-major order, in the buffer
        for r in range(0,3) :
            for c in range(0,3) :
                R[r,c] = self.buf[3 + r*3 + c];
        return pose
        
        
    def readVector(self, size) :
        if not self._readBuff(size) :
            return None

        ret = np.empty(size)
        ret[:] = self.buf[0:size]        
        return ret

    def _readBuff(self, howmany) :
        if howmany == 0 : return

        sizeoff = NaiveBinDataset.sizeOfFloatInDataset
        bytesCount = howmany*sizeoff
        bytes = self.f.read( bytesCount )
        actualCount = len(bytes)
        
        if actualCount==0 :
            self._eof = True
            return False
        if actualCount<bytesCount :
            print("Warning, expected {0} bytes, read {1}. Assuming EOF".format(bytesCount, actualCount))
            self._eof = True
            return False

        # Decode the raw bytes into floats
        for f in range(0, howmany) :
            self.buf[f] = struct.unpack("f", bytes[f*sizeoff:(f+1)*sizeoff] )[0]

        # Let's look ahead and see whether we reached EOF
        dummy = self.f.peek(1)
        if len(dummy)==0:
            self._eof = True

        return True
