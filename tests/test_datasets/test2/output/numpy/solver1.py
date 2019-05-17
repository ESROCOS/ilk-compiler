from __future__ import print_function
import sys
import numpy as np

from ilknumpy import backend as backend
from ilknumpy import dataset
from ur5 import ur5 as rob

mc = rob.ModelConstants()

if __name__ == "__main__" :

    if len(sys.argv) < 2 :
        print("Please provide a dataset file", file=sys.stderr)
        exit(-1)

    data = dataset.NaiveBinDataset(sys.argv[1]);

    err_pos = 0;
    err_ori = 0;
    count   = 0;

    qd = np.zeros(6)

    while not data.eof :
        q = data.readVector(6)
        given_1 = data.readPose()
        wrist_3__base,v__wrist_3__base,J_wrist_3_base = rob.solver1(mc, q, qd)

        Rdiff = backend.orientationDistance(
                    backend.rotationView(wrist_3__base),
                    backend.rotationView(given_1))
        err_ori += Rdiff.angle
        err_pos += np.linalg.norm(backend.positionView(wrist_3__base-given_1))
        count = count + 1

    err_pos /= count
    err_ori /= count

    print("Average position error   : {0}".format(err_pos))
    print("Average orientation error: {0}".format(err_ori))

