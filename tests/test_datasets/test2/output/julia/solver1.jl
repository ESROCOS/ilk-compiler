import ILKBackend
import ur5
import NaiveDataset
import LinearAlgebra

mc = ur5.ModelConstants

function test_fk_dataset( ds::NaiveDataset.BinDataset )

    err_pos = 0;
    err_ori = 0;
    count   = 0;
    qd = ILKBackend.matrix(6,1)
    while ! ds.eof
        q = NaiveDataset.readVector(ds, UInt16(6));
        given_1 = NaiveDataset.readPose(ds);
        wrist_3__base,v__wrist_3__base,J_wrist_3_base = ur5.solver1(mc, q, qd);

        Rdiff = ILKBackend.orientationDistance(
                    ILKBackend.rotationView(wrist_3__base),
                    ILKBackend.rotationView(given_1))
        err_ori += Rdiff.angle;
        err_pos += LinearAlgebra.norm(ILKBackend.positionView(wrist_3__base-given_1));
        count = count + 1;
    end

    err_pos /= count
    err_ori /= count

    println("Average position error   : " * string(err_pos))
    println("Average orientation error: " * string(err_ori))
end

if abspath(PROGRAM_FILE) == @__FILE__
    if length(Base.ARGS) < 1
        print("Please provide a dataset file")
        exit(-1)
    end
    filename = Base.ARGS[1]
    ds = NaiveDataset.openBinDataset( filename )

    test_fk_dataset( ds )
end