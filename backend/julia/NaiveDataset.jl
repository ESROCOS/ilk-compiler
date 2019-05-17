module NaiveDataset

import ILKBackend

export BinDataset, openBinDataset, readVector, readPose

mutable struct BinDataset

    istream
    eof::Bool
    buf
end

function openBinDataset(filename::String)
        i = open(filename, "r")
        buf = Array{Float32, 1}(undef,128);
        return BinDataset(i, false, buf)
end

function readVector(ds::BinDataset, size::UInt16)
    readBuff(ds, size)
    buf = Array{Float32, 1}(undef, size)
    buf[:] = ds.buf[1:size]
    return buf
end

function readPose(ds::BinDataset)
    if ! readBuff(ds, UInt16(12))
        return None
    end
    pose = ILKBackend.pose()
    R = ILKBackend.rotationView(pose);
    p = ILKBackend.positionView(pose);
    p[1] = ds.buf[1];
    p[2] = ds.buf[2];
    p[3] = ds.buf[3];

    # expect the rotation matrix elements in row-major order, in the buffer
    for r in 1:3
        for c in 1:3
            R[r,c] = ds.buf[r*3 + c];
        end
    end
    return pose
end

sizeOfFloatInDataset = sizeof(Float32)
bytes = Array{UInt8, 1}(undef, 64)

function readBuff(dataset::BinDataset, howmany::UInt16)
    if howmany == 0
        return
    end


    bytesCount  = howmany*sizeOfFloatInDataset
    actualCount = readbytes!(dataset.istream, bytes, bytesCount )

    if actualCount==0
        dataset.eof = true
        return False
    end
    if actualCount<bytesCount
        print("Warning, unexpected number of bytes read. Assuming EOF")
        dataset.eof = true
        return False
    end

    # Decode the raw bytes into floats
    for f in 0:howmany-1
        buffer = IOBuffer(bytes[f*sizeOfFloatInDataset+1:(f+1)*sizeOfFloatInDataset+1])
        dataset.buf[f+1] = read(buffer, Float32)
    end
    dataset.eof = eof(dataset.istream)

    return true
end

end
