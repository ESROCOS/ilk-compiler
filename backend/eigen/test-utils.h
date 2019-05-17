#ifndef KUL_ESROCOS_ILK_GENERATOR_EIGEN_BACKEND_TEST_UTILS
#define KUL_ESROCOS_ILK_GENERATOR_EIGEN_BACKEND_TEST_UTILS

#include <iostream>
#include <fstream>
#include <exception>

#include "core-types.h"

namespace kul
{

/**
 * Trivial wrapper of a text dataset.
 *
 * The expected format is one data item on each line, with space-separated
 * values. For example, a 6 dimensional vector must be stored as 6 numbers
 * separated by space, on the same line.
 *
 */
class TextDataset
{
public:
    TextDataset(const std::string&  file) {
        source.open( file, std::ios::in);
        if( !source.is_open() ) {
            throw std::runtime_error("Could not open file " + file);
        }
    }

    ~TextDataset() {
        source.close();
    }

    bool eof() { return source.eof(); }

    /**
     * A pose must be stored as a 12 element vector, first the three elements of the
     * position vector, than the 9 elements of the rotation matrix, row wise.
     */
    void readPose(kul::pose_t& pose);

    template<typename V>
    void readVector(unsigned int count, V& out)
    {
        readLine();
        for(unsigned int c=0; c<count; c++) {
            reader >> out(c);
        }
    }

private:
    void readLine();

    std::ifstream source;
    std::istringstream reader;
};

class NaiveBinDataset
{
public:
    NaiveBinDataset(const std::string&  file) {
        source.open( file, std::ios::in | std::ios::binary);
        if( !source.is_open() ) {
            throw std::runtime_error("Could not open file " + file);
        }
    }

    ~NaiveBinDataset() {
        source.close();
    }

    bool eof() { return source.eof(); }

    /**
     * A pose must be stored as a 12 element vector, first the three elements of the
     * position vector, than the 9 elements of the rotation matrix, row wise.
     */
    void readPose(kul::pose_t& pose);

    template<typename V>
    void readVector(unsigned int count, V& out)
    {
        readBuff(count);
        for(unsigned int c=0; c<count; c++) {
            out(c) = buf[c];
        }
    }

private:
    void readBuff(unsigned short howmany) {
        source.read(reinterpret_cast<char*>(buf), howmany*sizeof(float));
        source.peek(); // make sure to trigger EOF now, if there are no more chars
    }

    float buf[64];
    std::ifstream source;
};

}



#endif

