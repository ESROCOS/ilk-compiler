#ifndef KUL_ESROCOS_ILK_GENERATOR_EIGEN_BACKEND_TEST_UTILS
#define KUL_ESROCOS_ILK_GENERATOR_EIGEN_BACKEND_TEST_UTILS

#include <iostream>
#include <fstream>
#include <exception>

#include "core-types.h"

namespace kul
{

/**
 * Trivial wrapper of a test dataset in text format.
 *
 * A test dataset contains the coefficients of an homogeneous transformation
 * matrix for different values of the variables/parameters the matrix depends
 * on.
 *
 * The expected format of the underlying data file is one entry on each
 * line, with space-separated values.
 * Each entry shall contain a value for each variable/parameter the matrix
 * depends on, but there are no constraints here about the ordering nor the
 * count of these values. This is because this class provides a generic
 * 'readVector' to read N values, and it is the user's task to call it correctly
 * depending on the actual format of the dataset.
 *
 * On the other hand, the homogeneous matrix coefficients MUST be stored with
 * the following layout: first the three elements of the position
 * vector, than the nine elements of the rotation matrix, row wise.
 * These coefficients MUST be stored _after_ the values for the
 * variables/parameters.
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
            line_parser >> out(c);
        }
    }

private:
    void readLine();

    std::ifstream source;
    std::istringstream line_parser;
    bool fresh_line{false};
};

/**
 * A wrapper for a test dataset in binary format.
 * For the general, expected data format see `TextDataset`. Every value must be
 * encoded as a 4-byte float in machine's endianess. There are no separators
 * between subsequent entries.
 */
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

