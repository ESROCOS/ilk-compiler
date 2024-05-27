#include "test-utils.h"


void kul::TextDataset::readPose(kul::pose_t& pose)
{
    readLine();
    pose.setIdentity();
    auto R = kul::eg_get_rotation(pose);
    auto p = kul::eg_get_position(pose);
    line_parser >> p(0) >> p(1) >> p(2);
    line_parser >> R(0,0) >> R(0,1) >> R(0,2)
           >> R(1,0) >> R(1,1) >> R(1,2)
           >> R(2,0) >> R(2,1) >> R(2,2);
    if( not line_parser.eof() ) {
        std::cerr << "Warning, end-of-line was expected after reading the pose data" << std::endl;
    }
    fresh_line = false;
}

void kul::TextDataset::readLine()
{
    if(fresh_line) return;
    std::string line;
    std::getline(source, line);
    source.peek(); // trigger EOF if it's over

    // replace commas with blanks to make the extraction operator work
    std::replace(line.begin(), line.end(), ',', ' ');
    line_parser.str(line);  // does not reset flags such as eof
    line_parser.seekg(std::ios_base::beg); // must do this to clear the eof, if reached previously

    fresh_line = true;
}


void kul::NaiveBinDataset::readPose(kul::pose_t& pose)
{
    readBuff(12);
    pose.setIdentity();
    auto R = kul::eg_get_rotation(pose);
    auto p = kul::eg_get_position(pose);
    p(0) = buf[0];
    p(1) = buf[1];
    p(2) = buf[2];

    // expect the rotation matrix elements in row-major order, in the buffer
    for(int r=0;r<3;r++) {
        for(int c=0;c<3;c++) {
            R(r,c) = buf[3 + r*3 + c];
        }
    }
}
