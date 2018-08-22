#include "test-utils.h"


void kul::TextDataset::readPose(kul::pose_t& pose)
{
    readLine();
    pose.setIdentity();
    auto R = kul::eg_get_rotation(pose);
    auto p = kul::eg_get_position(pose);
    reader >> p(0) >> p(1) >> p(2);
    reader >> R(0,0) >> R(0,1) >> R(0,2)
           >> R(1,0) >> R(1,1) >> R(1,2)
           >> R(2,0) >> R(2,1) >> R(2,2);
}

void kul::TextDataset::readLine()
{
    std::string line;
    std::getline(source, line);
    reader.str(line);
    reader.seekg(std::ios_base::beg);
}