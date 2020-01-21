#ifndef ILKCOMPILER_EIGENBACKEND_OPERATORS_H_
#define ILKCOMPILER_EIGENBACKEND_OPERATORS_H_

namespace kul {

template<typename S = double>
inline twist_t Sdot_revolute(const twist_t& v)
{
    twist_t Sdot;
    Sdot << v(AY), -v(AX), 0.0, v(LY), -v(LX), 0.0;
    return Sdot;
}


template<typename S = double>
inline twist_t Sdot_prismatic(const twist_t& v)
{
    twist_t Sdot;
    Sdot << 0.0, 0.0, 0.0, v(AY), -v(AX), 0.0;
    return Sdot;
}

}

#endif
