#ifndef MATLABCOMMUNICATE_H
#define MATLABCOMMUNICATE_H

#include <array>

namespace Settings
{
    constexpr std::size_t numValues{15};
};

// functions for getting flight sim data from MATLAB for testing purposes

bool handshake();
// returns false if handshake unsuccesful

std::array<double, Settings::numValues> getData();
// gets the data from Serial and puts it into an array




#endif