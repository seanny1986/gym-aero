/* Copyright 2014-2017 The MathWorks, Inc. */

#ifndef MATLAB_DATA_OBJECT_ARRAY_HPP
#define MATLAB_DATA_OBJECT_ARRAY_HPP

#include "TypedArray.hpp"

namespace matlab {
    namespace data {
        class Object;
        using ObjectArray = TypedArray<Object>;
    }
}

#endif
