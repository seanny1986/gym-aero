/* Copyright 2016-2018 The MathWorks, Inc. */

#ifndef MATLAB_DATA_MEMORY_LAYOUT_HPP
#define MATLAB_DATA_MEMORY_LAYOUT_HPP


namespace matlab {
namespace data {
enum class MemoryLayout : int { COLUMN_MAJOR = 0, ROW_MAJOR, UNKNOWN };
}
} // namespace matlab

#endif
