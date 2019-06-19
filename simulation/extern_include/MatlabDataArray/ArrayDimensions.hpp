/* Copyright 2015 MathWorks, Inc. */

#ifndef MATLAB_DATA_ARRAY_DIMENSIONS_HPP
#define MATLAB_DATA_ARRAY_DIMENSIONS_HPP

#include "detail/publish_util.hpp"

#include <vector>
#include <cstdlib>

namespace matlab {
    namespace data {

        using ArrayDimensions = std::vector<size_t>;

        /**
         * Determine the number of elements based on the ArrayDimensions.
         *
         * @param dims - the dimensions
         * @return size_t - the number of elements
         * @throw - none.
         */
        inline size_t getNumElements(const ArrayDimensions& dims) MW_NOEXCEPT {
            size_t retVal = 1;
            for (auto& elem : dims) {
                retVal *= elem;
            }
            return retVal;
        }
    }
}

#endif
