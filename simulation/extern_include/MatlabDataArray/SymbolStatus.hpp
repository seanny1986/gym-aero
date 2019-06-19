/* Copyright 2017 The MathWorks, Inc. */

#ifndef SYMBOL_STATUS_HPP_
#define SYMBOL_STATUS_HPP_

namespace matlab {
    namespace data {
        enum class SymbolStatus : int {
            NOT_A_VARIABLE,
            IS_A_VARIABLE
        };
    }
}

#endif
