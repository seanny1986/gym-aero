/**
 * Published header for C++ MEX Exception
 *
 * Copyright 2017 The MathWorks, Inc.
 */

#ifndef __MEX_EXCEPTION_TYPE_HPP__
#define __MEX_EXCEPTION_TYPE_HPP__

namespace matlab {
    namespace mex {
        namespace detail {
            enum class ErrorType: int { 
                NoException = 0,
                RuntimeError,
                SyntaxError,
                ExecutionError,
                EngineError,
                ThreadError,
                OutOfMemory, 
                CancelError,
                InterruptedError,
                SystemError
            };
        }
    }
}


#endif