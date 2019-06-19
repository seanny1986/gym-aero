/* Copyright 2017 The MathWorks, Inc. */

#ifndef ENGINE_EXCEPTION_HPP
#define ENGINE_EXCEPTION_HPP

#include <MatlabExecutionInterface/exception.hpp>
#include <MatlabExecutionInterface/detail/exception_impl.hpp>

namespace matlab {

    namespace engine {

        using Exception = matlab::execution::Exception;
        using MATLABNotAvailableException = matlab::execution::MATLABNotAvailableException;
        using CancelledException = matlab::execution::CancelledException;
        using InterruptedException = matlab::execution::InterruptedException;
        using MATLABException = matlab::execution::MATLABException;
        using MATLABSyntaxException = matlab::execution::MATLABSyntaxException;
        using MATLABExecutionException = matlab::execution::MATLABExecutionException;
        using TypeConversionException = matlab::execution::TypeConversionException;
        using StackFrame = matlab::execution::StackFrame;
            
        class EngineException final: public matlab::execution::Exception {
        public:
            EngineException();
            
            /**
            * Constructor that accepts an error message
            */
            EngineException(const std::string& msg);
        };
        
    }
}

#endif /* ENGINE_EXCEPTION_HPP */
