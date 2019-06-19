/* Copyright 2017 The MathWorks, Inc. */

#ifndef __MEX_EXCEPTION_IMPL_CPP_HPP__
#define __MEX_EXCEPTION_IMPL_CPP_HPP__

#include "cppmex/mexException.hpp"
#include "mexEngineUtilImpl.hpp"
#include <sstream>

namespace matlab {
    namespace engine {

        /* MATLABException */
        inline Exception::Exception() :std::exception() {}

        inline Exception::Exception(const std::string& msg) : message(msg) {}

        inline Exception::~Exception() {}

        inline Exception& Exception::operator=(const Exception& rhs) {
            message = rhs.message;
            return *this;
        }

        inline const char* Exception::what() const NOEXCEPT { return message.c_str(); }

    
        /* MATLABSyntaxException */
        inline MATLABException::MATLABException() : Exception() {}

        inline MATLABException::~MATLABException() {};

        inline MATLABException::MATLABException(const std::string& msg) : Exception(msg) {}

        inline MATLABException::MATLABException(const std::string& id, const std::basic_string<char16_t>& txt):Exception(convertUTF16StringToUTF8String(txt)), errorID(id), errorText(txt) {}

        inline MATLABException::MATLABException(const MATLABException& rhs) : Exception(convertUTF16StringToUTF8String(rhs.errorText)), errorID(rhs.errorID), errorText(rhs.errorText) {}

        inline MATLABException& MATLABException::operator=(const MATLABException& rhs) {
            Exception::operator=(convertUTF16StringToUTF8String(rhs.errorText));
            errorID = rhs.errorID;
            errorText = rhs.errorText;
            return *this;
        }

        inline std::string MATLABException::getMessageID() const { return errorID; }

        inline std::basic_string<char16_t> MATLABException::getMessageText() const { return errorText; }


        /* MATLABSyntaxException */
        inline MATLABSyntaxException::MATLABSyntaxException(const std::string& msg) : MATLABException(msg) {}

        inline MATLABSyntaxException::MATLABSyntaxException(const std::string& id, const std::basic_string<char16_t>& txt) : MATLABException(id, txt) {}


        /* MATLABExecutionException */
        inline MATLABExecutionException::MATLABExecutionException() : MATLABException() {}

        inline MATLABExecutionException::MATLABExecutionException(const std::string& msg, const std::vector<StackFrame>& trace) : MATLABException(msg), stackTrace(trace) {}

        inline MATLABExecutionException::MATLABExecutionException(const std::string& id,
                                                                  const std::basic_string<char16_t>& txt,
                                                                  const std::vector<StackFrame>& trace,
                                                                  const std::vector<MATLABExecutionException>& cause) : MATLABException(id, txt), stackTrace(trace), errorCause(cause) {}

        inline std::vector<StackFrame> MATLABExecutionException::getStackTrace() const { return stackTrace; }

        inline std::vector<MATLABExecutionException> MATLABExecutionException::getCause() const { return errorCause; }

        inline void MATLABExecutionException::setCause(const std::vector<MATLABExecutionException>& cause) { errorCause = cause; }



        /* TypeConversionException */
        inline TypeConversionException::TypeConversionException() : Exception() {}

        inline TypeConversionException::TypeConversionException(const std::string& msg) : Exception(msg) {}

    
        /* StackFrame */
        inline StackFrame::StackFrame() {}

        inline StackFrame::StackFrame(const std::basic_string<char16_t>& file, const std::basic_string<char16_t>& func, uint64_t line) :fileName(file), funcName(func), lineNumber(line) {}

        inline std::basic_string<char16_t> StackFrame::getFileName() const {
            return fileName;
        }

        inline std::basic_string<char16_t> StackFrame::getFunctionName() const {
            return funcName;
        }

        inline uint64_t StackFrame::getLineNumber() const {
            return lineNumber;
        }

        inline CancelledException::CancelledException() : MATLABException() {}

        inline CancelledException::~CancelledException() {}

        inline CancelledException::CancelledException(const std::string& id, const std::u16string& msg) : MATLABException(id, msg) {}

        inline InterruptedException::InterruptedException() : MATLABException() {}

        inline InterruptedException::~InterruptedException() {}

        inline InterruptedException::InterruptedException(const std::string& id, const std::u16string& msg) : MATLABException(id, msg) {}
    }
}
    
#endif
