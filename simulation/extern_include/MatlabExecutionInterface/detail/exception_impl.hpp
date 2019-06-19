/* Copyright 2017 The MathWorks, Inc. */

#ifndef MATLAB_EXECUTION_EXCEPTION_IMPL_HPP
#define MATLAB_EXECUTION_EXCEPTION_IMPL_HPP


#include "../exception.hpp"
#include "../util.hpp"
#include <vector>
#include <streambuf>
#include <memory>
#include <future>
#include <string>




namespace matlab {
    namespace execution { 

        inline Exception::Exception() :std::exception() {}

        inline Exception::Exception(const std::string& msg) : message(msg) {}

        inline Exception::~Exception() {}

        inline Exception& Exception::operator=(const Exception& rhs) {
            message = rhs.message;
            return *this;
        }

        inline const char* Exception::what() const NOEXCEPT { return message.c_str(); }

        inline MATLABNotAvailableException::MATLABNotAvailableException() : Exception() {}

        inline MATLABNotAvailableException::MATLABNotAvailableException(const std::string& msg) : Exception(msg) {}

        inline CancelledException::CancelledException() : Exception() {}

        inline CancelledException::CancelledException(const std::string& msg) : Exception(msg) {}

        inline InterruptedException::InterruptedException() : Exception() {}

        inline InterruptedException::InterruptedException(const std::string& msg) : Exception(msg) {}

        inline MATLABException::MATLABException() : Exception() {}

        inline MATLABException::MATLABException(const std::string& msg) : Exception(msg) {}

        inline MATLABException::MATLABException(const std::string& id, const std::u16string& txt):Exception(convertUTF16StringToUTF8String(txt)), errorID(id), errorText(txt) {}

        inline MATLABException::MATLABException(const MATLABException& rhs) : Exception(convertUTF16StringToUTF8String(rhs.errorText)), errorID(rhs.errorID), errorText(rhs.errorText) {}

        inline MATLABException& MATLABException::operator=(const MATLABException& rhs) {
            Exception::operator=(convertUTF16StringToUTF8String(rhs.errorText));
            errorID = rhs.errorID;
            errorText = rhs.errorText;
            return *this;
        }

        inline std::string MATLABException::getMessageID() const { return errorID; }

        inline std::u16string MATLABException::getMessageText() const { return errorText; }

        inline MATLABSyntaxException::MATLABSyntaxException(const std::string& msg) : MATLABException(msg) {}

        inline MATLABSyntaxException::MATLABSyntaxException(const std::string& id, const std::u16string& txt) : MATLABException(id, txt) {}

        inline MATLABExecutionException::MATLABExecutionException() : MATLABException() {}

        inline MATLABExecutionException::MATLABExecutionException(const std::string& msg, const std::vector<StackFrame>& trace) : MATLABException(msg), stackTrace(trace) {}

        inline MATLABExecutionException::MATLABExecutionException(const std::string& id, const std::u16string& txt, const std::vector<StackFrame>& trace, const std::vector<MATLABExecutionException>& cause) : MATLABException(id, txt), stackTrace(trace), errorCause(cause) {
        }

        inline std::vector<StackFrame> MATLABExecutionException::getStackTrace() const { return stackTrace; }

        /**
        * Get the cause of the MATLAB exception
        */
        inline std::vector<MATLABExecutionException> MATLABExecutionException::getCause() const { return errorCause; }

        /**
        * Set the cause of the MATLAB exception
        */
        inline void MATLABExecutionException::setCause(const std::vector<MATLABExecutionException>& cause) { errorCause = cause; }

        inline TypeConversionException::TypeConversionException() : Exception() {}

        inline TypeConversionException::TypeConversionException(const std::string& msg) : Exception(msg) {}

        inline StackFrame::StackFrame() {}

        inline StackFrame::StackFrame(const std::u16string& file, const std::u16string& func, uint64_t line) :fileName(file), funcName(func), lineNumber(line) {}

        inline std::u16string StackFrame::getFileName() const {
            return fileName;
        }

        inline std::u16string StackFrame::getFunctionName() const {
            return funcName;
        }

        inline uint64_t StackFrame::getLineNumber() const {
            return lineNumber;
        }
    }
}


#endif /* MATLAB_EXECUTION_EXCEPTION_IMPL_HPP */