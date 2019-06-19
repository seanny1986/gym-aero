/* Copyright 2017 The MathWorks, Inc. */

#ifndef MATLAB_EXECUTION_EXCEPTION_HPP
#define MATLAB_EXECUTION_EXCEPTION_HPP

#include <MatlabDataArray/Exception.hpp>
#include <vector>
#include <streambuf>
#include <memory>
#include <future>
#include <exception>

#if defined(_WIN32 ) 
#define NOEXCEPT throw() 
#else
#define NOEXCEPT noexcept
#endif


namespace matlab {

    namespace execution {

        class Exception : public std::exception, public matlab::Exception {
        public:
            Exception();
            
            /**
            * Constructor that accepts an error message
            */
            Exception(const std::string& msg);

            virtual ~Exception();

            Exception& operator=(const Exception& rhs);

            /**
            * Returns the error message
            */
            virtual const char* what() const NOEXCEPT;
        private:
            std::string message;
        };

        class MATLABNotAvailableException final : public Exception {
        public:
            MATLABNotAvailableException();

            /**
            * Constructor that accepts an error message
            */
            MATLABNotAvailableException(const std::string& msg);
        };

        class CancelledException final: public Exception {
        public:
            CancelledException();

            /**
            * Constructor that accepts an error message
            */
            CancelledException(const std::string& msg);
        };

        class InterruptedException final: public Exception {
        public:
            InterruptedException();

            /**
            * Constructor that accepts an error message
            */
            InterruptedException(const std::string& msg);
        };

        class StackFrame final {
        public:
            StackFrame();
            
            /**
            * Constructor that accepts a file name, function name and line number
            */
            StackFrame(const std::u16string& file, const std::u16string& func, uint64_t line);

            /**
            * Get the file name
            */
            std::u16string getFileName() const;

            /**
            * Get the function name
            */
            std::u16string getFunctionName() const;

            /**
            * Get the line number
            */
            uint64_t getLineNumber() const;
        private:
            std::u16string fileName;
            std::u16string funcName;
            uint64_t lineNumber;
        };

        class MATLABException : public Exception {
        public:
            MATLABException();

            /**
            * Constructor that accepts an error message
            */
            MATLABException(const std::string& msg);

            /**
            * Constructor that accepts an error message ID and an error message
            */
            MATLABException(const std::string& id, const std::u16string& txt);

            /**
            * Get the message ID
            */
            std::string getMessageID() const;

            /**
            * Get the error message
            */
            std::u16string getMessageText() const;

            virtual ~MATLABException() {};

            /**
            * Copy constructor
            */
            MATLABException(const MATLABException& rhs);

            /**
            * Assignment operator
            */
            MATLABException& operator=(const MATLABException& rhs);
        private:
            std::string errorID;
            std::u16string errorText;
        };

        class MATLABSyntaxException final : public MATLABException {
        public:
            /**
            * Constructor that accepts an error message
            */
            MATLABSyntaxException(const std::string& msg);

            /**
            * Constructor that accepts an error message ID and an error message
            */
            MATLABSyntaxException(const std::string& id, const std::u16string& txt);
        };

        class MATLABExecutionException final : public MATLABException {
        public:
            MATLABExecutionException();

            /**
            * Constructor that accepts an error message ID and an error message
            */
            MATLABExecutionException(const std::string& msg, const std::vector<StackFrame>& trace);

            /**
            * Constructor that accepts an error message ID and an error message
            */
            MATLABExecutionException(const std::string& id, const std::u16string& txt, const std::vector<StackFrame>& trace, const std::vector<MATLABExecutionException>& cause);

            /**
            * Get the stack trace
            */
            std::vector<StackFrame> getStackTrace() const;

            /**
            * Get the cause of the MATLAB exception
            */
            std::vector<MATLABExecutionException> getCause() const;

            /**
            * Set the cause of the MATLAB exception
            */
            void setCause(const std::vector<MATLABExecutionException>& cause);
        private:
            std::vector<StackFrame> stackTrace;
            std::vector<MATLABExecutionException> errorCause;
        };

        class TypeConversionException final : public Exception {
        public:
            TypeConversionException();

            /**
            * Constructor that accepts an error message
            */
            TypeConversionException(const std::string& msg);
        };
    }
}

#endif /* MATLAB_EXECUTION_EXCEPTION_HPP */