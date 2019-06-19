#ifndef __MEX_API_ERROR_DISPATCH_HPP__
#define __MEX_API_ERROR_DISPATCH_HPP__

#include "mexExceptionType.hpp"


namespace matlab {
    namespace engine {
        namespace detail {

            inline matlab::engine::MATLABExecutionException createMATLABExecutionException(const matlab::data::StructArray& mException);

            inline std::vector<matlab::engine::MATLABExecutionException> createCause(const matlab::data::CellArray& cause) {
                size_t nCauses = cause.getNumberOfElements();
                std::vector<matlab::engine::MATLABExecutionException> causes(nCauses);
                for (size_t i = 0; i < nCauses; i++) {
                    matlab::data::Array exRef = cause[i];
                    matlab::data::StructArray ex(exRef);
                    causes[i] = createMATLABExecutionException(ex);
                }
                return causes;
            }

            inline std::vector<matlab::engine::StackFrame> createStackTrace(const matlab::data::StructArray& stack) {
                size_t nFrames = stack.getNumberOfElements();
                std::vector<matlab::engine::StackFrame> stackFrames(nFrames);

                for (size_t i = 0; i < nFrames; i++) {
                    matlab::data::Array fileRef = stack[i]["File"];
                    matlab::data::CharArray fileStr(fileRef);

                    matlab::data::Array nameRef = stack[i]["Name"];
                    matlab::data::CharArray nameStr(nameRef);

                    matlab::data::Array lineRef = stack[i]["Line"];
                    double line = lineRef[0];
                    stackFrames[i] = matlab::engine::StackFrame(fileStr.toUTF16(), nameStr.toUTF16(), uint32_t(line));
                }

                return stackFrames;
            }

            inline matlab::engine::MATLABExecutionException createMATLABExecutionException(const matlab::data::StructArray& mException) {
                matlab::data::Array idRef = mException[0][std::string("identifier")];
                matlab::data::CharArray id(idRef);
                matlab::data::Array messageRef = mException[0][std::string("message")];
                matlab::data::CharArray message(messageRef);

                matlab::data::Array stackRef = mException[0][std::string("stack")];
                matlab::data::StructArray stack(stackRef);

                matlab::data::Array causeRef = mException[0][std::string("cause")];
                matlab::data::CellArray cause(causeRef);

                std::vector<matlab::engine::MATLABExecutionException> meCause = createCause(cause);
                std::vector<matlab::engine::StackFrame> meStack = createStackTrace(stack);

                return matlab::engine::MATLABExecutionException(id.toAscii(), message.toUTF16(), meStack, meCause);
            }

            inline matlab::engine::MATLABSyntaxException createMATLABSyntaxException(const matlab::data::StructArray& mException) {
                matlab::data::Array idRef = mException[0][std::string("identifier")];
                matlab::data::CharArray id(idRef);
                matlab::data::Array messageRef = mException[0][std::string("message")];
                matlab::data::CharArray message(messageRef);

                return matlab::engine::MATLABSyntaxException(id.toAscii(), message.toUTF16());
            }

            inline matlab::engine::MATLABException createMATLABException(const matlab::data::StructArray& mException) {
                matlab::data::Array idRef = mException[0][std::string("identifier")];
                matlab::data::CharArray id(idRef);
                matlab::data::Array messageRef = mException[0][std::string("message")];
                matlab::data::CharArray message(messageRef);

                return matlab::engine::MATLABException(id.toAscii(), message.toUTF16());
            }
        }
    }
}


void throwIfError(int errID, void* mexcept) {
    matlab::mex::detail::ErrorType errorID(static_cast<matlab::mex::detail::ErrorType>(errID));
    switch (errorID) {
    case matlab::mex::detail::ErrorType::NoException:
        break;
    case matlab::mex::detail::ErrorType::RuntimeError: {
        matlab::data::impl::ArrayImpl* impl = reinterpret_cast<matlab::data::impl::ArrayImpl*>(mexcept);
        matlab::data::Array exArr(matlab::data::detail::Access::createObj<matlab::data::Array>(impl));
        matlab::data::StructArray sArr(exArr);
        matlab::data::TypedArray<int> errStatus = sArr[0][std::string("status")];
        int rErrID_ = errStatus[0];
        matlab::mex::detail::ErrorType rErrorID(static_cast<matlab::mex::detail::ErrorType>(rErrID_));
        switch (rErrorID) {
        case matlab::mex::detail::ErrorType::SyntaxError: {
            matlab::engine::MATLABSyntaxException exp = matlab::engine::detail::createMATLABSyntaxException(sArr);
            throw exp;
        }
        case matlab::mex::detail::ErrorType::ExecutionError: {
            matlab::engine::MATLABExecutionException exp = matlab::engine::detail::createMATLABExecutionException(sArr);
            throw exp;
        }
        case matlab::mex::detail::ErrorType::EngineError: {
            matlab::engine::MATLABException exp = matlab::engine::detail::createMATLABException(sArr);
            throw exp;
        }
        case matlab::mex::detail::ErrorType::InterruptedError: {
            std::string msg = "MATLAB command was interrupted.";
            throw matlab::engine::InterruptedException("MATLAB:mex:CppMexException", matlab::engine::convertUTF8StringToUTF16String(msg));
        }
        default:
            throw matlab::engine::MATLABException("MATLAB:mex:CppMexException", matlab::engine::convertUTF8StringToUTF16String("Runtime Error"));
        }
    }
    case matlab::mex::detail::ErrorType::ThreadError: {
        const std::string msg = "Synchronous version of MATLAB Engine functions must be called from the thread that called the MEX function.";
        throw matlab::engine::MATLABException("MATLAB:mex:CppMexThreadMismatch", matlab::engine::convertUTF8StringToUTF16String(msg));
    }
    case matlab::mex::detail::ErrorType::OutOfMemory: {
        std::string outOfMemoryError = "Not enough memory available to support the request.";
        throw matlab::OutOfMemoryException(outOfMemoryError);
    }
    case matlab::mex::detail::ErrorType::CancelError: {
        std::string msg = "MATLAB command was cancelled.";
        throw matlab::engine::CancelledException("MATLAB:mex:CppMexException", matlab::engine::convertUTF8StringToUTF16String(msg));
    }
    case matlab::mex::detail::ErrorType::SystemError: {
        std::string msg = "Unexpected exception caught in feval.";
        throw matlab::engine::MATLABException("MATLAB:mex:CppMexException", matlab::engine::convertUTF8StringToUTF16String(msg));
    }
    default:
        throw matlab::engine::MATLABException("MATLAB:mex:CppMexException", matlab::engine::convertUTF8StringToUTF16String("Error"));
    }
}

#endif