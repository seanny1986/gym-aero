/* Copyright 2016-2018 The MathWorks, Inc. */

#ifndef MATLAB_EXTDATA_EXCEPTION_HELPER_FUNCTIONS_HPP
#define MATLAB_EXTDATA_EXCEPTION_HELPER_FUNCTIONS_HPP

#include "publish_util.hpp"
#include "ExceptionType.hpp"
#include "FunctionType.hpp"

#include <algorithm>

MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C void* get_function_ptr(int fcn);

namespace matlab {
namespace data {
namespace detail {
inline void throwIfError(int type) {
    switch (ExceptionType(type)) {
    case ExceptionType::NoException:
        break;
    case ExceptionType::InvalidFieldName:
        throw InvalidFieldNameException("Field does not exist in this struct");
    case ExceptionType::InvalidDataType:
        throw TypeMismatchException("Data type mismatch");
    case ExceptionType::InvalidArrayType:
        throw InvalidArrayTypeException("Array type mismatch");
    case ExceptionType::InvalidMemoryLayout:
        throw InvalidMemoryLayoutException("Memory layout mismatch");
    case ExceptionType::InvalidDimensionsInRowMajorArray:
        throw InvalidDimensionsInRowMajorArrayException("Memory layout mismatch");
    case ExceptionType::InvalidTypeInRowMajorArray:
        throw InvalidTypeInRowMajorArrayException("Memory layout mismatch");
    case ExceptionType::MustSpecifyClassName:
        throw MustSpecifyClassNameException("Must specify a class name");
    case ExceptionType::WrongNumberOfEnumsSupplied:
        throw WrongNumberOfEnumsSuppliedException("Wrong number of enums supplied");
    case ExceptionType::OutOfMemory:
        throw matlab::OutOfMemoryException("Not enough memory available to support the request");
    case ExceptionType::TooManyIndicesProvided:
        throw TooManyIndicesProvidedException("Too many indices provided");
    case ExceptionType::NotEnoughIndicesProvided:
        throw NotEnoughIndicesProvidedException("Not enough indices provided");
    case ExceptionType::StringIndexMustBeLast:
        throw StringIndexMustBeLastException("String index must be last");
    case ExceptionType::StringIndexNotValid:
        throw StringIndexNotValidException("String index not valid for this array");
    case ExceptionType::CanOnlyUseOneStringIndex:
        throw CanOnlyUseOneStringIndexException("Can only provide one string index");
    case ExceptionType::InvalidArrayIndex:
        throw InvalidArrayIndexException("Array index is invalid");
    case ExceptionType::DuplicateFieldNameInStructArray:
        throw DuplicateFieldNameInStructArrayException("Duplicate field names not allowed");
    case ExceptionType::NonAsciiCharInRequestedAsciiOutput:
        throw NonAsciiCharInRequestedAsciiOutputException(
            "Can't create a std::string from this data");
    case ExceptionType::CantIndexIntoEmptyArray:
        throw CantIndexIntoEmptyArrayException("Can't index into an empty array");
    case ExceptionType::NumberOfElementsExceedsMaximum:
        throw NumberOfElementsExceedsMaximumException("Number of elements exceeds max");        
    default:
        throw SystemErrorException("Error");
    }
}

template <class T>
T resolveFunctionNoExcept(FunctionType fcn) {
    void* ptr = get_function_ptr(static_cast<int>(fcn));
    return (T)ptr;
}

template <class T>
T resolveFunction(FunctionType fcn) {
    void* ptr = get_function_ptr(static_cast<int>(fcn));
    if (ptr == nullptr) {
        static const std::vector<std::pair<FunctionType, std::string>> minRelease = {
            {FunctionType::CREATE_ENUM_ARRAY, "R2018a"},
            {FunctionType::SPARSE_ARRAY_GET_INDEX_ARITHMETIC, "R2018b"},
            {FunctionType::CREATE_ARRAY_FROM_BUFFER_V2, "R2019a"},
            {FunctionType::PLACEHOLDER, "FUTURE"}};
        auto it = std::find_if(
            minRelease.rbegin(), minRelease.rend(),
            [fcn](std::pair<FunctionType, std::string> elem) { return fcn >= elem.first; });
        throw FeatureNotSupportedException(std::string("This feature requires ") +
                                           std::string(it->second.c_str()));
    }
    return (T)ptr;
}
} // namespace detail
} // namespace data
} // namespace matlab

#endif
