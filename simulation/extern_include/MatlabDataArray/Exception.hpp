/* Copyright 2015-2018 The MathWorks, Inc. */

#ifndef MATLAB_EXCEPTION_HPP_
#define MATLAB_EXCEPTION_HPP_

#include "detail/ExceptionType.hpp"
#include "detail/exception_interface.hpp"

#include <string>
#include <exception>

namespace matlab {

/**
 * This is the base class for all External Data Interface exceptions.
 *
 * Concrete exception classes can inherit from std::exception or its subclasses (i.e.
 * std::bad_malloc) ONLY in cases when they are leaf classes.
 *
 */
class Exception {
  public:
    virtual ~Exception() MW_NOEXCEPT {
    }
    virtual const char* what() const MW_NOEXCEPT = 0;
};

using OutOfMemoryException =
    data::detail::ArrayException<matlab::Exception, data::ExceptionType::OutOfMemory>;

namespace data {
using InvalidArrayIndexException =
    detail::ArrayException<matlab::Exception, ExceptionType::InvalidArrayIndex>;
using TypeMismatchException =
    detail::ArrayException<matlab::Exception, ExceptionType::InvalidDataType>;
using InvalidArrayTypeException =
    detail::ArrayException<matlab::Exception, ExceptionType::InvalidArrayType>;
using InvalidMemoryLayoutException =
    detail::ArrayException<matlab::Exception, ExceptionType::InvalidMemoryLayout>;
using InvalidDimensionsInRowMajorArrayException =
    detail::ArrayException<matlab::Exception, ExceptionType::InvalidDimensionsInRowMajorArray>;
using InvalidTypeInRowMajorArrayException =
    detail::ArrayException<matlab::Exception, ExceptionType::InvalidTypeInRowMajorArray>;
using SystemErrorException = detail::ArrayException<matlab::Exception, ExceptionType::SystemError>;
using InvalidFieldNameException =
    detail::ArrayException<matlab::Exception, ExceptionType::InvalidFieldName>;
using FailedToLoadLibMatlabDataArrayException =
    detail::ArrayException<matlab::Exception, ExceptionType::FailedToLoadLibMatlabDataArray>;
using FailedToResolveSymbolException =
    detail::ArrayException<matlab::Exception, ExceptionType::FailedToResolveSymbol>;
using WrongNumberOfEnumsSuppliedException =
    detail::ArrayException<matlab::Exception, ExceptionType::WrongNumberOfEnumsSupplied>;
using MustSpecifyClassNameException =
    detail::ArrayException<matlab::Exception, ExceptionType::MustSpecifyClassName>;
using TooManyIndicesProvidedException =
    detail::ArrayException<matlab::Exception, ExceptionType::TooManyIndicesProvided>;
using NotEnoughIndicesProvidedException =
    detail::ArrayException<matlab::Exception, ExceptionType::NotEnoughIndicesProvided>;
using StringIndexMustBeLastException =
    detail::ArrayException<matlab::Exception, ExceptionType::StringIndexMustBeLast>;
using StringIndexNotValidException =
    detail::ArrayException<matlab::Exception, ExceptionType::StringIndexNotValid>;
using CanOnlyUseOneStringIndexException =
    detail::ArrayException<matlab::Exception, ExceptionType::CanOnlyUseOneStringIndex>;
using NonAsciiCharInInputDataException =
    detail::ArrayException<matlab::Exception, ExceptionType::NonAsciiCharInInputData>;
using NonAsciiCharInRequestedAsciiOutputException =
    detail::ArrayException<matlab::Exception, ExceptionType::NonAsciiCharInRequestedAsciiOutput>;
using InvalidDimensionsInSparseArrayException =
    detail::ArrayException<matlab::Exception, ExceptionType::InvalidDimensionsInSparseArray>;
using DuplicateFieldNameInStructArrayException =
    detail::ArrayException<matlab::Exception, ExceptionType::DuplicateFieldNameInStructArray>;
using CantIndexIntoEmptyArrayException =
    detail::ArrayException<matlab::Exception, ExceptionType::CantIndexIntoEmptyArray>;
using FeatureNotSupportedException =
    detail::ArrayException<matlab::Exception, ExceptionType::FeatureNotSupported>;
using NumberOfElementsExceedsMaximumException =
    detail::ArrayException<matlab::Exception, ExceptionType::NumberOfElementsExceedsMaximum>;
} // namespace data
} // namespace matlab


#endif
