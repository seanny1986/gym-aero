/* Copyright 2016-2018 The MathWorks, Inc. */

#ifndef MATLAB_EXCEPTION_TYPE_HPP_
#define MATLAB_EXCEPTION_TYPE_HPP_


namespace matlab {
namespace data {
enum class ExceptionType : int {
    NoException = 0,
    InvalidArrayIndex,
    StringIndexMustBeLast,
    StringIndexNotValid,
    CantAssignArrayToThisArray,
    NotEnoughIndicesProvided,
    TooManyIndicesProvided,
    CanOnlyUseOneStringIndex,
    InvalidDataType,
    InvalidArrayType,
    InvalidFieldName,
    FailedToLoadLibMatlabDataArray,
    FailedToResolveSymbol,
    MustSpecifyClassName,
    WrongNumberOfEnumsSupplied,
    NonAsciiCharInInputData,
    NonAsciiCharInRequestedAsciiOutput,
    InvalidDimensionsInSparseArray,
    DuplicateFieldNameInStructArray,
    OutOfMemory,
    SystemError,
    CantIndexIntoEmptyArray,
    FeatureNotSupported,
    InvalidMemoryLayout,
    InvalidDimensionsInRowMajorArray,
    InvalidTypeInRowMajorArray,
    NumberOfElementsExceedsMaximum
};
}
} // namespace matlab

#endif
