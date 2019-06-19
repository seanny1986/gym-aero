// Copyright 2007-2018 The MathWorks, Inc.

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SFUN_TYPE_HPP
#define SFUN_TYPE_HPP

#include "fwd.hpp"
#include "impl/type_c_api.hpp"
#include "impl/rtw_api_lint_begin.hpp"

namespace RTW {

struct EnumConstant {
    const char* name;
    long long value;
};

class Type {
  public:
   
    explicit Type(DTypeId dataType);
   
    Type(const Type& other);

    Type& operator=(const Type& other);

    ~Type();

    static Type voidType();
   
    explicit Type(SFun_Type_Impl* pImpl);
    SFun_Type_Impl* getImpl();
    const SFun_Type_Impl* getImpl() const;

  private:
    SFun_Type_Impl* fpImpl;
};

Type complex(const Type& baseType);

Type pointerTo(const Type& baseType);

Type vectorOf(const Type& elementType, size_t size, DimensionsMode_T dimsMode = FIXED_DIMS_MODE);

Type unsizedVectorOf(const Type& elementType);

Type matrixOf(const Type& elementType,
              size_t rows,
              size_t columns,
              DimensionsMode_T dimsMode = FIXED_DIMS_MODE);

Type matrixOf(const Type& elementType,
              const DimsInfo_T& dimensions,
              const DimensionsMode_T* dimsModes = NULL,
              bool isColumnMajor = true);

}

#include "impl/rtw_api_lint_end.hpp"

#include "impl/api_util.hpp"
#include "value.hpp"
#include "impl/rtw_api_lint_begin.hpp"

namespace RTW {

inline Type::Type(SFun_Type_Impl* pImpl)
    : fpImpl(pImpl) {
    verify(fpImpl);
}

inline SFun_Type_Impl* Type::getImpl() {
    return fpImpl;
}

inline const SFun_Type_Impl* Type::getImpl() const {
    return fpImpl;
}

inline Type::Type(DTypeId dataType)
    : fpImpl(SFun_Type_ctor_DT(dataType)) {
    verify(fpImpl);
}
inline Type::Type(const Type& other)
    : fpImpl(SFun_Type_cctor(other.fpImpl)) {
    verify(fpImpl);
}

inline Type& Type::operator=(const Type& other) {
    verify(SFun_Type_op_assign(fpImpl, other.fpImpl));
    return *this;
}

inline Type::~Type() {
    SFun_Type_dtor(fpImpl);
}

inline Type Type::voidType() {
    SFun_Type_Impl* pResult = SFun_Type_voidType();
    return Type(pResult);
}
inline Type complex(const Type& baseType) {
    SFun_Type_Impl* pResult = SFun_complex_T(baseType.getImpl());
    return Type(pResult);
}

inline Type pointerTo(const Type& baseType) {
    SFun_Type_Impl* pResult = SFun_pointerTo_T(baseType.getImpl());
    return Type(pResult);
}

inline Type vectorOf(const Type& elementType, size_t size, DimensionsMode_T dimsMode) {
    SFun_Type_Impl* pResult = SFun_vectorOf_T_sz_DM(elementType.getImpl(), size, dimsMode);
    return Type(pResult);
}

inline Type unsizedVectorOf(const Type& elementType) {
    SFun_Type_Impl* pResult = SFun_unsizedVectorOf_T_sz_DM(elementType.getImpl());
    return Type(pResult);
}

inline Type matrixOf(const Type& elementType,
                     size_t rows,
                     size_t columns,
                     DimensionsMode_T dimsMode) {
    SFun_Type_Impl* pResult =
        SFun_matrixOf_T_sz_sz_DM(elementType.getImpl(), rows, columns, dimsMode);
    return Type(pResult);
}

inline Type matrixOf(const Type& elementType,
                     const DimsInfo_T& dimensions,
                     const DimensionsMode_T* dimsModes,
                     bool isColumnMajor) {
    SFun_Type_Impl* pResult =
        SFun_matrixOf_T_DI_DM(elementType.getImpl(), &dimensions, dimsModes, isColumnMajor);
    return Type(pResult);
}
}

#include "impl/rtw_api_lint_end.hpp"

#endif
