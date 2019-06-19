// Copyright 2007-2018 The MathWorks, Inc.

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SFUN_TYPE_C_API_HPP
#define SFUN_TYPE_C_API_HPP

#include "../libmwcgir_construct.hpp"
#include "../fwd.hpp"

#if defined(EXPORT_CGIR_CONSTRUCT_API) || defined(DLL_IMPORT_SYM)

#include "simstruct/simstruc.h"
#else

#include "simstruc.h"
#endif

namespace RTW {
extern "C" {

CGIR_CONSTRUCT_API SFun_Type_Impl* SFun_Type_ctor_DT(DTypeId dataType);
CGIR_CONSTRUCT_API SFun_Type_Impl* SFun_Type_cctor(const SFun_Type_Impl* pOther);

CGIR_CONSTRUCT_API int SFun_Type_op_assign(SFun_Type_Impl* pThis, const SFun_Type_Impl* pOther);

CGIR_CONSTRUCT_API void SFun_Type_dtor(SFun_Type_Impl* pThis);

CGIR_CONSTRUCT_API SFun_Type_Impl* SFun_Type_voidType();
CGIR_CONSTRUCT_API SFun_Type_Impl* SFun_complex_T(const SFun_Type_Impl* pBaseType);

CGIR_CONSTRUCT_API SFun_Type_Impl* SFun_pointerTo_T(const SFun_Type_Impl* pBaseType);

CGIR_CONSTRUCT_API SFun_Type_Impl* SFun_vectorOf_T_sz_DM(const SFun_Type_Impl* pElementType,
                                                         size_t size,
                                                         DimensionsMode_T dimsMode);

CGIR_CONSTRUCT_API SFun_Type_Impl* SFun_unsizedVectorOf_T_sz_DM(const SFun_Type_Impl* pElementType);

CGIR_CONSTRUCT_API SFun_Type_Impl* SFun_matrixOf_T_sz_sz_DM(const SFun_Type_Impl* pElementType,
                                                            size_t rows,
                                                            size_t columns,
                                                            DimensionsMode_T dimsMode);

CGIR_CONSTRUCT_API SFun_Type_Impl* SFun_matrixOf_T_DI_DM(const SFun_Type_Impl* pElementType,
                                                         const DimsInfo_T* pDimensions,
                                                         const DimensionsMode_T* dimsModes,
                                                         bool isColumnMajor);
}
}

#endif
