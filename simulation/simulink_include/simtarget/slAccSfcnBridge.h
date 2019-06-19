/* Copyright 2008-2014 The MathWorks, Inc. */

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SLACCELSFCNBRIDGE_H
#define SLACCELSFCNBRIDGE_H

#include "tmwtypes.h"

#ifdef BUILDING_LIBMWSIMULINK
 
  #include "package.h"
  # define SIMULINK_EXPORT_EXTERN_C extern "C" DLL_EXPORT_SYM
#else
 
  #if defined(__cplusplus)
   
    #define SIMULINK_EXPORT_EXTERN_C extern "C"
  #else
    #define SIMULINK_EXPORT_EXTERN_C extern
  #endif
#endif

SIMULINK_EXPORT_EXTERN_C void slAccRegPrmChangeFcn(SimStruct* S,void (*fcnPtr)(SimStruct*, int));

SIMULINK_EXPORT_EXTERN_C void
slAccPostBlock(
    SimStruct* S,
    int sysIdx,
    int blkIdx,
    int enumFunction   
    );

#endif
