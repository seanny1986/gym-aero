/* Copyright 2013-2017 The MathWorks, Inc. */

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SL_SFCN_COV_BRIDGE_H
#define SL_SFCN_COV_BRIDGE_H

#include "tmwtypes.h"

#ifdef BUILDING_LIBMWSL_SFCN_COV_BRIDGE
  /* being included from the module source code */
  #include "package.h"
  #define LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C extern "C" DLL_EXPORT_SYM
#else
  /* being included from outside */
  #ifdef __cplusplus
    #define LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C extern "C"
  #else
    #define LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C extern
  #endif
#endif

#ifdef BUILDING_LIBMWSL_SFCN_COV_BRIDGE
  /* internal use */
  #include "simstruct/simstruc.h"
#else
  /* external use */
  #ifndef _SIMSTRUCT
    #define _SIMSTRUCT
      typedef struct SimStruct_tag SimStruct;
  #endif
#endif

typedef uint32_T covid_T;

LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C boolean_T slcovStartRecording(SimStruct* S);
LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C void slcovStopRecording(SimStruct* S);
LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C size_t slcovGetNumberOfSFunctionInstances(SimStruct* S);
LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C boolean_T slcovIsSFunctionInstanceSupported(SimStruct* S);
LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C boolean_T slcovEnterSFunctionMethod(SimStruct* S);
LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C boolean_T slcovExitSFunctionMethod(SimStruct* S);
LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C void slcovUploadSFunctionCoverageData(covid_T covId);
LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C void slcovUploadSFunctionCoverageSynthesis(SimStruct* S,
                                                        const uint32_T numH,
                                                        uint32_T* hTable,
                                                        const uint32_T numT,
                                                        uint32_T* tTable);
LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C void slcovClearSFunctionCoverageData(SimStruct* S,
                                                  const uint32_T numH,
                                                  uint32_T* hTable,
                                                  const uint32_T numT,
                                                  uint32_T* tTable);

LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C double slcovGetCovBoundaryAbsTol(void);
LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C double slcovGetCovBoundaryRelTol(void);

LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C boolean_T slcovIsInitialized(void);
LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C void slcovUploadStateflowCoverageSynthesisByName(const char* mainMachineName, 
                                                              const char* machineName,
                                                              const uint32_T numH,
                                                              uint32_T* hTable,
                                                              const uint32_T numT,
                                                              uint32_T* tTable);
LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C void slcovUploadStateflowCoverageSynthesisBySimstruct(SimStruct* S,
                                                                   const uint32_T numH,
                                                                   uint32_T* hTable,
                                                                   const uint32_T numT,
                                                                   uint32_T* tTable);

LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C void slcovUploadCoverageSynthesisById(const char* id);
LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C void slcovUploadCoverageSynthesisBySimstruct(SimStruct* S);
LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C void slcovUploadCoverageSynthesisByModel(const double modelH);

#include "matrix.h"

typedef struct slcovMxFunctionRef_tag slcovMxFunctionRef;

LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C boolean_T slcovLoadInstrumentedMexFunction(SimStruct* S, slcovMxFunctionRef **p);
LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C void slcovRunInstrumentedMexFunction(slcovMxFunctionRef *p,
                                                  int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
LIBMWSL_SFCN_COV_BRIDGE_EXTERN_C void slcovUnloadInstrumentedMexFunction(slcovMxFunctionRef **p);

#endif
