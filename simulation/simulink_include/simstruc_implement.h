/* Copyright 1990-2018 The MathWorks, Inc. */

/**
 * @file: simstruc_implement.h
 *  
 * @brief Required ssAPIs for simstruc.h and s-function demo examples, but not documented.
 *
 */

#ifndef __SIMSTRUC__IMPLEMENT
#define __SIMSTRUC__IMPLEMENT

#define _ssSafelyCallGenericFcnStart(S) \
       (((S)->mdlInfo->genericFcn==NULL) ? 0 : ((*(S)->mdlInfo->genericFcn)
#define _ssSafelyCallGenericFcnEnd ))


#if !SS_SFCN_LEVEL_1

# define _ssGetNumInputPorts(S) \
      ((ssGetSfcnUsesNumPorts(S) == 1)? \
       ((S)->sizes.in.numInputPorts) : 0)                          /*(int_T) */

# define _ssSetNumInputPorts(S,nInputPorts) \
          (S)->sizes.in.numInputPorts = (nInputPorts)

# define _ssGetNumOutputPorts(S) \
      ((ssGetSfcnUsesNumPorts(S) == 1)? \
       ((S)->sizes.out.numOutputPorts) : 0)                         /*(int_T) */


# define _ssSetNumOutputPorts(S,nOutputPorts) \
          (S)->sizes.out.numOutputPorts = (nOutputPorts)

#endif


#define _ssSetCurrentOutputPortDimensions(S, pIdx, dIdx, val) \
    (S)->blkInfo.blkInfo2->portInfo2->outputs[(pIdx)].portVarDims[(dIdx)] = (val)

struct _ssVarDimsIdxVal_tag {
    int dIdx;
    int dVal;
};

#define _ssGetSFcnParam(S,index) \
          (S)->sfcnParams.dlgParams[index]                /*   (mxArray *)   */

#define _ssSetNumRunTimeParams(S, num) \
         (S)->sfcnParams.numRtp.numRtp = (num)


#define _ssSetRunTimeParamInfo(S, idx, p) (S)->sfcnParams.rtp[idx] = (p)

/* DWork - This is the number of the data type work vector of your S-function.
 *   The root SimStruct contains the total number of data type work vectors
 *   used within the model.
 */
#define _ssSetNumDWork(S,nDWork) \
          ((S)->sizes.numDWork = (nDWork))

#define ssIsTIDInStInfo(S,tid) \
        (((tid) >= 0) && \
         ((tid) < (ssGetParentSS(S) ? \
                   ssGetNumSampleTimes(ssGetParentSS(S)) : \
                   ssGetNumSampleTimes(S))) && \
         (ssGetSampleTimeTaskID(S,tid) < ssGetNumRootSampleTimes(S)))

#define ssGetSampleHitPtr(S) \
          (S)->mdlInfo->sampleHits                        /*   (int_T *) */


#if SS_SL_INTERNAL
#  define ssGetSampleTimeTaskID(S,sti) \
            (S)->stInfo.sampleTimeTaskIDs[sti]         /*   (int_T)       */
#elif TID_EQUALS_SAMPLE_TIME_INDEX
#  define ssGetSampleTimeTaskID(S,sti) (sti)           /*   (int_T)       */
#else
#    define ssGetSampleTimeTaskID(S, sti) \
            (S)->stInfo.sampleTimeTaskIDs[sti]         /*   (int_T)       */
#endif


#  define ssGetRegOutputPortDimensionInfoFcn(S) \
             ((S)->mdlInfo->regOutputPortDimsInfo)


 /* InputPortFrameData -  For each input port or your S-function block, this
  *   is whether or not the incoming signal is frame data, where (-1=either,
  *   0=no, 1=yes).
  */

# define ssSetInputPortFrameData(S,port,val) \
          (S)->portInfo.inputs[(port)].attributes.frameData = \
          CONV_INT2BITS(val)    

 /* OutputPortFrameData - For each input port or your S-function block, this
  *   is whether or not the outgoing signal is frame data, where (-1=either,
  *   0=no, 1=yes).
  */

# define ssSetOutputPortFrameData(S,port,val) \
   (S)->portInfo.outputs[(port)].attributes.frameData = \
            CONV_INT2BITS(val)



#if SS_SFCN_FOR_SIM

#ifdef __cplusplus
extern "C" {
#endif
    
extern int_T ssWriteRTWNameValuePair(     SimStruct    *S,
                                            int_T        type,
                                            const char_T *name,
                                            const void   *value, ...);

extern int_T _ssSetInputPortMatrixDimensions(SimStruct *S, int_T port,
                                           int_T m, int_T n);
extern int_T _ssSetOutputPortMatrixDimensions(SimStruct *S, int_T port,
                                            int_T m, int_T n);
extern int_T _ssSetInputPortVectorDimension(SimStruct *S, int_T port, int_T m);
extern int_T _ssSetOutputPortVectorDimension(SimStruct *S, int_T port, int_T m);
extern int_T ssIsRunTimeParamTunable(SimStruct *S, const int_T rtPIdx);
extern double ssGetSFuncBlockHandle(SimStruct *S);
extern int_T _ssGetCurrentInputPortWidth( SimStruct *S, int_T pIdx);
extern int_T _ssGetCurrentOutputPortWidth(SimStruct *S, int_T pIdx);
extern int_T _ssGetCallSystemNumFcnCallDestinations(SimStruct *S, int_T elemIdx);
extern boolean_T ssGetParameterTuningCompliance(SimStruct *S);
#ifdef __cplusplus
}
#endif

#endif




typedef struct _ssRTPRegInfoWithType_tag {
    int dlgIdx;
    int rtIdx;
    const char* name;
    int dtId;
} ssRTPRegInfoWithType;


#if !SS_SFCN_LEVEL_1

# if SS_SL_INTERNAL || SS_SFCN_FOR_SIM
#  define ssGetRegInputPortDimensionInfoFcn(S) \
             ((S)->mdlInfo->regInputPortDimsInfo)
#endif

#endif


/*
 * ssSetCallSystemOutput -
 *   S-functions should use this in mdlInitializeSampleTimes to specify
 *   which output port elements of the first output port are connected
 *   to function-call subsystems. The S-function can then execute
 *   the function-call subsystems via ssCallSystemWithTID
 */
#define ssGetCallSystemOutputPtr(S) \
          (S)->callSys.outputs                            /* (int_T*)    */
#define _ssSetCallSystemOutputPtr(S,ptr) \
          (S)->callSys.outputs = (ptr)
#if !SS_SFCN
#define ssSetCallSystemOutputPtr(S,ptr) _ssSetCallSystemOutputPtr(S,ptr)
#else
#define ssSetCallSystemOutputPtr(S,ptr) \
          ssSetCallSystemOutputPtr_cannot_be_used_in_SFunctions
#endif

#define ssGetCallSystemOutput(S,element) \
          (S)->callSys.outputs[element]                   /* (int_T)     */





/* ================== APIs for zero crossing =================================*/

typedef struct {
    int_T        _regionIdx;
    int_T        _regionElIdx;
    boolean_T    _result;
}_ssRegionElementIdxInfo;

#ifdef __cplusplus
extern "C" {
#endif

extern boolean_T ssGetIsZeroCrossControlDisabled(SimStruct *S);
extern void ssSetIsInputPortUsedForContZcSignal(SimStruct *S, int_T pIdx, boolean_T value);
extern void ssSetZcSignalIsZcElementDisc(SimStruct *S, int_T zcsIdx, int_T zcsElIdx, boolean_T value);
extern void ssSetZcSignalName(SimStruct *S, int_T zcsIdx, char* name);
extern void ssSetZcSignalWidth(SimStruct *S, int_T zcsIdx, int_T width);
extern void ssSetZcSignalZcEventType(SimStruct *S, int_T zcsIdx, slZcEventType zcEventType);
extern void ssSetZcSignalType(SimStruct *S, int_T zcsIdx, slZcSignalType zcSignalType);
extern void ssSetZcSignalZcTol(SimStruct *S, int_T zcsIdx,  double zcTol);
extern void ssSetZcSignalNeedsEventNotification(SimStruct *S, int_T zcsIdx, boolean_T needsEventNotification);

extern         int_T  ssCreateAndAddZcSignalInfo(SimStruct *S);
extern         int_T  ssCloneAndAddZcSignalInfo(SimStruct *S, int_T zcsIdx);
extern         int_T  ssGetNumZcSignals(SimStruct *S);

extern      double*   ssGetZcSignalVector(SimStruct *S, int_T zcsIdx);
extern const char*    ssGetZcSignalName(SimStruct *S, int_T zcsIdx);
extern       int_T    ssGetZcSignalWidth(SimStruct *S, int_T zcsIdx);
extern slZcEventType  ssGetZcSignalZcEventType(SimStruct *S, int_T zcsIdx);
extern slZcEventType* ssGetZcSignalZcEvents(SimStruct *S, int_T zcsIdx);
extern slZcSignalType ssGetZcSignalType(SimStruct *S, int_T zcsIdx);
extern double         ssGetZcSignalZcTol(SimStruct *S, int_T zcsIdx);
extern boolean_T      ssGetZcSignalNeedsEventNotification(SimStruct *S,

                                                          int_T zcsIdx);
extern boolean_T      ssGetZcSignalIsZcElementDisc(SimStruct *S,
                                                   int_T zcsIdx,
                                                   int_T zcsElIdx);

extern boolean_T      ssGetIsInputPortElementContinuous(SimStruct *S,
                                                        int_T pIdx,
                                                        int_T eIdx);
#ifdef __cplusplus
}
#endif




#define ssGetNumRootSampleTimes(S) ssGetNumSampleTimes(ssGetRootSS(S))



# if SS_SIM
#define dtaGetDataTypeSize(dta, blockPath, id) \
   ((dta) != NULL ? \
   (dta)->getGenericDTAIntProp((dta)->dataTypeTable, (blockPath), (id), \
     GEN_DTA_INT_PROP_SIZE) : \
   INVALID_DTYPE_SIZE)
# else
#  define dtaGetDataTypeSize(dta, blockPath, id) \
    dtaGetDataTypeSize_cannot_be_used_in_RTW
# endif









/*
 * The following macro, ssSetTimeOfNextVarHit, will not work in non-inlined
 * S-functions in rapid accelerator or rsim. When using this advanced
 * and undocumented feature, TLC will be required in order to support
 * Rapid Accelerator simulation mode or Simulink Coder code generation
 * (rsim target).
 */

#define _ssSetVarNextHitTime(S,i,value) \
          (S)->mdlInfo->varNextHitTimesList[(i)] = (value)
#define _ssGetVarNextHitTime(S,i) \
         ((real_T)((S)->mdlInfo->varNextHitTimesList[(i)]))


    
#if SS_SL_INTERNAL || SS_SFCN_FOR_SIM
# if defined(RTW_GENERATED_S_FUNCTION)
#  define ssSetTimeOfNextVarHit(S, sti, tNext)                          \
    {                                                                   \
        _ssSetVarNextHitTime(                                           \
            ssGetRootSS(S),                                             \
            (int)(ssGetOffsetTime(ssGetRootSS(S), sti)),                \
            tNext);                                                     \
        ssSetTNext(S, tNext);                                           \
    }

#  define ssGetTimeOfNextVarHit(S,sti)                                  \
    _ssGetVarNextHitTime(                                               \
        ssGetRootSS(S),                                                 \
        (int)(ssGetOffsetTime( ssGetRootSS(S), sti)))

# else
#  define ssSetTimeOfNextVarHit(S,sti,tNext)                            \
    {                                                                   \
        _ssSetVarNextHitTime(                                           \
            ssGetRootSS(S),                                             \
            (int)(ssGetOffsetTime( ssGetRootSS(S),                      \
                                   ssGetSampleTimeTaskID(S, sti))),     \
            tNext);                                                     \
        ssSetTNext(S, tNext);                                           \
    }

#  define ssGetTimeOfNextVarHit(S,sti)                                  \
    _ssGetVarNextHitTime(                                               \
        ssGetRootSS(S),                                                 \
        (int)(ssGetOffsetTime( ssGetRootSS(S),                          \
                               ssGetSampleTimeTaskID(S, sti))))

# endif
#endif


#if SS_MULTITASKING || SS_SL_INTERNAL || SS_SFCN_FOR_SIM
# define ssGetPerTaskSampleHitsPtr(S) \
          (S)->mdlInfo->perTaskSampleHits                 /*   (int_T *) */
#endif


#if !SS_SFCN_LEVEL_1
# if SS_SIM
 
typedef struct _ssRegisterTypeFromNameType_tag {
      const char *name;
      int        *dataTypeId;
} ssRegisterTypeFromNameType;

#endif

#endif


#ifdef __cplusplus
extern "C" {
#endif
    extern real_T* _ssGetAbsTol(SimStruct *S);
    extern void _ssSetStateAbsTol(SimStruct *S, 
                                  const int_T idx, 
                                  const real_T val);

#ifdef __cplusplus
}
#endif



#ifndef _DIMSDEPENDINFO_T
#define _DIMSDEPENDINFO_T

/* Set the current dimension of output port outIdx based on the dims dependent rule ruleIdx */
typedef void (*SetOutputDimsFcn)(SimStruct *S, int outIdx, int *inps, int numInps);
/*
 * RTWCG function of setting the current dimension of output port outIdx based
 * on the dims dependent rule ruleIdx
 */
typedef void (*SetOutputDimsRTWCGFcn) (SimStruct *S, void *rtwBlk, int oIdx, int*inps, int nInputs);

typedef struct {
    int              *inputs;           /* index to inputs whose dims affect the output dims*/
    int              numInputs;         /* number of inputs affect the output dims */
    SetOutputDimsFcn setOutputDimsFcn;  /* function to update the output dims
                                           based on the dims of inputs*/
    SetOutputDimsRTWCGFcn setOutputDimsRTWCGFcn;
                                        /* RTWCG function to update output dims
                                           based on the dims of inputs */
} DimsDependInfo_T;

#endif



typedef void (*mdlSetInputPortDimensionsModeFcn)(SimStruct        *S,
                                                 int_T            portIdx,
                                                 DimensionsMode_T dimsMode);



/* ssSetSFcnParamTunable - This should be used by S-function blocks
 *   to specify when a parameter is tunable or not tunable.
 */
#if SS_SL_INTERNAL || SS_SFCN_FOR_SIM
# define ssSetSFcnParamNotTunable(S,index) \
          (S)->sfcnParams.dlgAttribs[index] = \
            ((S)->sfcnParams.dlgAttribs[index] | SFCNPARAM_NOT_TUNABLE)
#else /* RT or NRT */
# define ssSetSFcnParamNotTunable(S,index) /* do nothing */
#endif


# define ssSetInputPortSignalWhenNeeded(S,port,val) \
          (S)->portInfo.inputs[(port)].attributes.cinId = (val)



#if SS_SIM
  # define ssComputeInput(S, pIdx) \
     _ssSafelyCallGenericFcnStart(S)(S,GEN_FCN_COMPUTE_INPUT,pIdx,NULL)\
     _ssSafelyCallGenericFcnEnd
#else
  # define ssComputeInput(S, pIdx) \
     ssComputeInput_cannot_be_used_in_RTW
#endif



# if SS_SIM
#  define dtaSetErrorString(dta, string) \
    ((dta) != NULL ? \
    (dta)->errorString = (string) : \
    0)
# else
#  define dtaSetErrorString(dta, string) \
    dtaSetErrorString_cannot_be_used_in_RTW
# endif


# if SS_SIM
#define dtaSetConvertBetweenFcn(dta, blockPath, id, fcn) \
  ((dta) != NULL ? \
  (dta)->setConvertBetweenFcn((dta)->dataTypeTable, (blockPath), (id), (fcn)) : \
  0)
# else
#  define dtaSetConvertBetweenFcn(dta, blockPath, id, fcn) \
   dtaSetConvertBetweenFcn_cannot_be_used_in_RTW
# endif



# if SS_SIM
#define dtaSetIsPositiveFcn(dta, blockPath, id, fcn) \
  ((dta) != NULL ? \
  (dta)->setGenericDTAUnaryFcnGW((dta)->dataTypeTable, (blockPath), (id), (fcn), \
  GEN_DTA_UNARY_FCN_IS_POSITIVE) : \
  0)
# else
#  define dtaSetIsPositiveFcn(dta, blockPath, id, fcn) \
    dtaSetIsPositiveFcn_cannot_be_used_in_RTW
# endif





#if SS_SL_INTERNAL || SS_SFCN_FOR_SIM
#ifdef ssGetOwnerBlock
#define ssSetInputPortBusMode(S, pIdx, value) \
{ \
    if (ssGetOwnerBlock(S) != NULL) { \
        BusMode_T val = value; \
        _ssSafelyCallGenericFcnStart(S)((S),GEN_FCN_SET_INPUT_BUS_MODE, pIdx, &val)\
        _ssSafelyCallGenericFcnEnd; \
    } \
 }

#define ssSetOutputPortBusMode(S, pIdx, value) \
{ \
    if (ssGetOwnerBlock(S) != NULL) { \
       BusMode_T val = value; \
       _ssSafelyCallGenericFcnStart(S)((S),GEN_FCN_SET_OUTPUT_BUS_MODE, pIdx, &val)\
       _ssSafelyCallGenericFcnEnd; \
    } \
}
#else
#define ssSetInputPortBusMode(S, pIdx, value)
#define ssSetOutputPortBusMode(S, pIdx, value)
#endif
#else
#define ssSetInputPortBusMode(S, pIdx, value)
#define ssSetOutputPortBusMode(S, pIdx, value)
#endif


# if SS_SIM
#  define dtaGetDataTypeElementOffset(dta, blockPath, id, eIdx) \
   ((dta) != NULL ? \
   (dta)->getGenericDTAIntElemProp((dta)->dataTypeTable, (blockPath), (id), \
     (eIdx), GEN_DTA_INT_PROP_ELEMENT_OFFSET) : \
   -1)
# else
#  define dtaGetDataTypeElementOffset(dta, blockPath, id, eIdx) \
    dtaGetDataTypeElementOffset_cannot_be_used_in_RTW
# endif




#define ssGetTNextWasAdjusted(S, sti) \
        ((S)->mdlInfo->tNextWasAdjusted[sti]) /* (boolean_T) */


 /* InputPortAcceptExprInRTW - Specify that the input port signal can be
  * an expression in RTW.  This allows the block to fold an upstream
  * calculation into its own.
  */
# define ssGetInputPortAcceptExprInRTW(S,port) \
       ((S)->portInfo.inputs[(port)].attributes.acceptExprInRTW == 1U)
# define ssSetInputPortAcceptExprInRTW(S,port,val) \
       ((S)->portInfo.inputs[(port)].attributes.acceptExprInRTW = \
        (val) ? 1U : 0U)    

 /* OutputPortOutputExprInRTW - Specify that the output signal is an expression
  * in RTW (versus a statement).  This allows a downstream block to fold
  * the expression into its calculation.  Note that Simulink may not honor
  * this setting, in which case the block must produce a statement.
  */
# define ssGetOutputPortOutputExprInRTW(S,port) \
   ((S)->portInfo.outputs[(port)].attributes.outputExprInRTW == 1U)
# define ssSetOutputPortOutputExprInRTW(S,port,val) \
   ((S)->portInfo.outputs[(port)].attributes.outputExprInRTW = (val) ? 1U : \
   ((S)->portInfo.outputs[(port)].attributes.trivialOutputExprInRTW = \
    (S)->portInfo.outputs[(port)].attributes.constOutputExprInRTW = 0U))

#define ssSetmdlOutputs(S,outputs) \
         (S)->modelMethods.sFcn.mdlOutputs.level2 = (outputs)

# if SS_SIM
#define dtaGetDataTypeIdAliasedThruTo(dta, blockPath, id) \
   ((dta) != NULL ? \
   (dta)->getGenericDTAIntProp((dta)->dataTypeTable, (blockPath), (id), \
     GEN_DTA_INT_PROP_ID_ALIASED_THRU_TO) : \
   INVALID_DTYPE_ID)
# else
#  define dtaGetDataTypeIdAliasedThruTo(dta, blockPath, id) \
    dtaGetDataTypeIdAliasedThruTo_cannot_be_used_in_RTW
# endif


#endif  /* __SIMSTRUC__IMPLEMENT */
