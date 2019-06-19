/* Copyright 1990-2016 The MathWorks, Inc.
 *
 * File    : simulink.c
 * Abstract:
 *      Epilogue C include file used when compiling MEX-file S-functions.
 *
 *      This file should be included at the end of a MEX-file system.  It
 *      provides an interface to the MEX-file mechanism that allows blocks
 *      and systems to be entered only as their corresponding mathematical
 *      functions without the need for addition interface code
 *
 *      All local functions and defines begin with an underscore "_" to help
 *      in avoiding name conflicts with user functions and defines.
 *
 *      This file supports both level 1 and level 2 S-functions.
 */

#undef   printf   /* don't want to redefine mexPrintf! */
#include <stdio.h>

#include "simstruc_internal.h"

/* Simulink solver API */
#include "simulink_solver_api.c"

/*LINTLIBRARY*/

/*==============================*
 * Pre-processor error checking *
 *==============================*/

#ifndef S_FUNCTION_NAME
#error S_FUNCTION_NAME must be defined
#endif

#if defined(_S_FUNCTION_NAME_NOT_DEFINED_BEFORE_SIMSTRUCT)
#error S_FUNCTION_NAME must be defined before include of simstruc.h
#endif

#if S_FUNCTION_LEVEL == 2
# if defined(MDL_GET_INPUT_PORT_WIDTH)
#   error mdlGetInputPortWidth(S,outputWidth) cannot be used in \
        level 2 S-functions see mdlSetInputPortWidth(S,port,width) and \
        mdlSetOutputPortWidth(S,port,width)
# endif
# if defined(MDL_GET_OUTPUT_PORT_WIDTH)
#   error mdlGetOutputPortWidth(S,inputWidth) cannot be used in \
        level 2 S-functions see mdlSetInputPortWidth(S,port,width) and \
        mdlSetOutputPortWidth(S,port,width)
# endif


# if (defined(MDL_SET_INPUT_PORT_WIDTH) || \
      defined(MDL_SET_OUTPUT_PORT_WIDTH)) && \
     (defined(MDL_SET_INPUT_PORT_DIMENSION_INFO) || \
      defined(MDL_SET_OUTPUT_PORT_DIMENSION_INFO))
#   error Cannot use mdlSetInput(Output)PortWidth \
and mdlSetInput(Output)PortDimensionInfo at the same time in an S-function; \
Use either a width or dimension method, but not both

#endif
#endif


#if S_FUNCTION_LEVEL == 2
#  ifndef MDL_DERIVATIVES
#    if (defined(MDL_FORCINGFUNCTION) || defined(MDL_MASSMATRIX))
#      error mdlDerivatives required for linearly-implicit systems
#    endif
#  endif
#endif


/*=========*
 * Defines *
 *=========*/

#define _QUOTE1(name) #name
#define _QUOTE(name) _QUOTE1(name)   /* need to expand name */


#define _LHS_RET    0    /* return (sizes, derivs, dstates, output, tnext)   */
#define _LHS_X0     1    /* initial state conditions                         */
#define _LHS_STR    2    /* state strings                                    */
#define _LHS_TS     3    /* sample times (sampling period, offset)           */
#define _LHS_XTS    4    /* state sample times (sampling period, offset)     */

#define _LHS_SS     1    /* Pointer to SimStruct if (nlhs < 0)               */

#define _RHS_T      0    /* Time                                             */
#define _RHS_X      1    /* States                                           */
#define _RHS_U      2    /* Inputs                                           */
#define _RHS_FLAG   3    /* mode flag                                        */


#define _GetNumEl(pm) (mxGetM(pm)*mxGetN(pm))

#define _NumArg(x) "<arg type=\"numeric\">" #x "</arg>"
#define _StrArg(x) "<arg type=\"string\">" #x "</arg>"
#define _SimstructErrmsgWithArg(x,y) \
    "<diag_root><diag id=" #x "><arguments>" y "</arguments></diag></diag_root>"
#define _SimstructErrmsg(x) _SimstructErrmsgWithArg(x,"")
#define _mxStrArg(x) mxCreateString(x)
#define _mxNumArg(x) mxCreateDoubleScalar(x)

/*===========================*
 * Data local to this module *
 *===========================*/

static char_T _sfcnName[] = "MEX level" _QUOTE(S_FUNCTION_LEVEL)
     " S-function \"" _QUOTE(S_FUNCTION_NAME) "\"";

const DimsInfo_T DYNAMIC_DIMENSION_DEF = {-1, -1, NULL, NULL};
const DimsInfo_T *DYNAMIC_DIMENSION    = &DYNAMIC_DIMENSION_DEF;

/*===================*
 * Private functions *
 *===================*/

#define _ssFatalError_1(S,errid,arg1) {                                 \
    mxArray *prhs[2];                                                   \
    mxArray *plhs[1] = { NULL };                                        \
    prhs[0] = (errid);                                                  \
    prhs[1] = (arg1);                                                   \
    mexCallMATLAB(1, plhs, (sizeof(prhs)/sizeof(prhs[0])), prhs, "message"); \
    mexCallMATLAB(0, NULL, 1, plhs, "error"); }
#define _ssFatalError_2(S,errid,arg1,arg2) {                            \
    mxArray *prhs[3];                                                   \
    mxArray *plhs[1] = { NULL };                                        \
    prhs[0] = (errid);                                                  \
    prhs[1] = (arg1);                                                   \
    prhs[2] = (arg2);                                                   \
    mexCallMATLAB(1, plhs, (sizeof(prhs)/sizeof(prhs[0])), prhs, "message"); \
    mexCallMATLAB(0, NULL, 1, plhs, "error"); }
#define _ssFatalError_3(S,errid,arg1,arg2,arg3) {                       \
    mxArray *prhs[4];                                                   \
    mxArray *plhs[1] = { NULL };                                        \
    prhs[0] = (errid);                                                  \
    prhs[1] = (arg1);                                                   \
    prhs[2] = (arg2);                                                   \
    prhs[3] = (arg3);                                                   \
    mexCallMATLAB(1, plhs, (sizeof(prhs)/sizeof(prhs[0])), prhs, "message"); \
    mexCallMATLAB(0, NULL, 1, plhs, "error"); }
#define _ssFatalError_4(S,errid,arg1,arg2,arg3,arg4) {                  \
    mxArray *prhs[5];                                                   \
    mxArray *plhs[1] = { NULL };                                        \
    prhs[0] = (errid);                                                  \
    prhs[1] = (arg1);                                                   \
    prhs[2] = (arg2);                                                   \
    prhs[3] = (arg3);                                                   \
    prhs[4] = (arg4);                                                   \
    mexCallMATLAB(1, plhs, (sizeof(prhs)/sizeof(prhs[0])), prhs, "message"); \
    mexCallMATLAB(0, NULL, 1, plhs, "error"); }
#define _ssWarning_1(S,errid,arg1) {                                    \
    mxArray *prhs[2];                                                   \
    mxArray *plhs[1] = { NULL };                                        \
    prhs[0] = (errid);                                                  \
    prhs[1] = (arg1);                                                   \
    mexCallMATLAB(1, plhs, (sizeof(prhs)/sizeof(prhs[0])), prhs, "message"); \
    mexCallMATLAB(0, NULL, 1, plhs, "warning"); }


#if S_FUNCTION_LEVEL == 1 
/* For Internal use*/
# define ssLocalGetU(S) ((S)->states.U.vect) 
#endif

#ifndef MDL_ZERO_CROSSINGS
  /* Function: mdlZeroCrossings ===============================================
   * Abstract:
   *    This routine is present for S-functions which register that they
   *    have nonsampled zero crossings, but don't register this routine.
   */
  static void mdlZeroCrossings(SimStruct *S)
  {
      ssSetErrorStatus(S, _SimstructErrmsg("Simulink:SFunctions:SimStructNoMdlZeroCrossings"));
      return;
  }
#endif


#ifndef MDL_GET_TIME_OF_NEXT_VAR_HIT
  /* Function: mdlGetTimeOfNextVarHit =========================================
   * Abstract:
   *    This routine is present for backwards compatibility with
   *    Simulink 1.3 S-function which didn't have mdlGetTimeOfNextVarHit
   */
  static void mdlGetTimeOfNextVarHit(SimStruct *S)
  {
      ssSetErrorStatus(S, _SimstructErrmsg("Simulink:SFunctions:SimStructNoMdlGetTimeOfNextVarHit"));
      return;
  } /* end mdlGetTimeOfNextVarHit */
#endif


#if S_FUNCTION_LEVEL==2
/* Function: _RegNumInputPortsCB ==============================================
 * Abstract:
 *      Called by a level 2 S-function during mdlInitializeSizes.
 *
 * Returns:
 *      1  - register was successful
 *      0 - register was not successful
 */
static int_T _RegNumInputPortsCB(void *Sptr, int_T nInputPorts) {
    SimStruct *S = (SimStruct *)Sptr;

    if (nInputPorts < 0) {
        return(0);
    }

    _ssSetNumInputPorts(S,nInputPorts);
    _ssSetSfcnUsesNumPorts(S, 1);

    if (nInputPorts > 0) {
        ssSetPortInfoForInputs(S,
           (struct _ssPortInputs*)mxCalloc((size_t)nInputPorts,
                                           sizeof(struct _ssPortInputs)));
    }

    return(1);

} /* end _RegNumInputPortsCB */



/* Function: _RegNumOutputPortsCB =============================================
 * Abstract:
 *      Called by a level 2 S-function during mdlInitializeSizes.
 *
 * Returns:
 *      1  - register was successful
 *      0 - register was not successful
 */
static int_T _RegNumOutputPortsCB(void *Sptr, int_T nOutputPorts) {
    SimStruct *S = (SimStruct *)Sptr;

    if (nOutputPorts < 0) {
        return(0);
    }

    _ssSetNumOutputPorts(S,nOutputPorts);
    _ssSetSfcnUsesNumPorts(S, 1);

    if (nOutputPorts > 0) {
        ssSetPortInfoForOutputs(S,
            (struct _ssPortOutputs*)mxCalloc((size_t)nOutputPorts,
                                             sizeof(struct _ssPortOutputs)));
    }

    return(1);

} /* end _RegNumOutputPortsCB */

/* Function:  _ssSetInputPortMatrixDimensions ===================================
 * Returns:
 *      1 - set was successful
 *      0 - set was not successful
 */
int_T _ssSetInputPortMatrixDimensions(SimStruct *S, int_T port, int_T m, int_T n)
{
    int_T      status;
    DimsInfo_T dimsInfo = *DYNAMIC_DIMENSION;
    int_T      dims[2];

    dims[0]            = m;
    dims[1]            = n;

    dimsInfo.numDims   = 2;
    dimsInfo.dims      = dims;

    if ((m != DYNAMICALLY_SIZED) && (n != DYNAMICALLY_SIZED)) {
        if (m > INT_MAX/n) {
            /* Dimension overflow */
            return(status = 0);
        } else {
            dimsInfo.width = m*n;
        }
    } else {
        dimsInfo.width = DYNAMICALLY_SIZED;
    }

    status = ssSetInputPortDimensionInfo(S, port, &dimsInfo);

    return(status);
} /* end _ssSetInputPortMatrixDimensions */


/* Function:  _ssSetOutputPortMatrixDimensions ====================================
 * Returns:
 *      1 - set was successful
 *      0 - set was not successful
 */
int_T _ssSetOutputPortMatrixDimensions(SimStruct *S, int_T port, int_T m, int_T n)
{
    int_T      status;
    DimsInfo_T dimsInfo = *DYNAMIC_DIMENSION;
    int_T      dims[2];

    dims[0]            = m;
    dims[1]            = n;

    dimsInfo.numDims   = 2;
    dimsInfo.dims      = dims;

    if ((m != DYNAMICALLY_SIZED) && (n != DYNAMICALLY_SIZED)) {
        if (m > INT_MAX/n) {
            /* Dimension overflow */
            return(status = 0);
        } else {
            dimsInfo.width = m*n;
        }
    } else {
        dimsInfo.width = DYNAMICALLY_SIZED;
    }

    status = ssSetOutputPortDimensionInfo(S, port, &dimsInfo);

    return(status);
} /* end _ssSetOutputPortMatrixDimensions */

/* Function:  _ssSetInputPortVectorDimension ====================================
 * Returns:
 *      1 - set was successful
 *      0 - set was not successful
 */
int_T _ssSetInputPortVectorDimension(SimStruct *S, int_T port, int_T m)
{
    int_T      status;
    DimsInfo_T dimsInfo = *DYNAMIC_DIMENSION;

    dimsInfo.numDims   = 1;
    dimsInfo.dims      = &(dimsInfo.width);
    dimsInfo.width     = m;

    status = ssSetInputPortDimensionInfo(S, port, &dimsInfo);

    return(status);
} /* end _ssSetInputPortVectorDimension */


/* Function:  _ssSetOutputPortVectorDimension ====================================
 * Returns:
 *      1 - set was successful
 *      0 - set was not successful
 */
int_T _ssSetOutputPortVectorDimension(SimStruct *S, int_T port, int_T m)
{
    int_T      status;
    DimsInfo_T dimsInfo = *DYNAMIC_DIMENSION;

    dimsInfo.numDims   = 1;
    dimsInfo.dims      = &(dimsInfo.width);
    dimsInfo.width     = m;

    status = ssSetOutputPortDimensionInfo(S, port, &dimsInfo);

    return(status);
} /* end _ssSetOutputPortVectorDimension */

/* Function: _ssSetSymbolicDimsSupport =========================================
 * Abstract:
 *    This function is used to specify that an S-function does or does not
 *    support symbolic dimension (default false).
 */
void _ssSetSymbolicDimsSupport(SimStruct *S, const boolean_T val)
{
    _ssSafelyCallGenericFcnStart(S)(
        S, GEN_FCN_SET_SYMBOLIC_DIMS_SUPPORT, (int_T)val, NULL)
        _ssSafelyCallGenericFcnEnd;
}

/* Function: _ssRegisterSymbolicDimsExpr =======================================
 * Abstract:
 *    This function is used to create a symbolic dimensions id (SymbDimsId) from
 *    a expression. Notice that the expression string must form a valid syntax
 *    in C.
 */
SymbDimsId _ssRegisterSymbolicDimsExpr(SimStruct *S, const char_T* aExpr)
{
    ssSymbolicDimStringOp val = {NULL, SL_NOSYMBDIMS};
    val.expr = aExpr;
    _ssSafelyCallGenericFcnStart(S)(
        S, GEN_FCN_REGISTER_SYMBOLIC_DIMS_EXPR,0, &val)
        _ssSafelyCallGenericFcnEnd;
    return val.returnValue;
}

/* Function: _ssRegisterSymbolicDims ===========================================
 * Abstract:
 *    This function is used to create n-dimensional symbolic dimensions id 
 *    (SymbDimsId) from a vector of symbolic dimension ids.
 */
SymbDimsId _ssRegisterSymbolicDims(SimStruct *S, 
                                   const SymbDimsId* aDimsVec,
                                   const size_t aNumDims)
{ 
    ssSymbolicDimNDOp val = {NULL, 0U, SL_NOSYMBDIMS};
    val.dims    = aDimsVec;
    val.numDims = aNumDims;
    _ssSafelyCallGenericFcnStart(S)(
        S,  GEN_FCN_REGISTER_SYMBOLIC_DIMS, 0,&val)
        _ssSafelyCallGenericFcnEnd;
    return val.returnValue;
}

/* Function: _ssRegisterSymbolicDimsString =====================================
 * Abstract:
 *    This function is used to create a symbolic dimensions id (SymbDimsId) from
 *    a identifier string.
 */
SymbDimsId _ssRegisterSymbolicDimsString(SimStruct *S, const char_T* aString)
{
    ssSymbolicDimStringOp val = {NULL, SL_NOSYMBDIMS};
    val.expr = aString;
    _ssSafelyCallGenericFcnStart(S)(
        S, GEN_FCN_REGISTER_SYMBOLIC_DIMS_STRING, 0, &val)
        _ssSafelyCallGenericFcnEnd;
    return val.returnValue;
}

/* Function: _ssRegisterSymbolicDimsIntValue ===================================
 * Abstract:
 *    This function is used to create a symbolic dimensions id (SymbDimsId) from
 *    a integer value.
 */
SymbDimsId _ssRegisterSymbolicDimsIntValue(SimStruct *S, const int_T aIntValue)
{
    SymbDimsId val = SL_NOSYMBDIMS;
    _ssSafelyCallGenericFcnStart(S)(
        S, GEN_FCN_REGISTER_SYMBOLIC_DIMS_INT_VALUE, aIntValue, &val)
        _ssSafelyCallGenericFcnEnd;
    return val;
}

/* Function: _ssRegisterSymbolicDimsPlus =======================================
 * Abstract:
 *    This function is used to add two symbolic dimensions
 */
SymbDimsId _ssRegisterSymbolicDimsPlus(SimStruct *S, 
                                       const SymbDimsId aLHS,
                                       const SymbDimsId aRHS)
{
    SymbDimsId val = aRHS;
    _ssSafelyCallGenericFcnStart(S)(
        S, GEN_FCN_REGISTER_SYMBOLIC_DIMS_PLUS, aLHS, &val)
        _ssSafelyCallGenericFcnEnd;
    return val;
}

/* Function: _ssRegisterSymbolicDimsMinus ======================================
 * Abstract:
 *    This function is used to subtract two symbolic dimensions
 */
SymbDimsId _ssRegisterSymbolicDimsMinus(SimStruct *S, 
                                        const SymbDimsId aLHS,
                                        const SymbDimsId aRHS)
{
    SymbDimsId val = aRHS;
    _ssSafelyCallGenericFcnStart(S)(
        S, GEN_FCN_REGISTER_SYMBOLIC_DIMS_MINUS, aLHS, &val)
        _ssSafelyCallGenericFcnEnd;
    return val;
}

/* Function: _ssRegisterSymbolicDimsMultiply ===================================
 * Abstract:
 *    This function is used to multiply two symbolic dimensions
 */
SymbDimsId _ssRegisterSymbolicDimsMultiply(SimStruct *S, 
                                           const SymbDimsId aLHS,
                                           const SymbDimsId aRHS)
{
    SymbDimsId val = aRHS;
    _ssSafelyCallGenericFcnStart(S)(
        S, GEN_FCN_REGISTER_SYMBOLIC_DIMS_MULTIPLY, aLHS, &val)
        _ssSafelyCallGenericFcnEnd;
    return val;
}

/* Function: _ssRegisterSymbolicDimsDivide =====================================
 * Abstract:
 *    This function is used to divide two symbolic dimensions
 */
SymbDimsId _ssRegisterSymbolicDimsDivide(SimStruct *S, 
                                         const SymbDimsId aLHS,
                                         const SymbDimsId aRHS)
{
    SymbDimsId val = aRHS;
    _ssSafelyCallGenericFcnStart(S)(
        S, GEN_FCN_REGISTER_SYMBOLIC_DIMS_DIVIDE, aLHS, &val)
        _ssSafelyCallGenericFcnEnd;
    return val;
}

/* Function: _ssGetNumSymbolicDims =============================================
 * Abstract:
 *    This function returns the number of dimensions for a given symbolic 
 *    dimensions id (SymbDimsId)
 */
size_t _ssGetNumSymbolicDims(SimStruct *S, const SymbDimsId aSymbDimsId)
{
    size_t val = 0U;
    _ssSafelyCallGenericFcnStart(S)(
        S, GEN_FCN_GET_NUM_SYMBOLIC_DIMS, aSymbDimsId, &val)
        _ssSafelyCallGenericFcnEnd;
    return val;
}

/* Function: _ssGetSymbolicDim =================================================
 * Abstract:
 *    This function returns the number of dimensions for a given symbolic 
 *    dimensions id (SymbDimsId)
 */
SymbDimsId _ssGetSymbolicDim(SimStruct *S,
                             const SymbDimsId aSymbDimsId,
                             const int_T aDimsIdx)
{
    SymbDimsId val = aSymbDimsId; 
    _ssSafelyCallGenericFcnStart(S)(
        S, GEN_FCN_GET_SYMBOLIC_DIM, aDimsIdx, &val)
        _ssSafelyCallGenericFcnEnd;
    return val;
}

/* Function: _ssSetInputPortSymbolicDimsId =====================================
 * Abstract:
 *    This function sets the pre compiled symbolic dimensions id (SymbDimsId)
 *    of a given input port index.
 */
void _ssSetInputPortSymbolicDimsId(SimStruct *S, 
                                   const int_T aPortIdx, 
                                   const SymbDimsId aSymbDimsId)
{
    SymbDimsId val = aSymbDimsId;
    _ssSafelyCallGenericFcnStart(S)(
        S, GEN_FCN_SET_INPUT_PORT_SYMBOLIC_DIMS_ID, aPortIdx, &val)
        _ssSafelyCallGenericFcnEnd;
}

/* Function: _ssGetCompInputPortSymbolicDimsId =================================
 * Abstract:
 *    This function returns the compiled symbolic dimensions id (SymbDimsId)
 *    for given input port index.
 */
SymbDimsId _ssGetCompInputPortSymbolicDimsId(SimStruct *S, 
                                             const int_T aPortIdx)
{
    SymbDimsId val = SL_NOSYMBDIMS;
    _ssSafelyCallGenericFcnStart(S)(
        S, GEN_FCN_GET_COMP_INPUT_PORT_SYMBOLIC_DIMS_ID, aPortIdx, &val)
        _ssSafelyCallGenericFcnEnd;
    return val;
}

/* Function: _ssSetCompInputPortSymbolicDimsId =================================
 * Abstract:
 *    This function sets the compiled symbolic dimensions id (SymbDimsId)
 *    of a given input port index.
 */
void _ssSetCompInputPortSymbolicDimsId(SimStruct *S, 
                                       const int_T aPortIdx, 
                                       const SymbDimsId aSymbDimsId)
{ 
    SymbDimsId val = aSymbDimsId;
    _ssSafelyCallGenericFcnStart(S)(
        S, GEN_FCN_SET_COMP_INPUT_PORT_SYMBOLIC_DIMS_ID, aPortIdx, &val)
        _ssSafelyCallGenericFcnEnd;
}

/* Function: _ssSetOutputPortSymbolicDimsId ====================================
 * Abstract:
 *    This function sets the pre compiled symbolic dimensions id (SymbDimsId)
 *    of a given output port index.
 */
void _ssSetOutputPortSymbolicDimsId(SimStruct *S, 
                                    const int_T aPortIdx, 
                                    const SymbDimsId aSymbDimsId)
{
    SymbDimsId val = aSymbDimsId;
    _ssSafelyCallGenericFcnStart(S)(
        S, GEN_FCN_SET_OUTPUT_PORT_SYMBOLIC_DIMS_ID, aPortIdx, &val)
        _ssSafelyCallGenericFcnEnd;
}

/* Function: _ssGetCompOutputPortSymbolicDimsId ================================
 * Abstract:
 *    This function returns the compiled symbolic dimensions id (SymbDimsId)
 *    for given output port index.
 */
SymbDimsId _ssGetCompOutputPortSymbolicDimsId(SimStruct *S, 
                                              const int_T aPortIdx)
{
    SymbDimsId val = SL_NOSYMBDIMS;
    _ssSafelyCallGenericFcnStart(S)(
        S, GEN_FCN_GET_COMP_OUTPUT_PORT_SYMBOLIC_DIMS_ID, aPortIdx, &val)
        _ssSafelyCallGenericFcnEnd;
    return val;
}

/* Function: _ssSetCompOutputPortSymbolicDimsId ================================
 * Abstract:
 *    This function sets the compiled symbolic dimensions id (SymbDimsId)
 *    of a given output port index.
 */
void _ssSetCompOutputPortSymbolicDimsId(SimStruct *S, 
                                        const int_T aPortIdx, 
                                        const SymbDimsId aSymbDimsId)
{ 
    SymbDimsId val = aSymbDimsId;
    _ssSafelyCallGenericFcnStart(S)(
        S, GEN_FCN_SET_COMP_OUTPUT_PORT_SYMBOLIC_DIMS_ID, aPortIdx, &val)
        _ssSafelyCallGenericFcnEnd;
}

/* Function: _ssSetCompDWorkSymbolicDimsId =====================================
 * Abstract:
 *    This function sets the compiled symbolic dimensions id (SymbDimsId)
 *    of a given dwork port index.
 */
void _ssSetCompDWorkSymbolicDimsId(SimStruct *S, 
                                   const int_T aPortIdx, 
                                   const SymbDimsId aSymbDimsId)
{ 
    SymbDimsId val = aSymbDimsId;
    _ssSafelyCallGenericFcnStart(S)(
        S, GEN_FCN_SET_COMP_DWORK_SYMBOLIC_DIMS_ID, aPortIdx, &val)
        _ssSafelyCallGenericFcnEnd;
}

/* Function:  ssIsRunTimeParamTunable ==========================================
 * Returns:
 *      1 - if tunable
 *      0 - otherwise
 */
int_T ssIsRunTimeParamTunable(SimStruct *S, const int_T rtPIdx)
{
    int_T result = 0;

    _ssIsRunTimeParamTunable(S,rtPIdx,&result);
    return(result);
} /* end ssIsRunTimeParamTunable */


/* Function:  ssGetSFuncBlockHandle ============================================
 * Returns:
 *      A double - the handle to the owner block.
 */
double ssGetSFuncBlockHandle(SimStruct *S)
{
    double result = 0;

    _ssGetSFuncBlockHandle(S, &result);
    return(result);
} /* end ssGetSFuncBlockHandle */

/* Function:  _ssGetCurrentInputPortWidth ======================================
 * Returns:
 *      An int_T: current input port width
 */
int_T _ssGetCurrentInputPortWidth(SimStruct *S, int_T pIdx)
{
    if (ssGetInputPortDimensionsMode(S, pIdx) == FIXED_DIMS_MODE) {
        return(ssGetInputPortWidth(S, pIdx));
    }
    else {
        int_T currWidth = 1, dIdx;
        for(dIdx = 0; dIdx < ssGetInputPortNumDimensions(S, pIdx); dIdx++) {
            currWidth *= ssGetCurrentInputPortDimensions(S, pIdx, dIdx);
        }
        return(currWidth);
    }
} /* end _ssGetCurrentInputPortWidth */

/* Function:  _ssGetCurrentOutputPortWidth =====================================
 * Returns:
 *      An int_T: current output port width
 */
int_T _ssGetCurrentOutputPortWidth(SimStruct *S, int_T pIdx)
{
    if (ssGetOutputPortDimensionsMode(S, pIdx) == FIXED_DIMS_MODE) {
        return(ssGetOutputPortWidth(S, pIdx));
    }
    else {
        int_T currWidth = 1, dIdx;
        for(dIdx = 0; dIdx < ssGetOutputPortNumDimensions(S, pIdx); dIdx++) {
            currWidth *= ssGetCurrentOutputPortDimensions(S, pIdx, dIdx);
        }
        return(currWidth);
    }
} /* end _ssGetCurrentOutputPortWidth */

/* Function:  _ssGetCallSystemNumFcnCallDestinations ===========================
 * Returns:
 *      An int_T: the number of (branched) function-call destinations that are
 *      invoked from the specified element of the function-call port of the
 *      S-function.
 */
int_T _ssGetCallSystemNumFcnCallDestinations(SimStruct *S, int_T elemIdx)
{
    int_T nFcnCallDest = 0;
    ssGetNumFcnCallDestinations(S, elemIdx, &nFcnCallDest);
    return(nFcnCallDest);
} /* end _ssGetCallSystemNumFcnCallDestinations */

/* Function: ssGetParameterTuningCompliance ====================================
 * Returns:
 *      true - if tunable
 *      false - otherwise
 */
boolean_T ssGetParameterTuningCompliance(SimStruct *S)
{
    boolean_T parameterTuningCompliance = false;
    _ssSafelyCallGenericFcnStart(S)(S,GEN_FCN_GET_PRM_TUNING_COMPLIANCE,0, &parameterTuningCompliance) \
    _ssSafelyCallGenericFcnEnd; 
    return parameterTuningCompliance;
} /* end ssGetParameterTuningCompliance */

#endif



/* Function: _CreateSimStruct =================================================
 * Abstract:
 *      Allocate the simulation structure.
 */
static SimStruct *_CreateSimStruct(int_T nrhs, const mxArray *prhs[])
{
    int_T nParams = (nrhs > 4)? nrhs - 4: 0;
    SimStruct* S = (SimStruct*) mxCalloc((size_t) 1,sizeof(SimStruct));
    struct _ssMdlInfo* mdlInfo =
        (struct _ssMdlInfo*) mxCalloc((size_t) 1,sizeof(struct _ssMdlInfo));
    struct _ssStatesInfo2* stateInfo2 =
        (struct _ssStatesInfo2*) mxCalloc((size_t) 1,sizeof(struct _ssStatesInfo2));
    ssPeriodicStatesInfo* periodicStatesInfo =
        (ssPeriodicStatesInfo*) mxCalloc((size_t) 1,sizeof(ssPeriodicStatesInfo));

    mdlInfo->errorStatusBuffer = (char_T*)mxCalloc(SS_ERROR_STATUS_BUFFER_SIZE, sizeof(char_T));

    _ssSetRootSS(S, S);
    _ssSetMdlInfoPtr(S, mdlInfo);
    _ssSetSimMode(S, SS_SIMMODE_SIZES_CALL_ONLY);
    _ssSetSFcnParamsCount(S,nParams);

    _ssSetPath(S,_QUOTE(S_FUNCTION_NAME));
    _ssSetModelName(S,_QUOTE(S_FUNCTION_NAME));
    _ssSetStatesInfo2(S,stateInfo2);
    _ssSetPeriodicStatesInfo(S,periodicStatesInfo);

    if (nParams > 0) {
        /******************************
         * Load S-function parameters *
         ******************************/
        mxArray **ppa = (mxArray**)mxCalloc((size_t)nParams, sizeof(ppa[0]));
        int_T           i;

        _ssSetSFcnParamsPtr(S, ppa);

        for (i = 0; i < nParams; i++) {
            union { mxArray *mute; const mxArray *nonMute; } pa;
            pa.nonMute = prhs[4+i];
            _ssSetSFcnParam(S, i, pa.mute);
        }
    }

    /*
     * Setup to handle level 2, ssSetNum[Out|In]putPorts.
     */
#   if S_FUNCTION_LEVEL == 2
    ssSetRegNumInputPortsFcn(S, _RegNumInputPortsCB);
    ssSetRegNumInputPortsFcnArg(S, (void *)S);
    ssSetRegNumOutputPortsFcn(S, _RegNumOutputPortsCB);
    ssSetRegNumOutputPortsFcnArg(S, (void *)S);
#   endif

    return(S);

} /* end _CreateSimStruct */


/*==================*
 * Global functions *
 *==================*/


/* Function: ssWarning ========================================================
 * Abstract:
 *	Call mexWarnMstTxt to issue the specified warning message
 */
void ssWarning(SimStruct *S, const char *msg)
{
    const char fmt[] = "block '%s': %s";
    char *warnMsg = (ssGetPath(S) != NULL?
                     (char *)malloc(strlen(msg)+
                                    sizeof(fmt)+strlen(ssGetPath(S))+1):
                     NULL);
    if (warnMsg == NULL) {
        mexWarnMsgTxt(msg);
    } else {
        (void)sprintf(warnMsg,fmt,ssGetPath(S),msg);
        mexWarnMsgTxt(warnMsg);
        free(warnMsg);
    }
} /* end ssWarning */


/* ================== APIs for zero crossing ==================================
 * The set methods are implemented as macros and the get methods are functions
 */
/* ================== APIs for zero crossing =================================*/

/*get the zerocross control status from the Configset setting */

boolean_T ssGetIsZeroCrossControlDisabled(SimStruct *S)
{
    boolean_T zcDisabled = false;
    _ssSafelyCallGenericFcnStart(S)(S,GEN_FCN_GET_ZC_CONTROL_DISABLED,0, &zcDisabled) \
    _ssSafelyCallGenericFcnEnd; 
    return zcDisabled;
}

/*
 * Specify that the input port is being used to compute the continuous zero
 * crossing signal values
 */

void ssSetIsInputPortUsedForContZcSignal(SimStruct *S, int_T pIdx, boolean_T value) 
{   
    _ssSafelyCallGenericFcnStart(S)(S,GEN_FCN_SET_IS_INPORT_USEDFOR_CONT_ZC, pIdx, &value ) \
    _ssSafelyCallGenericFcnEnd; 
}

void ssSetZcSignalIsZcElementDisc(SimStruct *S, int_T zcsIdx, int_T zcsElIdx, boolean_T value) 
{ 
    _ssRegionElementIdxInfo locInfo; 
    locInfo._regionIdx   =  zcsIdx;  
    locInfo._regionElIdx =  zcsElIdx; 
    locInfo._result = value; 
    _ssSafelyCallGenericFcnStart(S)(S,GEN_FCN_SET_ZCSIGNAL_ISZCELEMEMT_DISC, zcsIdx, &locInfo) \
    _ssSafelyCallGenericFcnEnd; 
}

void ssSetZcSignalName(SimStruct *S, int_T zcsIdx, char* name) 
{
    _ssSafelyCallGenericFcnStart(S)(S,GEN_FCN_SET_ZCSIGNAL_NAME, zcsIdx, name) \
    _ssSafelyCallGenericFcnEnd; 
} 

void ssSetZcSignalWidth(SimStruct *S, int_T zcsIdx, int_T width) 
{ 
    _ssSafelyCallGenericFcnStart(S)(S,GEN_FCN_SET_ZCSIGNAL_WIDTH, zcsIdx, &width) \
    _ssSafelyCallGenericFcnEnd;\
}

void ssSetZcSignalZcEventType(SimStruct *S, int_T zcsIdx, slZcEventType zcEventType) 
{ 
    _ssSafelyCallGenericFcnStart(S)(S, GEN_FCN_SET_ZCSIGNAL_ZCEVENT_TYPE, zcsIdx, &zcEventType) \
    _ssSafelyCallGenericFcnEnd;
}

void ssSetZcSignalType(SimStruct *S, int_T zcsIdx, slZcSignalType zcSignalType)
{ 
    _ssSafelyCallGenericFcnStart(S)(S,GEN_FCN_SET_ZCSIGNAL_TYPE, zcsIdx, &zcSignalType) \
    _ssSafelyCallGenericFcnEnd;
}

void ssSetZcSignalZcTol(SimStruct *S, int_T zcsIdx,  double zcTol)
{ 
    _ssSafelyCallGenericFcnStart(S)(S,GEN_FCN_SET_ZCSIGNAL_ZCTOL, zcsIdx, &zcTol) \
    _ssSafelyCallGenericFcnEnd; 
}

void ssSetZcSignalNeedsEventNotification(SimStruct *S, int_T zcsIdx, boolean_T needsEventNotification)
{ 
    _ssSafelyCallGenericFcnStart(S)(S,GEN_FCN_SET_ZCSIGNAL_NEEDS_EVENT_NOTIFICATION, zcsIdx, &needsEventNotification) \
    _ssSafelyCallGenericFcnEnd; 
}

int_T ssCreateAndAddZcSignalInfo(SimStruct *S)
{
    int_T zcsIdx = -1; /* invalid number */
    _ssSafelyCallGenericFcnStart(S)(S,GEN_FCN_CREATE_AND_ADD_ZCSIGNAL_INFO, 0, &zcsIdx) \
    _ssSafelyCallGenericFcnEnd;
    return zcsIdx;
}

int_T ssCloneAndAddZcSignalInfo(SimStruct *S, int_T srcIdx)
{
    int_T dstIdx = -1; /* invalid number */
    _ssSafelyCallGenericFcnStart(S)(S, GEN_FCN_CLONE_AND_ADD_ZCSIGNAL_INFO, srcIdx, &dstIdx) \
        _ssSafelyCallGenericFcnEnd;
    return dstIdx;
}

int_T ssGetNumZcSignals(SimStruct *S)
{
    int_T numZcSignals = 0;
    _ssSafelyCallGenericFcnStart(S)(S,GEN_FCN_GET_NUM_ZCSIGNALS, 0, &numZcSignals) \
    _ssSafelyCallGenericFcnEnd; 
    return numZcSignals; 
}

double* ssGetZcSignalVector(SimStruct *S, int_T zcsIdx)
{
    double* zcsVector = NULL;
    _ssSafelyCallGenericFcnStart(S)(S, GEN_FCN_GET_ZCSIGNAL_VECTOR, zcsIdx, &zcsVector) \
    _ssSafelyCallGenericFcnEnd; 
    return zcsVector;
}

slZcEventType* ssGetZcSignalZcEvents(SimStruct *S, int_T zcsIdx)
{
    slZcEventType* zcEvents = NULL;
    _ssSafelyCallGenericFcnStart(S)(S, GEN_FCN_GET_ZCSIGNAL_ZCEVENTS, zcsIdx, &zcEvents) \
    _ssSafelyCallGenericFcnEnd; 
    return zcEvents;
}

const char* ssGetZcSignalName(SimStruct *S, int_T zcsIdx)
{
    char* name = NULL;
    _ssSafelyCallGenericFcnStart(S)(S,GEN_FCN_GET_ZCSIGNAL_NAME, zcsIdx, &name)
    _ssSafelyCallGenericFcnEnd; 
    return name;
}

int_T ssGetZcSignalWidth(SimStruct *S, int_T zcsIdx)
{
    int_T width = 0;
    _ssSafelyCallGenericFcnStart(S)(S,GEN_FCN_GET_ZCSIGNAL_WIDTH, zcsIdx, &width)
    _ssSafelyCallGenericFcnEnd; 
    return width;
}

slZcEventType ssGetZcSignalZcEventType(SimStruct *S, int_T zcsIdx)
{
    slZcEventType eventType = SL_ZCS_EVENT_ALL;
    _ssSafelyCallGenericFcnStart(S)(S,GEN_FCN_GET_ZCSIGNAL_ZCEVENT_TYPE, zcsIdx, &eventType)
    _ssSafelyCallGenericFcnEnd; 
    return eventType;
}

slZcSignalType ssGetZcSignalType(SimStruct *S, int_T zcsIdx)
{
    slZcSignalType zcsType = SL_ZCS_TYPE_CONT;
    _ssSafelyCallGenericFcnStart(S)(S,GEN_FCN_GET_ZCSIGNAL_TYPE, zcsIdx, &zcsType)
    _ssSafelyCallGenericFcnEnd; 
    return zcsType;
}

double ssGetZcSignalZcTol(SimStruct *S, int_T zcsIdx)
{
    double zcsTol = -1;
    _ssSafelyCallGenericFcnStart(S)(S,GEN_FCN_GET_ZCSIGNAL_ZCTOL, zcsIdx, &zcsTol)
    _ssSafelyCallGenericFcnEnd; 
    return zcsTol;
}

boolean_T ssGetZcSignalNeedsEventNotification(SimStruct *S, int_T zcsIdx)
{
    boolean_T needsEvent = false;
    _ssSafelyCallGenericFcnStart(S)(S,GEN_FCN_GET_ZCSIGNAL_NEEDS_EVENT_NOTIFICATION, zcsIdx, &needsEvent)
    _ssSafelyCallGenericFcnEnd; 
    return needsEvent;
}

boolean_T ssGetZcSignalIsZcElementDisc(SimStruct *S,
                                       int_T zcsIdx, 
                                       int_T zcsElIdx)
{
    _ssRegionElementIdxInfo locInfo;
    locInfo._regionIdx   = zcsIdx; 
    locInfo._regionElIdx = zcsElIdx;    
    locInfo._result = false;
    _ssSafelyCallGenericFcnStart(S)(S,GEN_FCN_GET_ZCSIGNAL_ISZCELEMEMT_DISC, 0, &locInfo) \
    _ssSafelyCallGenericFcnEnd; 
    return locInfo._result;
}

boolean_T ssGetIsInputPortElementContinuous(SimStruct *S,
                                            int_T pIdx, 
                                            int_T eIdx)
{                                           
    _ssRegionElementIdxInfo locInfo;
    locInfo._regionIdx   = pIdx;
    locInfo._regionElIdx = eIdx; 
    locInfo._result = false;
    _ssSafelyCallGenericFcnStart(S)(S,GEN_FCN_GET_IS_INPORT_ELEMENT_CONTINUOUS, 0, &locInfo) \
    _ssSafelyCallGenericFcnEnd; 
    return locInfo._result;
}

/* ==========================================================================*/



/* Function: _ssGetUFcn ========================================================
 * Abstract:
 *   Return the input signal for level 1 s-functions. Also detect
 *   illegal input signal access.
 */
#if S_FUNCTION_LEVEL == 1 && !defined(NDEBUG)
void *_ssGetUFcn(const SimStruct *S)
{
    if (S->states.U.vect == NULL) {
        _ssFatalError_2(S, _mxStrArg("Simulink:SFunctions:SimStructLevel1InvalidAccessToInputSignal"), _mxStrArg(ssGetPath(S)), _mxStrArg(ssGetModelName(S)));
    }

    return(S->states.U.vect);

} /* end _ssGetUFcn */
#endif


/* Function: _ssGetUPtrsFcn ====================================================
 * Abstract:
 *   Return the input signal for level 1 s-functions. Also detect
 *   illegal input signal access.
 */
#if S_FUNCTION_LEVEL == 1 && !defined(NDEBUG)
UPtrsType _ssGetUPtrsFcn(const SimStruct *S)
{
    if (S->states.U.uPtrs == NULL) {
     _ssFatalError_2(S, _mxStrArg("Simulink:SFunctions:SimStructLevel1InvalidAccessToInputSignal"), _mxStrArg(ssGetPath(S)), _mxStrArg(ssGetModelName(S)));
    }
 return(S->states.U.uPtrs);

} /* end _ssGetUPtrsFcn */
#endif



/* Function: _ssGetInputPortSignalPtrsFcn ======================================
 * Abstract:
 *   Return the input signal for level 2 s-functions. Also detect
 *   illegal input signal access.
 */
#if S_FUNCTION_LEVEL == 2 && !defined(NDEBUG)
InputPtrsType _ssGetInputPortSignalPtrsFcn(const SimStruct *S, int ip)
{
    int numports =  ((S)->sizes.in.numInputPorts);

    if (ip < 0 || ip >= numports) {
        _ssFatalError_4(S, _mxStrArg("Simulink:SFunctions:SimStructLevel2InvalidInputPortNumber"), _mxStrArg(ssGetModelName(S)), _mxStrArg(ssGetPath(S)), _mxNumArg(ip+1), _mxNumArg(numports-1));
    }

    if ((S)->portInfo.inputs[(ip)].signal.ptrs == NULL) {
        /* In case the user declared SS_REUSABLE_AND_LOCAL/SS_REUSABLE_AND_GLOBAL,
           input buffer may be set to NULL. Call _ssFatalError to report such error */
        if (ssGetInputPortOptimOpts((S), (ip)) == SS_REUSABLE_AND_GLOBAL ||
            ssGetInputPortOptimOpts((S), (ip)) == SS_REUSABLE_AND_LOCAL) {
            _ssFatalError_3(S, _mxStrArg("Simulink:SFunctions:SimStructLevel2InvalidAccessToReusableInputSignal"), _mxStrArg(ssGetModelName(S)), _mxStrArg(ssGetPath(S)), _mxNumArg(ip+1));
        }

        /* In case the user did not include ssSetInputPortDirectFeedThrough
         * in the mdlInitializeSizes method the direct feedthrough flag is
         * set to zero. Call _ssFatalError to report such error.
         */
        _ssFatalError_3(S, _mxStrArg("Simulink:SFunctions:SimStructLevel2InvalidAccessToNonDirectFeedthroughInputSignal"), _mxStrArg(ssGetModelName(S)), _mxStrArg(ssGetPath(S)), _mxNumArg(ip+1));
    }

    return((S)->portInfo.inputs[(ip)].signal.ptrs);
}
#endif



/* Function: _ssGetInputPortSignalFcn ======================================
 * Abstract:
 *   Return the input signal for level 2 s-functions. Also detect
 *   illegal input signal access.
 */
#if S_FUNCTION_LEVEL == 2 && !defined(NDEBUG)
const void *_ssGetInputPortSignalFcn(const SimStruct *S, int ip)
{
    int numports =  ((S)->sizes.in.numInputPorts);

    if (ip < 0 || ip >= numports) {
        _ssFatalError_4(S, _mxStrArg("Simulink:SFunctions:SimStructLevel2InvalidInputPortNumber"), _mxStrArg(ssGetModelName(S)), _mxStrArg(ssGetPath(S)), _mxNumArg(ip+1), _mxNumArg(numports-1));
    }

    if ((S)->portInfo.inputs[(ip)].signal.vect == NULL) {
        /* In case the user declared SS_REUSABLE_AND_LOCAL/SS_REUSABLE_AND_GLOBAL,
           input buffer may be set to NULL. Call _ssFatalError to report such error */
        if (ssGetInputPortOptimOpts((S), (ip)) == SS_REUSABLE_AND_GLOBAL ||
            ssGetInputPortOptimOpts((S), (ip)) == SS_REUSABLE_AND_LOCAL) {
            _ssFatalError_3(S, _mxStrArg("Simulink:SFunctions:SimStructLevel2InvalidAccessToReusableInputSignal"), _mxStrArg(ssGetModelName(S)), _mxStrArg(ssGetPath(S)), _mxNumArg(ip+1));
        }

        /* In case the user did not include ssSetInputPortDirectFeedThrough
         * in the mdlInitializeSizes method the direct feedthrough flag is
         * set to zero. Call _ssFatalError to report such error.
         */
        _ssFatalError_3(S, _mxStrArg("Simulink:SFunctions:SimStructLevel2InvalidAccessToNonDirectFeedthroughInputSignal"), _mxStrArg(ssGetModelName(S)), _mxStrArg(ssGetPath(S)), _mxNumArg(ip+1));
    }

    return ((S)->portInfo.inputs[(ip)].signal.vect);
}
#endif

/* Function: ssSetInputPortDirectFeedThrough  ===================================
* Abstract:
*   Set the direct feedthrough flag. Also detect for illegal settings
*/
#if S_FUNCTION_LEVEL == 2 && !defined(NDEBUG)
void _ssSetInputPortDirectFeedThroughFcn(const SimStruct *S, int ip,
                                             int dirFeed)
{
    /* Call _ssFatalError if
       Num input ports == 1 and ssSetInputPortDirectFeedThrough(S, 1, 1)
       This is a common mistake. It should be (S, 0, 1).
     */
    int numports = ((S)->sizes.in.numInputPorts);
    if (ip < 0 || ip >= numports) {
        _ssFatalError_4(S, _mxStrArg("Simulink:SFunctions:SimStructLevel2InvalidInputPortNumberForDirectFeedThrough"), _mxStrArg(ssGetModelName(S)), _mxStrArg(ssGetPath(S)), _mxNumArg(ip+1), _mxNumArg(numports-1));
    }

    (S)->portInfo.inputs[(ip)].directFeedThrough = (dirFeed);
}

/* Function: ssSetInputPortReusableFcn =========================================
 * Abstract:
 *   Issue a warning that the macro ssSetInputPortReusableFcn is invalid
 */
void _ssSetInputPortReusableFcn(SimStruct* S, int ip, int val)
{
    _ssWarning_1(S, _mxStrArg("Simulink:SFunctions:SimStructReplace_ssSetInputPortReusable_With_ssSetInputPortOptimOpts"), _mxStrArg(_sfcnName));

    S->portInfo.inputs[ip].attributes.optimOpts = (val) ?
        SS_REUSABLE_AND_LOCAL : SS_NOT_REUSABLE_AND_GLOBAL;
}

/* Function: ssSetOutputPortReusableFcn ========================================
 * Abstract:
 *   Issue a warning that the macro ssSetOutputPortReusableFcn is invalid
 */
void _ssSetOutputPortReusableFcn(SimStruct* S, int op, int val)
{
    _ssWarning_1(S, _mxStrArg("Simulink:SFunctions:SimStructReplace_ssSetOutputPortReusable_With_ssSetOutputPortOptimOpts"), _mxStrArg(_sfcnName));

    S->portInfo.outputs[op].attributes.optimOpts = (val) ?
        SS_REUSABLE_AND_LOCAL : SS_NOT_REUSABLE_AND_GLOBAL;
}

#endif

/* Function: ssGetDTypeIdFromMxArray ===========================================
 * Abstract:
 *      Utility to translate the mxClassId of an mxArray to one of Simulink's
 *      built-in data type indices. The return value is of type DTypeId, which
 *      is defined in simstruc.h
 *
 *      This function returns INVALID_DTYPE_ID if the mxClassId does not map to
 *      any built-in Simulink data type. For example, if mxId == mxSTRUCT_CLASS
 *      then the return value is INVALID_DTYPE_ID.
 *      Otherwise the return value is one of the enum values in BuiltInDTypeId.
 *      For example if mxId == mxUINT16_CLASS then the return value is SS_UINT16
 */
DTypeId ssGetDTypeIdFromMxArray(const mxArray *m)
{
    DTypeId dTypeId;
    mxClassID mxId = mxGetClassID(m);

    switch (mxId) {
      case mxCELL_CLASS:
      case mxSTRUCT_CLASS:
      case mxOBJECT_CLASS:
      case mxCHAR_CLASS:
        dTypeId = INVALID_DTYPE_ID;
        break;
      case mxDOUBLE_CLASS:
        dTypeId = SS_DOUBLE;
        break;
      case mxSINGLE_CLASS:
        dTypeId = SS_SINGLE;
        break;
      case mxINT8_CLASS:
        dTypeId = SS_INT8;
        break;
      case mxUINT8_CLASS:
        dTypeId = SS_UINT8;
        break;
      case mxINT16_CLASS:
        dTypeId = SS_INT16;
        break;
      case mxUINT16_CLASS:
        dTypeId = SS_UINT16;
        break;
      case mxINT32_CLASS:
        dTypeId = SS_INT32;
        break;
      case mxUINT32_CLASS:
        dTypeId = SS_UINT32;
        break;
      case mxLOGICAL_CLASS:
        dTypeId = SS_BOOLEAN;
        break;
      case mxINT64_CLASS:
      case mxUINT64_CLASS:
      case mxUNKNOWN_CLASS:
        dTypeId = INVALID_DTYPE_ID;
        break;
      default:
        dTypeId = INVALID_DTYPE_ID;
        break;
    }
    return(dTypeId);

} /* end ssGetDTypeIdFromMxArray */


/*===============================================*
 * Global functions (can be used only in mdlRTW) *
 *===============================================*/


/* Function: ssWriteRTWStr ====================================================
 * Abstract:
 *	Only for use in the mdlRTW method. This is a "low-level" routine
 *	for writing strings directly into the model.rtw file. It typically
 *	shouldn't be used by S-functions, unless you need to create
 *	"sub" Block record. These records should start with SFcn to
 *	avoid future compatibility problems. For example:
 *
 *	mdlRTW()
 *	{
 *	   if (!ssWriteRTWStr(S, "SFcnMySpecialRecord {")) return;
 *	   <snip>
 *	   if (!ssWriteRTWStr(S, "}")) return;
 *      }
 *
 */
int_T ssWriteRTWStr(SimStruct *S, const char_T *str)
{
    int_T ans = 0;
#   if SS_DO_FCN_CALL_ON_MAC && defined(__MWERKS__)
#   pragma mpwc on
#   endif
    ans = (*S->mdlInfo->writeRTWStrFcn)(S->mdlInfo->writeRTWFcnArg, str);
#   if SS_DO_FCN_CALL_ON_MAC && defined(__MWERKS__)
#   pragma mpwc off
#   endif
    return(ans);

} /* end ssWriteRTWStr */


/* Function: ssWriteRTWMxVectParam =============================================
 * Abstract:
 *	Only for use in the mdlRTW method. This is a "low-level" routine for
 *      writing Matlab style vectors (could be complex valued) into the mdl.rtw
 *      file. Typically this function should not be used by S-functions, the
 *      function ssWriteRTWParamSettings is more appropriate in most cases.
 *      However, this function is useful when you are writing out custom data
 *      structures directly in the mdl.rtw file.
 */
int_T ssWriteRTWMxVectParam(SimStruct    *S,
                            const char_T *name,
                            const void   *rVal,
                            const void   *iVal,
                            int_T        dtInfo,
                            int_T        numEl)
{
    const void *pD[2];

    pD[0] = rVal;
    pD[1] = iVal;
    return( ssWriteRTWNameValuePair(S,SSWRITE_VALUE_DTYPE_ML_VECT,
                                    name, pD, dtInfo, numEl) );

} /* end ssWriteRTWMxVectParam */


/* Function: ssWriteRTWMx2dMatParam ============================================
 * Abstract:
 *	Only for use in the mdlRTW method. This is a "low-level" routine for
 *      writing Matlab style matrices (could be complex valued) into the mdl.rtw
 *      file. Typically this function should not be used by S-functions, the
 *      function ssWriteRTWParamSettings is more appropriate in most cases.
 *      However, this function is useful when you are writing out custom data
 *      structures directly in the mdl.rtw file.
 */
int_T ssWriteRTWMx2dMatParam(SimStruct    *S,
                             const char_T *name,
                             const void   *rVal,
                             const void   *iVal,
                             int_T        dtInfo,
                             int_T        nRows,
                             int_T        nCols)
{
    const void *pD[2];
    pD[0] = rVal;
    pD[1] = iVal;
    return( ssWriteRTWNameValuePair(S,SSWRITE_VALUE_DTYPE_ML_2DMAT,
                                    name, pD, dtInfo, nRows, nCols) );

} /* end ssWriteRTWMx2dMatParam */


/* Function: ssWriteRTWNameValuePair ===========================================
 * Abstract:
 *	Only for use in the mdlRTW method. This is a "low-level" routine for
 *      writing name-value pairs directly into the model.rtw file. The input
 *      arguments for this function are subject to change, therefore you should
 *      not invoke this function directly. Instead, use the ssWriteRTWxxxParam()
 *      macros documented in matlabroot/simulink/src/sfuntmpl.doc (also see
 *      matlabroot/simulink/src/ml2rtw.c for examples of usage). The macros will
 *      in turn invoke this function with the appropriate arguments. Using these
 *      macros you will be able to write custom sub-records into your S-Function
 *      block record in the .rtw file. For example:
 *
 *	   mdlRTW()
 *	   {
 *	      if (!ssWriteRTWStr(S, "SFcnMySpecialRecord {")) return;
 *	      if (!ssWriteRTWStrParam(S, SSWRITE_VALUE_STR, "MyFieldName",
 *                                         "IsVeryVeryCool")) return;
 *	      if (!ssWriteRTWStr(S, "}")) return;
 *         }
 *
 *      will create the following sub-record in the Block's record:
 *
 *         Block {
 *            :
 *            :
 *            SFcnMySpecialRecord {
 *               MyFieldName   IsVeryVeryCool
 *            }
 *            :
 *            :
 *         }
 *
 *      Beware that you can easily corrupt the model.rtw file via these macros.
 *
 * Returns:
 *     1 - success
 *     0 - error. The appropriate error message is set in ssGetErrorStatus(S)
 */
int_T ssWriteRTWNameValuePair(SimStruct    *S,
                              int_T        type,
                              const char_T *name,
                              const void   *value,
                              ...)
{
    int_T      ans        = 1;      /* assume */
    int_T      dtInfo     = 0;      /* real and double */
    int_T      nRows      = 1;
    int_T      nCols      = 1;
    int_T      haveImData = 0;      /* assume */
    int_T      haveNCols  = 0;
    const void *pValue    = NULL;

    va_list    ap;
    va_start(ap, value);
    pValue = value;

    switch (type) {
      case SSWRITE_VALUE_STR:
      case SSWRITE_VALUE_QSTR:
        /* No additional args */
        break;
      case SSWRITE_VALUE_VECT_STR:
        nRows     = va_arg(ap, int_T);  /* nItems e.g. ["a", "b", "c"] has 3 */
        break;
      case SSWRITE_VALUE_NUM:
      case SSWRITE_VALUE_DTYPE_NUM:
        dtInfo = va_arg(ap, int_T);
        break;
      case SSWRITE_VALUE_VECT:
      case SSWRITE_VALUE_DTYPE_VECT:
        dtInfo     = va_arg(ap, int_T);
        nRows      = va_arg(ap, int_T);
        break;
      case SSWRITE_VALUE_2DMAT:
      case SSWRITE_VALUE_DTYPE_2DMAT:
        dtInfo    = va_arg(ap, int_T);
        nRows     = va_arg(ap, int_T);
        nCols     = va_arg(ap, int_T);
        haveNCols = 1;
        break;
      case SSWRITE_VALUE_DTYPE_ML_VECT:
        {
            haveImData = 1;
            dtInfo     = va_arg(ap, int_T);
            nRows      = va_arg(ap, int_T);
            break;
        }
      case SSWRITE_VALUE_DTYPE_ML_2DMAT:
        {
            haveImData = 1;
            haveNCols  = 1;
            dtInfo     = va_arg(ap, int_T);
            nRows      = va_arg(ap, int_T);
            nCols      = va_arg(ap, int_T);
            break;
        }
      default:
        {
            ssSetErrorStatus(S,  _SimstructErrmsg("Simulink:SFunctions:SimStructInvalidRTWPairValueType"));
            ans = 0;
            goto EXIT_POINT;
        }
    }

    if (name == NULL || name[0] == '\0') {
        ssSetErrorStatus(S, _SimstructErrmsg("Simulink:SFunctions:SimStructInvalidRTWPairName"));
        ans = 0;
        goto EXIT_POINT;
    }

    if (nRows < 0 || nCols < 0 ||
        (haveNCols && ((nRows==0 && nCols!=0) || (nRows!=0 && nCols==0)))) {
        ssSetErrorStatus(S, _SimstructErrmsg("Simulink:SFunctions:SimStructInvalidRTWPairDimension"));
        ans = 0;
        goto EXIT_POINT;
    }

    if (nRows != 0) {
        int_T ok = 1;

        if (haveImData) {
            const void * const *v2 = (const void * const *) value;
            if (v2[0] == NULL ||
                (GET_COMPLEX_SIGNAL(dtInfo) && v2[1] == NULL)) {
                ok = 0;
            }
        } else if (value == NULL) {
            ok = 0;
        }

        if (!ok) {
            ssSetErrorStatus(S, _SimstructErrmsg("Simulink:SFunctions:SimStructNullRTWPair"));
            ans = 0;
            goto EXIT_POINT;
        }
    }


#   if SS_DO_FCN_CALL_ON_MAC && defined(__MWERKS__)
#   pragma mpwc on
#   endif
    ans = (*S->mdlInfo->writeRTWNameValuePairFcn)(S->mdlInfo->writeRTWFcnArg,
                                        type, name, pValue,  dtInfo, nRows, nCols);
#   if SS_DO_FCN_CALL_ON_MAC && defined(__MWERKS__)
#   pragma mpwc off
#   endif

EXIT_POINT:

    va_end(ap);
    return(ans);

} /* end ssWriteRTWNameValuePair */



/* Function: ssWriteRTWParameters =============================================
 * Abstract:
 *	Used in mdlRTW to create Parameter records for your S-function.
 *      nParams is the number of tunable S-function parameters. Each parameter
 *	starts with an SSWRITE_VALUE_type which can be:
 *
 *         SSWRITE_VALUE_VECT,
 *           const char_T   *paramName,
 *           const char_T   *stringInfo,
 *           const real_T   *valueVect,
 *           int_T          vectLen
 *
 *         SSWRITE_VALUE_2DMAT,
 *           const char_T   *paramName,
 *           const char_T   *stringInfo,
 *           const real_T   *valueMat,
 *           int_T          nRows,
 *           int_T          nCols
 *
 *         SSWRITE_VALUE_DTYPE_VECT,
 *           const char_T   *paramName,
 *           const char_T   *stringInfo,
 *           const void     *valueVect,
 *           int_T          vectLen,
 *           int_T          dtInfo
 *
 *         SSWRITE_VALUE_DTYPE_2DMAT,
 *           const char_T   *paramName,
 *           const char_T   *stringInfo,
 *           const void     *valueMat,
 *           int_T          nRows,
 *           int_T          nCols,
 *           int_T          dtInfo
 *
 *         SSWRITE_VALUE_DTYPE_ML_VECT,
 *           const char_T   *paramName,
 *           const char_T   *stringInfo,
 *           const void     *rValueVect,
 *           const void     *iValueVect,
 *           int_T          vectLen,
 *           int_T          dtInfo
 *
 *         SSWRITE_VALUE_DTYPE_ML_2DMAT,
 *           const char_T   *paramName,
 *           const char_T   *stringInfo,
 *           const void     *rValueMat,
 *           const void     *iValueMat,
 *           int_T          nRows,
 *           int_T          nCols,
 *           int_T          dtInfo
 *
 */
int_T ssWriteRTWParameters(SimStruct *S, int_T nParams, ...)
{
    int_T     i;
    int_T     ans = 1; /* assume */
    va_list   ap;
    va_start(ap, nParams);

    for (i=0; i< nParams; i++) {
        int_T        type   = va_arg(ap, int_T);
        const char_T *name  = va_arg(ap, const char_T *);
        const char_T *str   = va_arg(ap, const char_T *);
        int_T        dtInfo = 0; /* real and double */
        int_T        nRows;
        int_T        nCols;
        const void   *ppValue[2];
        const void   *pValue;


        switch (type) {
          case SSWRITE_VALUE_VECT:
            {
                pValue = va_arg(ap, const real_T *);
                nRows  = va_arg(ap, int_T);
                nCols  = (nRows == 0)? 0: 1;
                dtInfo = 0; /* real, double */
            }
            break;
          case SSWRITE_VALUE_2DMAT:
            {
                pValue = va_arg(ap, const real_T *);
                nRows  = va_arg(ap, int_T);
                nCols  = va_arg(ap, int_T);
                dtInfo = 0; /* real, double */
            }
            break;
          case SSWRITE_VALUE_DTYPE_VECT:
            {
                pValue = va_arg(ap, const void *);
                nRows  = va_arg(ap, int_T);
                nCols  = (nRows == 0)? 0: 1;
                dtInfo = va_arg(ap, int_T);
            }
            break;
          case SSWRITE_VALUE_DTYPE_2DMAT:
            {
                pValue = va_arg(ap, const void *);
                nRows  = va_arg(ap, int_T);
                nCols  = va_arg(ap, int_T);
                dtInfo = va_arg(ap, int_T);
            }
            break;
          case SSWRITE_VALUE_DTYPE_ML_VECT:
            {
                ppValue[0] = va_arg(ap, const void *); /* real part */
                ppValue[1] = va_arg(ap, const void *); /* imag part */
                pValue     = ppValue;

                nRows      = va_arg(ap, int_T);
                nCols      = (nRows == 0)? 0: 1;
                dtInfo     = va_arg(ap, int_T);
            }
            break;
          case SSWRITE_VALUE_DTYPE_ML_2DMAT:
            {
                ppValue[0] = va_arg(ap, const void *);
                ppValue[1] = va_arg(ap, const void *);
                pValue     = ppValue;

                nRows      = va_arg(ap, int_T);
                nCols      = va_arg(ap, int_T);
                dtInfo     = va_arg(ap, int_T);
            }
            break;
          default:
            {
                ssSetErrorStatus(S, _SimstructErrmsg("Simulink:SFunctions:SimStructInvalidRTWParamValueType"));
                ans = 0;
                goto EXIT_POINT;
            }
        }

        if ( name == NULL || name[0] == '\0' ||
             (pValue == NULL && nRows != 0) ||
             nRows < 0 || nCols < 0 ||
             (nRows == 0 && nCols != 0) || (nRows != 0 && nCols == 0)) {
            ssSetErrorStatus(S, _SimstructErrmsg("Simulink:SFunctions:SimStructInvalidRTWParamArgument"));
            ans = 0;
            goto EXIT_POINT;
        }

#       if SS_DO_FCN_CALL_ON_MAC && defined(__MWERKS__)
#       pragma mpwc on
#       endif
        ans = (*S->mdlInfo->writeRTWParameterFcn)(S->mdlInfo->writeRTWFcnArg,
                                                  type, name, str, pValue,
                                                  dtInfo, nRows, nCols);
#       if SS_DO_FCN_CALL_ON_MAC && defined(__MWERKS__)
#       pragma mpwc off
#       endif

        if (ans == 0) {
            goto EXIT_POINT;
        }
    }

  EXIT_POINT:

    va_end(ap);
    return(ans);

} /* end ssWriteRTWParameters */



/* Function: ssWriteRTWParamSettings ==========================================
 * Abstract:
 *	Used in mdlRTW to create the SFcnParameterSettings record for
 *	your S-function (these are generally derived from the non-tunable
 *	parameters). For each parameter a "group" of values must be specified.
 *	These adhere to the following format:
 *
 *         SSWRITE_VALUE_STR,              - Used to write (un)quoted strings
 *           const char_T *settingName,      example:
 *           const char_T *value,              Country      USA
 *
 *         SSWRITE_VALUE_QSTR,             - Used to write quoted strings
 *           const char_T *settingName,      example:
 *           const char_T *value,              Country      "U.S.A"
 *
 *         SSWRITE_VALUE_VECT_STR,         - Used to write vector of strings
 *           const char_T *settingName,      example:
 *           const char_T *value,              Countries    ["USA", "Mexico"]
 *           int_T        nItemsInVect
 *
 *         SSWRITE_VALUE_NUM,              - Used to write numbers
 *           const char_T *settingName,      example:
 *           const real_T value                 NumCountries  2
 *
 *
 *         SSWRITE_VALUE_VECT,             - Used to write numeric vectors
 *           const char_T *settingName,      example:
 *           const real_T *settingValue,       PopInMil        [300, 100]
 *           int_T        vectLen
 *
 *         SSWRITE_VALUE_2DMAT,            - Used to write 2D matrices
 *           const char_T *settingName,      example:
 *           const real_T *settingValue,       PopInMilBySex  Matrix(2,2)
 *           int_T        nRows,                   [[170, 130],[60, 40]]
 *           int_T        nCols
 *
 *         SSWRITE_VALUE_DTYPE_NUM,        - Used to write numeric vectors
 *           const char_T   *settingName,    example: int8 Num 3+4i
 *           const void     *settingValue,   written as: [3+4i]
 *           int_T          dtInfo
 *
 *
 *         SSWRITE_VALUE_DTYPE_VECT,       - Used to write data typed vectors
 *           const char_T   *settingName,    example: int8 CArray [1+2i 3+4i]
 *           const void     *settingValue,   written as:
 *           int_T          vectLen             CArray  [1+2i, 3+4i]
 *           int_T          dtInfo
 *
 *
 *         SSWRITE_VALUE_DTYPE_2DMAT,      - Used to write data typed 2D
 *           const char_T   *settingName     matrices
 *           const void     *settingValue,   example:
 *           int_T          nRow ,            int8 CMatrix  [1+2i 3+4i; 5 6]
 *           int_T          nCols,            written as:
 *           int_T          dtInfo               CMatrix         Matrix(2,2)
 *                                                [[1+2i, 3+4i]; [5+0i, 6+0i]]
 *
 *
 *         SSWRITE_VALUE_DTYPE_ML_VECT,    - Used to write complex matlab data
 *           const char_T   *settingName,    typed vectors example:
 *           const void     *settingRValue,  example: int8 CArray [1+2i 3+4i]
 *           const void     *settingIValue,      settingRValue: [1 3]
 *           int_T          vectLen              settingIValue: [2 4]
 *           int_T          dtInfo
 *                                             written as:
 *                                                CArray    [1+2i, 3+4i]
 *
 *         SSWRITE_VALUE_DTYPE_ML_2DMAT,   - Used to write matlab complex
 *           const char_T   *settingName,    data typed 2D matrices
 *           const void     *settingRValue,  example
 *           const void     *settingIValue,      int8 CMatrix [1+2i 3+4i; 5 6]
 *           int_T          nRows                settingRValue: [1 5 3 6]
 *           int_T          nCols,               settingIValue: [2 0 4 0]
 *           int_T          dtInfo
 *                                              written as:
 *                                              CMatrix         Matrix(2,2)
 *                                                [[1+2i, 3+4i]; [5+0i, 6+0i]]
 *
 *-----------------------------------------------------------------------------
 */
int_T ssWriteRTWParamSettings(SimStruct *S, int_T nParams, ...)
{
    int_T i;
    int_T ans   = 1; /* assume */

    va_list   ap;
    va_start(ap, nParams);

    if (nParams <= 0) {
        ans = 0;
        goto EXIT_POINT;
    }

    if ( (ans = ssWriteRTWStr(S, "SFcnParamSettings {")) != 1 ) {
        goto EXIT_POINT;
    }
    for (i=0; i< nParams; i++) {
        int_T        type   = va_arg(ap, int_T);
        const char_T *name  = va_arg(ap, const char_T *);
        int_T        dtInfo = 0; /* real and double */
        int_T        nRows;
        int_T        nCols;

        switch (type) {
          case SSWRITE_VALUE_STR:
            /*FALLTHROUGH*/
          case SSWRITE_VALUE_QSTR: {
            const char_T *pValue = va_arg(ap, const char_T *);
            ans = ssWriteRTWNameValuePair(S,type,name,pValue);
            break;
          }
          case SSWRITE_VALUE_VECT_STR: {
            const char_T *pValue = va_arg(ap, const char_T *);
            nRows = va_arg(ap, int_T);
            ans = ssWriteRTWNameValuePair(S,type,name,pValue,nRows);
            break;
          }
          case SSWRITE_VALUE_NUM: {
            real_T value = va_arg(ap, real_T);
            ans = ssWriteRTWNameValuePair(S,type,name,&value,dtInfo);
            break;
          }
          case SSWRITE_VALUE_VECT: {
            const real_T *pValue = va_arg(ap, const real_T *);
            nRows = va_arg(ap, int_T);
            ans = ssWriteRTWNameValuePair(S,type,name,pValue,dtInfo,nRows);
            break;
          }
          case SSWRITE_VALUE_2DMAT: {
            const real_T *pValue = va_arg(ap, const real_T *);
            nRows = va_arg(ap, int_T);
            nCols = va_arg(ap, int_T);
            ans= ssWriteRTWNameValuePair(S,type,name,pValue,
                                         dtInfo,nRows,nCols);
            break;
          }
          case SSWRITE_VALUE_DTYPE_NUM: {
            const void *pValue = va_arg(ap, const void *);
            dtInfo = va_arg(ap, int_T);
            ans    = ssWriteRTWNameValuePair(S,type,name,pValue,dtInfo);
            break;
          }
          case SSWRITE_VALUE_DTYPE_VECT: {
            const void *pValue = va_arg(ap, const void *);
            nRows  = va_arg(ap, int_T);
            dtInfo = va_arg(ap, int_T);
            ans    = ssWriteRTWNameValuePair(S,type,name,pValue,dtInfo,nRows);
            break;
          }
          case SSWRITE_VALUE_DTYPE_2DMAT: {
            const void *pValue = va_arg(ap, const void *);
            nRows  = va_arg(ap, int_T);
            nCols  = va_arg(ap, int_T);
            dtInfo = va_arg(ap, int_T);
            ans    = ssWriteRTWNameValuePair(S,type,name,pValue,
                                             dtInfo,nRows,nCols);
            break;
          }
          case SSWRITE_VALUE_DTYPE_ML_VECT: {
            const void *pValue[2];
            pValue[0] = va_arg(ap, const void *);
            pValue[1] = va_arg(ap, const void *);
            nRows  = va_arg(ap, int_T);
            dtInfo = va_arg(ap, int_T);
            ans    = ssWriteRTWNameValuePair(S,type,name,pValue,dtInfo,nRows);
            break;
          }
          case SSWRITE_VALUE_DTYPE_ML_2DMAT: {
            const void *pValue[2];
            pValue[0] = va_arg(ap, const void *);
            pValue[1] = va_arg(ap, const void *);
            nRows  = va_arg(ap, int_T);
            nCols  = va_arg(ap, int_T);
            dtInfo = va_arg(ap, int_T);
            ans    = ssWriteRTWNameValuePair(S,type,name,pValue,
                                             dtInfo,nRows,nCols);
            break;
          }
          default:
            {
                ssSetErrorStatus(S, _SimstructErrmsg("Simulink:SFunctions:SimStructInvalidRTWParamSetting"));
                ans = 0;
                goto EXIT_POINT;
            }
        }
        if (ans != 1) {
            goto EXIT_POINT;
        }
    }
    if ( (ans = ssWriteRTWStr(S, "}")) != 1 ) {
        goto EXIT_POINT;
    }

  EXIT_POINT:

    va_end(ap);
    return(ans);

} /* end ssWriteRTWParamSettings */



/* Function: ssWriteRTWWorkVect ===============================================
 * Abstract:
 *    Used in mdlRTW to create work vector records for S-functions:
 *
 *       if (!ssWriteRTWWorkVect(S, vectName, nNames,
 *
 *                            name, size,   (must have nNames of these pairs)
 *                                 :
 *                           ) ) {
 *           return;  (error reporting will be handled by SL)
 *       }
 *
 *       Notes:
 *         a) vectName must be either "RWork", "IWork" or "PWork"
 *         b) nNames is an int_T, name is a const char_T* and size is int_T, and
 *            there must be nNames number of [name, size] pairs passed to the
 *            function.
 *         b) intSize1+intSize2+ ... +intSizeN = ssGetNum<vectName>(S)
 *            Recall that you would have to set ssSetNum<vectName>(S)
 *            in one of the initialization functions (mdlInitializeSizes
 *            or mdlSetWorkVectorWidths).
 *
 */
int_T ssWriteRTWWorkVect(SimStruct    *S,
                         const char_T *vectName,
                         int_T        nNames,
                         ...)
{
    int_T     i;
    int_T     nElementsWritten  = 0;
    int_T     nElementsExpected = 0;
    int_T     ans               = 1; /* assume */
    char_T    strBuf[40]        = "\0";
    va_list   ap;
    va_start(ap, nNames);

    /* vectName must be RWork, IWork or PWork */
    if ( (strcmp(vectName,"RWork") != 0 &&
          strcmp(vectName,"IWork") != 0 &&
          strcmp(vectName,"PWork") != 0) ||
         (strlen(vectName) + sizeof("NumDefines") > sizeof(strBuf))) {
        ssSetErrorStatus(S, _SimstructErrmsg("Simulink:SFunctions:SimStructInvalidRTWWorkVectorName"));
        ans = 0;
        goto EXIT_POINT;
    }

    if (nNames <= 0) {
        ssSetErrorStatus(S, _SimstructErrmsg("Simulink:SFunctions:SimStructInvalidRTWWorkVectorNumArgs"));
        ans = 0;
        goto EXIT_POINT;
    }


    if (strcmp(vectName, "RWork") == 0) {
        nElementsExpected = ssGetNumRWork(S);
    } else if (strcmp(vectName, "IWork") == 0) {
        nElementsExpected = ssGetNumIWork(S);
    }  else if (strcmp(vectName, "PWork") == 0) {
        nElementsExpected = ssGetNumPWork(S);
    } else {
        ssSetErrorStatus(S, _SimstructErrmsg("Simulink:SFunctions:SimStructInvalidRTWWorkVectorArgument"));
        ans = 0;
        goto EXIT_POINT;
    }


    (void)sprintf(strBuf,"Num%sDefines",vectName);
    ans = ssWriteRTWNameValuePair(S,SSWRITE_VALUE_DTYPE_NUM,strBuf,
                                  &nNames, DTINFO(SS_INT32, 0));
    if (ans != 1) goto EXIT_POINT;

    for (i = 0; i < nNames; i++) {
        const char_T *name = va_arg(ap, const char_T *);
        int_T        width = va_arg(ap, int_T);

        (void)sprintf(strBuf, "%sDefine {",vectName);
        ans = ssWriteRTWStr(S,strBuf);
        if (ans != 1) goto EXIT_POINT;

        ans = ssWriteRTWNameValuePair(S,SSWRITE_VALUE_QSTR,"Name",name);
        if (ans != 1) goto EXIT_POINT;

        ans = ssWriteRTWNameValuePair(S,SSWRITE_VALUE_DTYPE_NUM,"Width",
                                      &width, DTINFO(SS_INT32, 0));
        if (ans != 1) goto EXIT_POINT;

        ans = ssWriteRTWNameValuePair(S,SSWRITE_VALUE_DTYPE_NUM,"StartIndex",
                                      &nElementsWritten, DTINFO(SS_INT32,0));
        if (ans != 1) goto EXIT_POINT;

        ans = ssWriteRTWStr(S, "}");
        if (ans != 1) goto EXIT_POINT;

        nElementsWritten += width;
    }

    if (nElementsWritten != nElementsExpected) {
        char_T errmsg[200];
        (void)sprintf(errmsg,
                      _SimstructErrmsgWithArg(
                          "Simulink:SFunctions:SimStructRTWWorkVectorSizeMismatch",
                          _StrArg(%s) _NumArg(%d) _StrArg(%s) _NumArg(%d)), 
                      vectName, 
                      nElementsWritten, 
                      vectName, 
                      nElementsExpected);
        _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
        ans = 0;
    }

  EXIT_POINT:
    va_end(ap);
    return(ans);

} /* end ssWriteRTWWorkVect */


/*===================================*
 * Fix for MrC optimization problems *
 *===================================*/

/*
 * MAC - Require local stack frames for routines placed in the function
 * pointer table to work around bugs in MrC compiler.
 */

#if defined(__MRC__)

#define REQUIRE_LOCAL_STACK_FRAME volatile int_T __requireLocalStackFrame = 0

static void __mdlInitializeSizes(SimStruct *S)
{
    REQUIRE_LOCAL_STACK_FRAME;    
    mdlInitializeSizes(S);
}

#if S_FUNCTION_LEVEL == 1
# if defined(MDL_GET_INPUT_PORT_WIDTH)
    static int_T __mdlGetInputPortWidth(SimStruct *S, int_T outputWidth)
    {
        REQUIRE_LOCAL_STACK_FRAME;
        return mdlGetInputPortWidth(S, outputWidth);
    }
#  endif

# if defined(MDL_GET_OUTPUT_PORT_WIDTH)
    static int_T __mdlGetOutputPortWidth(SimStruct *S, int_T inputWidth)
    {
        REQUIRE_LOCAL_STACK_FRAME;
        return mdlGetOutputPortWidth(S, inputWidth);
    }
# endif
#endif


#if S_FUNCTION_LEVEL == 2

#   if defined(MDL_INITIALIZE_PROPAGATION_PASS)
      static void __mdlInitializePropagationPass(SimStruct *S, PropagationPassType passType)
      {
          REQUIRE_LOCAL_STACK_FRAME;
          mdlInitializePropagationPass(S, passType);
      }
#   endif

# if defined(MDL_SET_INPUT_PORT_WIDTH)
    static void __mdlSetInputPortWidth(SimStruct *S, int_T port, int_T width)
    {
        REQUIRE_LOCAL_STACK_FRAME;
        mdlSetInputPortWidth(S, port, width);
    }
# endif

# if defined(MDL_SET_OUTPUT_PORT_WIDTH)
    static void __mdlSetOutputPortWidth(SimStruct *S, int_T port, int_T width)
    {
        REQUIRE_LOCAL_STACK_FRAME;
        mdlSetOutputPortWidth(S, port, width);
    }
# endif

# if defined(MDL_SET_INPUT_PORT_DIMENSION_INFO)
    static void __mdlSetInputPortDimensionInfo(SimStruct        *S,
                                               int_T            port,
                                               const DimsInfo_T *dimsInfo)
    {
        REQUIRE_LOCAL_STACK_FRAME;
        mdlSetInputPortDimensionInfo(S, port, dimsInfo);
    }
# endif

# if defined(MDL_SET_OUTPUT_PORT_DIMENSION_INFO)
    static void __mdlSetOutputPortDimensionInfo(SimStruct        *S,
                                                int_T            port,
                                                const DimsInfo_T *dimsInfo)
    {
        REQUIRE_LOCAL_STACK_FRAME;
        mdlSetOutputPortDimensionInfo(S, port, dimsInfo);
    }
# endif

# if defined(MDL_SET_DEFAULT_PORT_DIMENSION_INFO)
    static void __mdlSetDefaultPortDimensionInfo(SimStruct *S)
    {
        REQUIRE_LOCAL_STACK_FRAME;
        mdlSetDefaultPortDimensionInfo(S);
    }
# endif

# if defined(MDL_SET_INPUT_PORT_SYMBOLIC_DIMENSIONS)
    static void __mdlSetInputPortSymbolicDimensions(SimStruct  *S,
                                                    int_T      port,
                                                    SymbDimsId symbDimsId)
    {
        REQUIRE_LOCAL_STACK_FRAME;
        mdlSetInputPortSymbolicDimensions(S, port, symbDimsId);
    }
# endif

# if defined(MDL_SET_OUTPUT_PORT_SYMBOLIC_DIMENSIONS)
    static void __mdlSetOutputPortSymbolicDimensions(SimStruct  *S,
                                                     int_T      port,
                                                     SymbDimsId symbDimsId)
    {
        REQUIRE_LOCAL_STACK_FRAME;
        mdlSetOutputPortSymbolicDimensions(S, port, symbDimsId);
    }
# endif

# if defined(MDL_SET_INPUT_PORT_SAMPLE_TIME)
    static void __mdlSetInputPortSampleTime(SimStruct *S, int_T port,
                                            real_T sampleTime,
                                            real_T offsetTime)
    {
        REQUIRE_LOCAL_STACK_FRAME;
        mdlSetInputPortSampleTime(S, port, sampleTime, offsetTime);
    }
# endif

# if defined(MDL_SET_OUTPUT_PORT_SAMPLE_TIME)
    static void __mdlSetOutputPortSampleTime(SimStruct *S, int_T port,
                                             real_T sampleTime,
                                             real_T offsetTime)
    {
        REQUIRE_LOCAL_STACK_FRAME;
        mdlSetOutputPortSampleTime(S, port, sampleTime, offsetTime);
    }
# endif

#   if defined(MDL_SET_INPUT_PORT_DATA_TYPE)
      static void __mdlSetInputPortDataType(SimStruct *S, int_T port,
                                            DTypeId   inputPortDataType)
      {
          REQUIRE_LOCAL_STACK_FRAME;
          mdlSetInputPortDataType(S, port, inputPortDataType);
      }
#   endif

#   if defined(MDL_SET_OUTPUT_PORT_DATA_TYPE)
      static void __mdlSetOutputPortDataType(SimStruct *S, int_T port,
                                             DTypeId   outputPortDataType)
      {
          REQUIRE_LOCAL_STACK_FRAME;
          mdlSetOutputPortDataType(S, port, outputPortDataType);
      }
#   endif

#   if defined(MDL_SET_DEFAULT_PORT_DATA_TYPES)
      static void __mdlSetDefaultPortDataTypes(SimStruct *S)
      {
          REQUIRE_LOCAL_STACK_FRAME;
          mdlSetDefaultPortDataTypes(S);
      }
#   endif



#   if defined(MDL_SET_INPUT_PORT_COMPLEX_SIGNAL)
      static void __mdlSetInputPortComplexSignal(SimStruct *S, int_T port,
                                                 int_T iPortComplexSignal)
      {
          REQUIRE_LOCAL_STACK_FRAME;
          mdlSetInputPortComplexSignal(S, port, iPortComplexSignal);
      }
#   endif

#   if defined(MDL_SET_OUTPUT_PORT_COMPLEX_SIGNAL)
      static void __mdlSetOutputPortComplexSignal(SimStruct *S, int_T port,
                                                 int_T oPortComplexSignal)
      {
          REQUIRE_LOCAL_STACK_FRAME;
          mdlSetOutputPortComplexSignal(S, port, oPortComplexSignal);
      }
#   endif

#   if defined(MDL_SET_DEFAULT_PORT_COMPLEX_SIGNALS)
      static void __mdlSetDefaultPortComplexSignals(SimStruct *S)
      {
          REQUIRE_LOCAL_STACK_FRAME;
          mdlSetDefaultPortComplexSignals(S);
      }
#   endif


#   if defined(MDL_SET_INPUT_PORT_FRAME_DATA)
      static void __mdlSetInputPortFrameData(SimStruct *S, int_T port,
                                            Frame_T   iPortFrameData)
      {
          REQUIRE_LOCAL_STACK_FRAME;
          mdlSetInputPortFrameData(S, port, iPortFrameData);
      }
#   endif

#endif

static void __mdlInitializeSampleTimes(SimStruct *S)
{
    REQUIRE_LOCAL_STACK_FRAME;
    mdlInitializeSampleTimes(S);
}

#if defined(MDL_SET_WORK_WIDTHS)
static void __mdlSetWorkWidths(SimStruct *S)
{
    REQUIRE_LOCAL_STACK_FRAME;
    mdlSetWorkWidths(S);
}
#endif

#if S_FUNCTION_LEVEL == 2 && defined(MDL_RTW)
  static void __mdlRTW(SimStruct *S)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlRTW(S);
  }
#endif

#if S_FUNCTION_LEVEL == 1
  static void __mdlInitializeConditions(real_T *x0, SimStruct *S)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlInitializeConditions(x0, S);
  }
#elif defined(MDL_INITIALIZE_CONDITIONS)
  static void __mdlInitializeConditions(SimStruct *S)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlInitializeConditions(S);
  }
#endif

#if S_FUNCTION_LEVEL == 2 && defined(MDL_DATA_TRANSFER_READ_WRITE)
  static void __mdlDataTransferRead(SimStruct* S, uint32_T* dtIdx, void* data) {
      REQUIRE_LOCAL_STACK_FRAME;
      DataTransferRead(S, dtIdx, data);
  }
  static void __mdlDataTransferWrite(SimStruct* S, uint32_T* dtIdx,
                                     void* data) {
      REQUIRE_LOCAL_STACK_FRAME;
      DataTransferWrite(S, dtIdx, data);
  }
#endif

#if S_FUNCTION_LEVEL == 2 && defined(MDL_DATA_TRANSFER_INIT)
  static void __mdlDataTransferInitBuffers(SimStruct* S, uint32_T* dtIdx,
                                           void* data) {
      REQUIRE_LOCAL_STACK_FRAME;
      DataTransferInitBuffers(S, dtIdx, data);
  }
#endif

#if S_FUNCTION_LEVEL == 2 && defined(MDL_SETUP_RUNTIME_RESOURCES)
  static void __mdlSetupRuntimeResources(SimStruct *S)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlSetupRuntimeResources(S);
  }
#endif

#if S_FUNCTION_LEVEL == 2 && defined(MDL_CLEANUP_RUNTIME_RESOURCES)
  static void __mdlCleanupRuntimeResources(SimStruct *S)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlCleanupRuntimeResources(S);
  }
#endif

#if S_FUNCTION_LEVEL == 2 && defined(MDL_START)
  static void __mdlStart(SimStruct *S)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlStart(S);
  }
#endif

#if defined(RTW_GENERATED_ENABLE) && defined(MDL_ENABLE)
# error MDL_ENABLE and RTW_GENERATED_ENABLE can not be defined simultaneously
#endif

#if S_FUNCTION_LEVEL == 2 && \
 (defined(RTW_GENERATED_ENABLE) || defined(MDL_ENABLE))
  static void __mdlEnable(SimStruct *S)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlEnable(S);
  }
#endif

#if defined(RTW_GENERATED_DISABLE) && defined(MDL_DISABLE)
# error MDL_DISABLE and RTW_GENERATED_DISABLE can not be defined simultaneously
#endif

#if S_FUNCTION_LEVEL == 2 && \
 (defined(RTW_GENERATED_DISABLE) || defined(MDL_DISABLE))
  static void __mdlDisable(SimStruct *S)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlDisable(S);
  }
#endif

#if defined(MDL_CHECK_PARAMETERS)
  static void __mdlCheckParameters(SimStruct *S)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlCheckParameters(S);
  }
#endif

#if S_FUNCTION_LEVEL == 2 && defined(MDL_PROCESS_PARAMETERS)
  static void __mdlProcessParameters(SimStruct *S)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlProcessParameters(S);
  }
#endif

#if S_FUNCTION_LEVEL == 2 && defined(MDL_INIT_SYSTEM_MATRICES)
  static void __mdlInitSystemMatrices(SimStruct *S)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlInitSystemMatrices(S);
  }
#endif

#if S_FUNCTION_LEVEL == 2 && defined(MDL_SIM_STATUS_CHANGE)
  static void __mdlSimStatusChange(SimStruct *S, ssSimStatusChangeType simStatus)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlSimStatusChange(S, simeStatus);
  }
#endif


#if defined(MDL_EXT_NODE_EXEC)
  static void __mdlExtModeExec(SimStruct *S)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlExtModeExec(S);
  }
#endif

static void __mdlGetTimeOfNextVarHit(SimStruct *S)
{
    REQUIRE_LOCAL_STACK_FRAME;
    mdlGetTimeOfNextVarHit(S);
}

#if S_FUNCTION_LEVEL == 1
  static void __mdlOutputs(real_T *y, real_T *x, real_T *u, SimStruct *S,
                           int_T tid)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlOutputs(y, x, u, S, tid);
  }
#elif !defined(S_FUNCTION_EXPORTS_FUNCTION_CALLS) || defined(MODELREF_EXPORTS_FUNCTION_CALLS)
  static void __mdlOutputs(SimStruct *S, int_T tid)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlOutputs(S, tid);
  }
#endif

#if S_FUNCTION_LEVEL == 1
  static void __mdlUpdate(real_T *x, real_T *u, SimStruct *S, int_T tid)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlUpdate(x, u, S, tid);
  }
#elif defined(MDL_UPDATE)
  static void __mdlUpdate(SimStruct *S, int_T tid)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlUpdate(S, tid);
  }
#endif

#if S_FUNCTION_LEVEL == 1
  static void __mdlDerivatives(real_T *dx, real_T *x, real_T *u, SimStruct *S,
                               int_T tid)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlDerivatives(dx, x, u, S, tid);
  }
#elif defined(MDL_DERIVATIVES)
  static void __mdlDerivatives(SimStruct *S)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlDerivatives(S);
  }
#endif

#if defined(MDL_JACOBIAN)
  static void __mdlJacobian(SimStruct *S)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlJacobian(S);
  }
#endif

#if defined(MDL_JACOBIANIRJC)
  static void __mdlJacobianIrJc(SimStruct *S)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlJacobianIrJc(S);
  }
#endif

#if defined(MDL_PROJECTION)
  static void __mdlProjection(SimStruct *S)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlProjection(S);
  }
#endif

#if defined(MDL_MASSMATRIX)
  static void __mdlMassMatrix(SimStruct *S)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlMassMatrix(S);
  }
#endif

#if defined(MDL_FORCINGFUNCTION)
  static void __mdlForcingFunction(SimStruct *S)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlForcingFunction(S);
  }
#endif

#if defined(MDL_CONSTRAINTS)
  static void __mdlConstraints(SimStruct *S)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlConstraints(S);
  }
#endif

#if defined(MDL_RTWCG)
  static void __mdlRTWCG(SimStruct *S, RTWCGInterface *iObj)
  {
      REQUIRE_LOCAL_STACK_FRAME;
      mdlRTWCG(S,iObj);
  }
#endif

static void __mdlZeroCrossings(SimStruct *S)
{
    REQUIRE_LOCAL_STACK_FRAME;
    mdlZeroCrossings(S);
}

static void __mdlTerminate(SimStruct *S)
{
    REQUIRE_LOCAL_STACK_FRAME;
    mdlTerminate(S);
}

#else

#if !defined(ADA_S_FUNCTION)
/* Ada S-Function's mdlInitializeSizes is called from __mdlInitializeSizes */
# define __mdlInitializeSizes          mdlInitializeSizes
#endif

#if S_FUNCTION_LEVEL == 1

# if defined(MDL_GET_INPUT_PORT_WIDTH)
#  define __mdlGetInputPortWidth      mdlGetInputPortWidth
# endif

# if defined(MDL_GET_OUTPUT_PORT_WIDTH)
#  define __mdlGetOutputPortWidth     mdlGetOutputPortWidth
# endif

#else /* level 2 */

# if defined(MDL_INITIALIZE_PROPAGATION_PASS)
#  define __mdlInitializePropagationPass   mdlInitializePropagationPass
# endif

# if defined(MDL_SET_INPUT_PORT_WIDTH)
#  define __mdlSetInputPortWidth      mdlSetInputPortWidth
# endif

# if defined(MDL_SET_OUTPUT_PORT_WIDTH)
#  define __mdlSetOutputPortWidth     mdlSetOutputPortWidth
# endif

# if defined(MDL_SET_INPUT_PORT_DIMENSION_INFO)
#  define __mdlSetInputPortDimensionInfo      mdlSetInputPortDimensionInfo
# endif

# if defined(MDL_SET_OUTPUT_PORT_DIMENSION_INFO)
#  define __mdlSetOutputPortDimensionInfo     mdlSetOutputPortDimensionInfo
# endif

# if defined(MDL_SET_DEFAULT_PORT_DIMENSION_INFO)
#  define __mdlSetDefaultPortDimensionInfo     mdlSetDefaultPortDimensionInfo
# endif

# if defined(MDL_SET_INPUT_PORT_SYMBOLIC_DIMENSIONS)
#  define __mdlSetInputPortSymbolicDimensions mdlSetInputPortSymbolicDimensions
# endif

# if defined(MDL_SET_OUTPUT_PORT_SYMBOLIC_DIMENSIONS)
#  define __mdlSetOutputPortSymbolicDimensions  mdlSetOutputPortSymbolicDimensions
# endif

# if defined(MDL_SET_INPUT_PORT_SAMPLE_TIME)
#  define __mdlSetInputPortSampleTime   mdlSetInputPortSampleTime
# endif

# if defined(MDL_SET_OUTPUT_PORT_SAMPLE_TIME)
#  define __mdlSetOutputPortSampleTime  mdlSetOutputPortSampleTime
# endif



# if defined(MDL_SET_INPUT_PORT_DATA_TYPE)
#  define __mdlSetInputPortDataType   mdlSetInputPortDataType
# endif

# if defined(MDL_SET_OUTPUT_PORT_DATA_TYPE)
#  define __mdlSetOutputPortDataType  mdlSetOutputPortDataType
# endif

# if defined(MDL_SET_DEFAULT_PORT_DATA_TYPES)
#  define __mdlSetDefaultPortDataTypes  mdlSetDefaultPortDataTypes
# endif


# if defined(MDL_SET_INPUT_PORT_COMPLEX_SIGNAL)
#  define __mdlSetInputPortComplexSignal   mdlSetInputPortComplexSignal
# endif

# if defined(MDL_SET_OUTPUT_PORT_COMPLEX_SIGNAL)
#  define __mdlSetOutputPortComplexSignal  mdlSetOutputPortComplexSignal
# endif

# if defined(MDL_SET_DEFAULT_PORT_COMPLEX_SIGNALS)
#  define __mdlSetDefaultPortComplexSignals  mdlSetDefaultPortComplexSignals
# endif


# if defined(MDL_SET_INPUT_PORT_FRAME_DATA)
#  define __mdlSetInputPortFrameData   mdlSetInputPortFrameData
# endif

#endif

#define __mdlInitializeSampleTimes    mdlInitializeSampleTimes

#if !defined(ADA_S_FUNCTION)
/* Ada S-Function's mdlSetWorkWidths is called from __mdlSetWorkWidths */
#define __mdlSetWorkWidths            mdlSetWorkWidths
#endif

#if S_FUNCTION_LEVEL == 2 && defined(MDL_RTW)
# define __mdlRTW                     mdlRTW
#endif

#if S_FUNCTION_LEVEL == 1 || defined(MDL_INITIALIZE_CONDITIONS)
# define __mdlInitializeConditions    mdlInitializeConditions
#endif

#if S_FUNCTION_LEVEL == 2 && defined(MDL_START)
# define __mdlStart                   mdlStart
#endif

#if S_FUNCTION_LEVEL == 2 && defined(MDL_DATA_TRANSFER_READ_WRITE)
# define __mdlDataTransferRead       DataTransferRead
# define __mdlDataTransferWrite       DataTransferWrite
#endif

#if S_FUNCTION_LEVEL == 2 && defined(MDL_DATA_TRANSFER_INIT)
# define __mdlDataTransferInitBuffers       DataTransferInitBuffers
#endif

#if S_FUNCTION_LEVEL == 2 && defined(MDL_SETUP_RUNTIME_RESOURCES)
# define __mdlSetupRuntimeResources       mdlSetupRuntimeResources
#endif

#if S_FUNCTION_LEVEL == 2 && defined(MDL_CLEANUP_RUNTIME_RESOURCES)
# define __mdlCleanupRuntimeResources      mdlCleanupRuntimeResources
#endif


#if S_FUNCTION_LEVEL == 2 && \
 (defined(RTW_GENERATED_ENABLE) || defined(MDL_ENABLE))
# define __mdlEnable                  mdlEnable
#endif

#if S_FUNCTION_LEVEL == 2 && \
 (defined(RTW_GENERATED_DISABLE) || defined(MDL_DISABLE))
# define __mdlDisable                  mdlDisable
#endif

#if defined(MDL_CHECK_PARAMETERS)
# define __mdlCheckParameters         mdlCheckParameters
#endif

#if S_FUNCTION_LEVEL == 2 && defined(MDL_PROCESS_PARAMETERS)
# define __mdlProcessParameters       mdlProcessParameters
#endif

#if defined(MDL_INIT_SYSTEM_MATRICES)
# define __mdlInitSystemMatrices      mdlInitSystemMatrices
#endif

#if defined(MDL_EXT_MODE_EXEC)
# define __mdlExtModeExec             mdlExtModeExec
#endif

#define __mdlGetTimeOfNextVarHit      mdlGetTimeOfNextVarHit

#define __mdlOutputs                  mdlOutputs

#if S_FUNCTION_LEVEL == 1 || defined(MDL_UPDATE)
# define __mdlUpdate                  mdlUpdate
#endif

#if S_FUNCTION_LEVEL == 1 || defined(MDL_DERIVATIVES)
# define __mdlDerivatives             mdlDerivatives
#endif

#if defined(MDL_JACOBIAN)
# define __mdlJacobian                mdlJacobian
#endif

#if defined(MDL_JACOBIANIRJC)
# define __mdlJacobianIrJc            mdlJacobianIrJc
#endif

#if defined(MDL_PROJECTION)
# define __mdlProjection              mdlProjection
#endif

#if defined(MDL_MASSMATRIX)
# define __mdlMassMatrix              mdlMassMatrix
#endif

#if defined(MDL_FORCINGFUNCTION)
# define __mdlForcingFunction         mdlForcingFunction
#endif

#if defined(MDL_CONSTRAINTS)
# define __mdlConstraints             mdlConstraints
#endif

#if defined(MDL_RTWCG)
# define __mdlRTWCG                   mdlRTWCG
#endif

#if defined(MDL_OPERATING_POINT)
# define __mdlGetOperatingPoint       mdlGetOperatingPoint
# define __mdlSetOperatingPoint       mdlSetOperatingPoint
#endif

#if defined(MDL_SIM_STATE)
# define __mdlGetSimState             mdlGetSimState
# define __mdlSetSimState             mdlSetSimState
#if defined(MDL_OPERATING_POINT)
#error "MDL_OPERATING_POINT and MDL_SIM_STATE may not both be defined"
#endif
#endif

#if S_FUNCTION_LEVEL == 2 && defined(MDL_SIM_STATUS_CHANGE)
# define __mdlSimStatusChange         mdlSimStatusChange
#endif

#define __mdlZeroCrossings            mdlZeroCrossings

#if !defined(ADA_S_FUNCTION)
/* Ada S-Function's mdlTerminate is called from __mdlTerminate */
#define __mdlTerminate                mdlTerminate
#endif

#endif /* defined __MRC__ */



/* Function: _ProcessMexSfunctionCmdLineCall ==================================
 * Abstract:
 *      Process a MEX S-function call which was issued at the MATLAB
 *      command line.
 *
 *      The only valid command is the sizes initialization:
 *
 *      [sizes,x0,str,ts,xts]=sfunc([],[],[],0)
 *
 *      Third parameter U is required if we have dynamically sized
 *      vector(s).
 */
static void _ProcessMexSfunctionCmdLineCall
(
 int_T         nlhs,
 mxArray       *plhs[],
 int_T         nrhs,
 const mxArray *prhs[]
)
{
#   if defined(ADA_S_FUNCTION)
    int_T     tempSimStruct = 0; /* assume */
#   endif
    SimStruct *S;
    int_T     flag;
    real_T    flagDbl;
    real_T    *dptr;

    /************************************************
     * Verify arguments aren't outside their limits *
     ************************************************/

    if (nrhs < 4) {
        _ssFatalError_2(S, _mxStrArg("Simulink:SFunctions:SimStructTooFewRHSArguments"), _mxStrArg(_sfcnName), _mxNumArg(4));
    }

    if (nlhs > 1) {
        _ssFatalError_1(S, _mxStrArg("Simulink:SFunctions:SimStructTooManyLHSArguments"), _mxStrArg(_sfcnName));
    }


    /*******************************
     * Get flag and verify it is 0 *
     *******************************/

    if (_GetNumEl(prhs[_RHS_FLAG]) != 1 ||
        mxIsComplex(prhs[_RHS_FLAG]) || !mxIsNumeric(prhs[_RHS_FLAG])) {
        _ssFatalError_1(S, _mxStrArg("Simulink:SFunctions:SimStructForthRHSArgIsNotInteger"), _mxStrArg(_sfcnName));
    }

    flagDbl = *(real_T*)mxGetPr(prhs[_RHS_FLAG]);
    flag = (int_T) flagDbl;

    if ((real_T)flag != flagDbl) {
        _ssFatalError_1(S, _mxStrArg("Simulink:SFunctions:SimStructForthRHSArgIsNotInteger"), _mxStrArg(_sfcnName));
    }

    if (flag != 0) {
        _ssFatalError_1(S, _mxStrArg("Simulink:SFunctions:SimStructInvalidFlagPassedToSfcn"), _mxStrArg(_sfcnName));
    }



    /*******************************************
     * Get SimStruct or create a temporary one *
     *******************************************/
    {
        size_t   m   = mxGetM(prhs[_RHS_X]);
        real_T   *pr = ((m == sizeof(SimStruct *)/sizeof(int_T) + 1)?
                      (real_T*)mxGetPr(prhs[_RHS_X]): (real_T*)NULL);
        /*
         * If number of rows is equal to a pointer split across int_T's,
         * then the SimStruct pointer is encoded in the X argument.
         * The pointer comes first. The version of the passed SimStruct comes
         * second.
         */
        if (pr != NULL && pr[m-1] == SIMSTRUCT_VERSION_LEVEL2) {
            int_T       intSptr[sizeof(SimStruct*)/sizeof(int_T)];
            size_t      i = 0;
            for (i = 0; i < sizeof(intSptr)/sizeof(*intSptr); ++i) {
                intSptr[i] = (int_T) pr[i];
            }
            memcpy(&S, intSptr, sizeof(intSptr));

            /*
             * Verify that S_FUNCTION_NAME and name entered in the
             * S-function dialog box match
             */
#           if S_FUNCTION_LEVEL == 2
            {
#           else
            if (ssGetSimMode(S) == SS_SIMMODE_RTWGEN) {
#           endif
                const char_T *sfcnName = _QUOTE(S_FUNCTION_NAME);

                if (strcmp(sfcnName,"simulink_only_sfcn") != 0 &&
                    strcmp(ssGetModelName(S),sfcnName) != 0) {
                    _ssFatalError_2(S, _mxStrArg("Simulink:SFunctions:SimStructSFunctionNameMismatch"), _mxStrArg(sfcnName), _mxStrArg(ssGetModelName(S)));
                }
            }

            /* Since this is Simulink calling us, load function pointers. */

            ssSetmdlInitializeSizes(S,__mdlInitializeSizes);

#           if S_FUNCTION_LEVEL==1 && defined(MDL_GET_INPUT_PORT_WIDTH)
                ssSetmdlGetInputPortWidthLevel1(S,
                    (mdlGetInputPortWidthLevel1Fcn)__mdlGetInputPortWidth);
#           endif

#           if S_FUNCTION_LEVEL==1 && defined(MDL_GET_OUTPUT_PORT_WIDTH)
                ssSetmdlGetOutputPortWidthLevel1(S,
                    (mdlGetOutputPortWidthLevel1Fcn)__mdlGetOutputPortWidth);
#           endif

#           if S_FUNCTION_LEVEL==2 && defined(MDL_SET_INPUT_PORT_WIDTH)
                ssSetmdlSetInputPortWidth(S,__mdlSetInputPortWidth);
#           endif

#           if S_FUNCTION_LEVEL==2 && defined(MDL_SET_OUTPUT_PORT_WIDTH)
                ssSetmdlSetOutputPortWidth(S,__mdlSetOutputPortWidth);
#           endif


#           if S_FUNCTION_LEVEL==2 && defined(MDL_SET_INPUT_PORT_DIMENSION_INFO)
             ssSetmdlSetInputPortDimensions(S,__mdlSetInputPortDimensionInfo);
#           endif

#           if S_FUNCTION_LEVEL==2 && defined(MDL_SET_OUTPUT_PORT_DIMENSION_INFO)
             ssSetmdlSetOutputPortDimensions(S,__mdlSetOutputPortDimensionInfo);
#           endif

#           if S_FUNCTION_LEVEL==2 && defined(MDL_SET_DEFAULT_PORT_DIMENSION_INFO)
             ssSetmdlSetDefaultPortDimensions(S,__mdlSetDefaultPortDimensionInfo);
#           endif

#           if S_FUNCTION_LEVEL==2 && defined(MDL_SET_INPUT_PORT_SYMBOLIC_DIMENSIONS)
             ssSetmdlSetInputPortSymbolicDimensions(S,__mdlSetInputPortSymbolicDimensions);
#           endif

#           if S_FUNCTION_LEVEL==2 && defined(MDL_SET_OUTPUT_PORT_SYMBOLIC_DIMENSIONS)
             ssSetmdlSetOutputPortSymbolicDimensions(S,__mdlSetOutputPortSymbolicDimensions);
#           endif

#           if S_FUNCTION_LEVEL==2 && defined(MDL_SET_INPUT_PORT_SAMPLE_TIME)
                ssSetmdlSetInputPortSampleTime(S,__mdlSetInputPortSampleTime);
#           endif

#           if S_FUNCTION_LEVEL==2 && defined(MDL_SET_OUTPUT_PORT_SAMPLE_TIME)
                ssSetmdlSetOutputPortSampleTime(S,__mdlSetOutputPortSampleTime);
#           endif

#           if  S_FUNCTION_LEVEL==2
#             if defined(MDL_INITIALIZE_PROPAGATION_PASS)
                  ssSetmdlInitializePropagationPass(S,__mdlInitializePropagationPass);
#             endif

#             if defined(MDL_SET_INPUT_PORT_DATA_TYPE)
                  ssSetmdlSetInputPortDataType(S,__mdlSetInputPortDataType);
#             endif

#             if defined(MDL_SET_OUTPUT_PORT_DATA_TYPE)
                  ssSetmdlSetOutputPortDataType(S,__mdlSetOutputPortDataType);
#             endif

#             if defined(MDL_SET_DEFAULT_PORT_DATA_TYPES)
                  ssSetmdlSetDefaultPortDataTypes(S,__mdlSetDefaultPortDataTypes);
#             endif


#             if defined(MDL_SET_INPUT_PORT_COMPLEX_SIGNAL)
                  ssSetmdlSetInputPortComplexSignal(S,
                                              __mdlSetInputPortComplexSignal);
#             endif

#             if defined(MDL_SET_OUTPUT_PORT_COMPLEX_SIGNAL)
                  ssSetmdlSetOutputPortComplexSignal(S,
                                              __mdlSetOutputPortComplexSignal);
#             endif

#             if defined(MDL_SET_DEFAULT_PORT_COMPLEX_SIGNALS)
                  ssSetmdlSetDefaultPortComplexSignals(S,
                                              __mdlSetDefaultPortComplexSignals);
#             endif


#             if defined(MDL_SET_INPUT_PORT_FRAME_DATA)
                  ssSetmdlSetInputPortFrameData(S,__mdlSetInputPortFrameData);
#             endif

#           endif

#           if defined(MDL_SET_WORK_WIDTHS)
                ssSetmdlSetWorkWidths(S,__mdlSetWorkWidths);
#           endif
#           if S_FUNCTION_LEVEL==2 && defined(MDL_RTW)
                ssSetmdlRTW(S,__mdlRTW);
#           endif

            ssSetmdlInitializeSampleTimes(S,__mdlInitializeSampleTimes);

#           if S_FUNCTION_LEVEL==1
              ssSetmdlInitializeConditionsLevel1(S,
                (mdlInitializeConditionsLevel1Fcn)__mdlInitializeConditions);
#           elif defined(MDL_INITIALIZE_CONDITIONS)
              ssSetmdlInitializeConditions(S,__mdlInitializeConditions);
#           endif

#           if S_FUNCTION_LEVEL==2 && defined(MDL_DATA_TRANSFER_READ_WRITE)
              ssSetmdlDataTransferRead(S,__mdlDataTransferRead);
              ssSetmdlDataTransferWrite(S,__mdlDataTransferWrite);
#           endif

#           if S_FUNCTION_LEVEL==2 && defined(MDL_DATA_TRANSFER_INIT)
              ssSetmdlDataTransferInitBuffers(S,__mdlDataTransferInitBuffers);
#           endif

#           if S_FUNCTION_LEVEL==2 && defined(MDL_SETUP_RUNTIME_RESOURCES)
              ssSetmdlSetupRuntimeResources(S,__mdlSetupRuntimeResources);
#           endif

#           if S_FUNCTION_LEVEL==2 && defined(MDL_CLEANUP_RUNTIME_RESOURCES)
              ssSetmdlCleanupRuntimeResources(S,__mdlCleanupRuntimeResources);
#           endif

#           if S_FUNCTION_LEVEL==2 && defined(MDL_START)
              ssSetmdlStart(S,__mdlStart);
#           endif

#           if defined(MDL_INIT_SYSTEM_MATRICES)
              ssSetmdlInitSystemMatrices(S,__mdlInitSystemMatrices);
#           endif

#           if S_FUNCTION_LEVEL==2 && defined(RTW_GENERATED_ENABLE)
              ssSetRTWGeneratedEnable(S,__mdlEnable);
#           endif

#           if S_FUNCTION_LEVEL==2 && defined(RTW_GENERATED_DISABLE)
              ssSetRTWGeneratedDisable(S,__mdlDisable);
#           endif

#           if S_FUNCTION_LEVEL==2 && defined(MDL_ENABLE)
              ssSetmdlEnable(S,__mdlEnable);
#           endif

#           if S_FUNCTION_LEVEL==2 && defined(MDL_DISABLE)
              ssSetmdlDisable(S,__mdlDisable);
#           endif

#           if defined(MDL_CHECK_PARAMETERS)
                ssSetmdlCheckParameters(S,__mdlCheckParameters);
#           endif

#           if S_FUNCTION_LEVEL==2 && defined(MDL_PROCESS_PARAMETERS)
                ssSetmdlProcessParameters(S,__mdlProcessParameters);
#           endif

#           if S_FUNCTION_LEVEL == 2 && defined(MDL_SIM_STATUS_CHANGE)
              ssSetmdlSimStatusChange(S,__mdlSimStatusChange);
#           endif

#           if defined(MDL_EXT_MODE_EXEC)
                ssSetmdlExtModeExec(S,__mdlExtModeExec);
#           endif

            ssSetmdlGetTimeOfNextVarHit(S, __mdlGetTimeOfNextVarHit);

#           if S_FUNCTION_LEVEL==1
              ssSetmdlOutputsLevel1(S,(mdlOutputsLevel1Fcn)__mdlOutputs);
#           elif defined(S_FUNCTION_EXPORTS_FUNCTION_CALLS)
              _ssSetSFcnExportsFunctionCalls(S);
#             if defined(MODELREF_EXPORTS_FUNCTION_CALLS)
                ssSetmdlOutputs(S,__mdlOutputs);
#             endif
#           else
              ssSetmdlOutputs(S,__mdlOutputs);
#           endif

#           if S_FUNCTION_LEVEL==1
              ssSetmdlUpdateLevel1(S,(mdlUpdateLevel1Fcn)__mdlUpdate);
#           elif defined(MDL_UPDATE)
              ssSetmdlUpdate(S,__mdlUpdate);
#           endif

#           if S_FUNCTION_LEVEL==1
            ssSetmdlDerivativesLevel1(S,
              (mdlDerivativesLevel1Fcn)__mdlDerivatives);
#           elif defined(MDL_DERIVATIVES)
            ssSetmdlDerivatives(S,__mdlDerivatives);
#           endif

#           if defined(MDL_FORCINGFUNCTION)
              ssSetmdlForcingFunction(S,__mdlForcingFunction);
#           endif

#           if defined(MDL_MASSMATRIX)
              ssSetmdlMassMatrix(S,__mdlMassMatrix);
#           endif

#           if defined(MDL_JACOBIAN)
              ssSetmdlJacobian(S,__mdlJacobian);
#           endif

#           if defined(MDL_JACOBIANIRJC)
              ssSetmdlJacobianIrJc(S,__mdlJacobianIrJc);
#           endif

#           if defined(MDL_PROJECTION)
              ssSetmdlProjection(S,__mdlProjection);
#           endif

#           if defined(MDL_CONSTRAINTS)
              ssSetmdlConstraints(S,__mdlConstraints);
#           endif

#           if defined(MDL_RTWCG)
              ssSetmdlRTWCG(S,__mdlRTWCG);
#           endif

            ssSetmdlZeroCrossings(S, __mdlZeroCrossings);
            ssSetmdlTerminate(S,__mdlTerminate);

        } else {

#           if defined(ADA_S_FUNCTION)
            tempSimStruct = 1;
#           endif
            S = _CreateSimStruct(nrhs,prhs);
        }
    }

#   if S_FUNCTION_LEVEL == 1
       ssSetVersion(S, SIMSTRUCT_VERSION_LEVEL1);
#   else
       ssSetVersion(S, SIMSTRUCT_VERSION_LEVEL2);
#   endif

    ssSupportVarTsInMdlRef(S, true);

#if S_FUNCTION_LEVEL == 2 && defined(MDL_SIM_STATE)
    S->sizes.flags.simStateCompliance = USE_CUSTOM_SIM_STATE;
#endif

    /************************
     * Issue the sizes call *
     ************************/

    __mdlInitializeSizes(S);

    /***********************
     * return sizes vector *
     ***********************/
    {
#       define SETU(sizefield) \
        i = (int_T)((uint_T)((&sizefield(S) - (uint_T*)ssGetSizesPtr(S)))); \
        dptr[i] = (real_T)sizefield(S)

        int_T i;
        plhs[_LHS_RET] = mxCreateDoubleMatrix((int_T)(SIZES_LENGTH), 1, mxREAL);
        dptr = (real_T*) mxGetPr(plhs[_LHS_RET]);
        for (i=0; i<(int_T)SIZES_LENGTH; i++) {
            dptr[i] = (real_T)ssGetSizesPtr(S)[i];
        }

        ssSetOptions(S, (SS_OPTION_SUPPORTS_MULTITASKING |
                         ssGetOptions(S)));

        /* Fix the unsigned items: */
        SETU(ssGetChecksum0);
        SETU(ssGetChecksum1);
        SETU(ssGetChecksum2);
        SETU(ssGetChecksum3);
        SETU(ssGetOptions);
    }

# if defined(ADA_S_FUNCTION)
    /* We need to do additional clean up if this is an Ada S-Function. */
    if ( tempSimStruct == 1 ) {
        __adaSFcnDestroyUserData(S);
    }
# endif

} /* end _ProcessMexSfunctionCmdLineCall */



/* Function: mexFunction ======================================================
 * Abstract:
 *
 *      Simulink C MEX S-function entry point.
 *
 */
void mexFunction
(
 int_T   nlhs,               /* in     - number of left hand side arguments  */
 mxArray *plhs[],            /* in/out - left hand side arguments            */
 int_T   nrhs,               /* in     - number of right hand side arguments */
 const mxArray *prhs[]       /* in     - right hand side arguments           */
)
{
    char_T errmsg[200];

    /*
     * We can get called from either Simulink or the MATLAB command line.
     * When called from Simulink, nlhs is negative and the SimStruct is
     * passed in as a left hand side argument.  When called from MATLAB
     * command line, we must create a temporary SimStruct and try our
     * best to perform the requested function specified by the flag
     * parameter.
     */

#ifdef PROCESS_MEX_SFUNCTION_EVERY_CALL
        if(ProcessMexSfunctionEveryCall(nlhs,plhs,nrhs,prhs)) return;
#endif


    if (nlhs < 0) {
        int_T flag = (int_T)(*(real_T*)mxGetPr(prhs[_RHS_FLAG]));

        /*
         * no need for error checking because we are being called from
         * Simulink
         */

        SimStruct *S = (SimStruct *) plhs[_LHS_SS];

        switch(flag) {

#         if S_FUNCTION_LEVEL == 1
          case SS_CALL_MDL_GET_INPUT_PORT_WIDTH:
            if (ssGetmdlGetInputPortWidthLevel1(S) != NULL) {
                int_T outputPortWidth = ssGetMexApiInt1(S);
                int_T uWidth = sfcnGetInputPortWidthLevel1(S, outputPortWidth);
                  ssSetMexApiInt1(S,uWidth);
            } else {                
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlGetInputPortWidth");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;

          case SS_CALL_MDL_GET_OUTPUT_PORT_WIDTH:
            if (ssGetmdlGetOutputPortWidthLevel1(S) != NULL) {
                int_T inputPortWidth = ssGetMexApiInt1(S);
                int_T yWidth = sfcnGetOutputPortWidthLevel1(S, inputPortWidth);
                ssSetMexApiInt1(S,yWidth);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlGetOutputPortWidth");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

#         if S_FUNCTION_LEVEL == 2
          case SS_CALL_MDL_INITIALIZE_PROPAGATION_PASS:
            if (ssGetmdlInitializePropagationPass(S) != NULL) {
                PropagationPassType passType = (PropagationPassType) ssGetMexApiInt1(S);
                sfcnInitializePropagationPass(S,passType);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlInitializePropagationPass");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;

          case SS_CALL_MDL_SET_INPUT_PORT_WIDTH:
            if (ssGetmdlSetInputPortWidth(S) != NULL) {
                int_T port  = ssGetMexApiInt1(S);
                int_T width = ssGetMexApiInt2(S);
                sfcnSetInputPortWidth(S,port,width);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlSetInputPortWidth");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;

          case SS_CALL_MDL_SET_OUTPUT_PORT_WIDTH:
            if (ssGetmdlSetOutputPortWidth(S) != NULL) {
                int_T port  = ssGetMexApiInt1(S);
                int_T width = ssGetMexApiInt2(S);
                sfcnSetOutputPortWidth(S,port,width);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlSetOutputPortWidth");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;

          case SS_CALL_MDL_SET_INPUT_PORT_DIMENSION_INFO:
            if (ssGetmdlSetInputPortDimensions(S) != NULL) {
                int_T            port      = ssGetMexApiInt1(S);
                const DimsInfo_T *dimsInfo = (const DimsInfo_T *)
                                              ssGetMexApiVoidPtr1(S);
                sfcnSetInputPortDimensionInfo(S,port,dimsInfo);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlSetInputPortDimensionInfo");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;

          case SS_CALL_MDL_SET_OUTPUT_PORT_DIMENSION_INFO:
            if (ssGetmdlSetOutputPortDimensions(S) != NULL) {
                int_T            port      = ssGetMexApiInt1(S);
                const DimsInfo_T *dimsInfo = (const DimsInfo_T *)
                                              ssGetMexApiVoidPtr1(S);
                sfcnSetOutputPortDimensionInfo(S,port,dimsInfo);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlSetOutputPortDimensionInfo");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;

          case SS_CALL_MDL_SET_DEFAULT_PORT_DIMENSION_INFO:
            if (ssGetmdlSetDefaultPortDimensions(S) != NULL) {
                sfcnSetDefaultPortDimensionInfo(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlSetDefaultPortDimensionInfo");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
            
          case SS_CALL_MDL_SET_INPUT_PORT_SYMBOLIC_DIMENSIONS:
            if (ssGetmdlSetInputPortDimensions(S) != NULL) {
                int_T            port      = ssGetMexApiInt1(S);
                SymbDimsId       dimsId    = ssGetMexApiInt2(S);
                sfcnSetInputPortSymbolicDimensions(S,port,dimsId);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlSetInputPortSymbolicDimensions");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;

          case SS_CALL_MDL_SET_OUTPUT_PORT_SYMBOLIC_DIMENSIONS:
            if (ssGetmdlSetOutputPortDimensions(S) != NULL) {
                int_T            port      = ssGetMexApiInt1(S);
                SymbDimsId       dimsId    = ssGetMexApiInt2(S);
                sfcnSetOutputPortSymbolicDimensions(S,port,dimsId);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlSetOutputPortSymbolicDimensions");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;

          case SS_CALL_MDL_SET_INPUT_PORT_SAMPLE_TIME:
            if (ssGetmdlSetInputPortSampleTime(S) != NULL) {
                int_T  port        = ssGetMexApiInt1(S);
                real_T sampleTime  = ssGetMexApiReal1(S);
                real_T offsetTime  = ssGetMexApiReal2(S);
                sfcnSetInputPortSampleTime(S,port,sampleTime,offsetTime);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlSetInputPortSampleTime");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;

          case SS_CALL_MDL_SET_OUTPUT_PORT_SAMPLE_TIME:
            if (ssGetmdlSetOutputPortSampleTime(S) != NULL) {
                int_T  port        = ssGetMexApiInt1(S);
                real_T sampleTime  = ssGetMexApiReal1(S);
                real_T offsetTime  = ssGetMexApiReal2(S);
                sfcnSetOutputPortSampleTime(S,port,sampleTime,offsetTime);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlSetOutputPortSampleTime");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;

          case SS_CALL_MDL_SET_INPUT_PORT_DATA_TYPE:
            if (ssGetmdlSetInputPortDataType(S) != NULL) {
                int_T     port               = ssGetMexApiInt1(S);
                DTypeId inputPortDataType  = ssGetMexApiInt2(S);
                sfcnSetInputPortDataType(S,port,inputPortDataType);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlSetInputPortDataType");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;

          case SS_CALL_MDL_SET_OUTPUT_PORT_DATA_TYPE:
            if (ssGetmdlSetOutputPortDataType(S) != NULL) {
                int_T     port                = ssGetMexApiInt1(S);
                DTypeId outputPortDataType  = ssGetMexApiInt2(S);
                sfcnSetOutputPortDataType(S,port,outputPortDataType);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlSetOutputPortDataType");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;

          case SS_CALL_MDL_SET_DEFAULT_PORT_DATA_TYPES:
            if (ssGetmdlSetDefaultPortDataTypes(S) != NULL) {
                sfcnSetDefaultPortDataTypes(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlSetDefaultPortDataTypes");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;


          case SS_CALL_MDL_SET_INPUT_PORT_COMPLEX_SIGNAL:
            if (ssGetmdlSetInputPortComplexSignal(S) != NULL) {
                int_T port               = ssGetMexApiInt1(S);
                int_T iPortComplexSignal = (int_T)ssGetMexApiInt2(S);
                sfcnSetInputPortComplexSignal(S,port,iPortComplexSignal);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlSetInputPortComplexSignal");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;

          case SS_CALL_MDL_SET_OUTPUT_PORT_COMPLEX_SIGNAL:
            if (ssGetmdlSetOutputPortComplexSignal(S) != NULL) {
                int_T port                = ssGetMexApiInt1(S);
                int_T oPortComplexSignal  = (int_T)ssGetMexApiInt2(S);
                sfcnSetOutputPortComplexSignal(S,port,oPortComplexSignal);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlSetOutputPortComplexSignal");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;

          case SS_CALL_MDL_SET_DEFAULT_PORT_COMPLEX_SIGNALS:
            if (ssGetmdlSetDefaultPortComplexSignals(S) != NULL) {
                sfcnSetDefaultPortComplexSignals(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlSetDefaultPortComplexSignals");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;

          case SS_CALL_MDL_SET_INPUT_PORT_FRAME_DATA:
            if (ssGetmdlSetInputPortFrameData(S) != NULL) {
                int_T   port               = ssGetMexApiInt1(S);
                Frame_T inputPortFrameData = (Frame_T)ssGetMexApiInt2(S);
                sfcnSetInputPortFrameData(S,port,inputPortFrameData);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlSetInputPortFrameData");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;

#         endif

          case SS_CALL_MDL_INITIALIZE_SAMPLE_TIMES:
            mdlInitializeSampleTimes(S);
            break;

          case SS_CALL_MDL_SET_WORK_WIDTHS:
            if (ssGetmdlSetWorkWidths(S) != NULL) {
                sfcnSetWorkWidths(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlSetWorkWidths");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;

#         if S_FUNCTION_LEVEL == 2 && defined(MDL_RTW)
          case SS_CALL_MDL_RTW:
            if (ssGetmdlRTW(S) != NULL) {
                sfcnRTW(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlRTW");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

#         if S_FUNCTION_LEVEL == 1
          case SS_CALL_MDL_INITIALIZE_CONDITIONS:
            mdlInitializeConditions(ssGetX(S),S);
            break;
#         elif defined(MDL_INITIALIZE_CONDITIONS)
          case SS_CALL_MDL_INITIALIZE_CONDITIONS:
            if (ssGetmdlInitializeConditions(S) != NULL) {
                mdlInitializeConditions(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlInitializeConditions");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

#         if S_FUNCTION_LEVEL == 2 && defined(MDL_START)
          case SS_CALL_MDL_START:
            if (ssGetmdlStart(S) != NULL) {
                mdlStart(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlStart");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

#         if S_FUNCTION_LEVEL == 2 && defined(MDL_SETUP_RUNTIME_RESOURCES)
          case SS_CALL_MDL_SETUP_RUNTIME_RESOURCES:
            if (ssGetmdlSetupRuntimeResources(S) != NULL) {
                mdlSetupRuntimeResources(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlSetupRuntimeResources");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

#         if S_FUNCTION_LEVEL == 2 && defined(MDL_CLEANUP_RUNTIME_RESOURCES)
          case SS_CALL_MDL_CLEANUP_RUNTIME_RESOURCES:
            if (ssGetmdlCleanupRuntimeResources(S) != NULL) {
                mdlCleanupRuntimeResources(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlCleanupRuntimeResources");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

#         if S_FUNCTION_LEVEL == 2 && defined(MDL_INIT_SYSTEM_MATRICES)
	  case SS_CALL_MDL_INIT_SYSTEM_MATRICES:
            if (ssGetmdlInitSystemMatrices(S) != NULL) {
                mdlInitSystemMatrices(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlInitSystemMatrices");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

#         if S_FUNCTION_LEVEL == 2 && defined(RTW_GENERATED_ENABLE)
          case SS_CALL_RTW_GENERATED_ENABLE:
            if (ssGetRTWGeneratedEnable(S) != NULL) {
                mdlEnable(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlEnable");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

#         if S_FUNCTION_LEVEL == 2 && defined(RTW_GENERATED_DISABLE)
          case SS_CALL_RTW_GENERATED_DISABLE:
            if (ssGetRTWGeneratedDisable(S) != NULL) {
                mdlDisable(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlDisable");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

#         if S_FUNCTION_LEVEL == 2 && defined(MDL_ENABLE)
          case SS_CALL_MDL_ENABLE:
            if (ssGetmdlEnable(S) != NULL) {
                mdlEnable(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlEnable");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

#         if S_FUNCTION_LEVEL == 2 && defined(MDL_DISABLE)
          case SS_CALL_MDL_DISABLE:
            if (ssGetmdlDisable(S) != NULL) {
                mdlDisable(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlDisable");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

          case SS_CALL_MDL_CHECK_PARAMETERS:
            if (ssGetmdlCheckParameters(S) != NULL) {
                sfcnCheckParameters(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlCheckParameters");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;

#         if S_FUNCTION_LEVEL == 2 && defined(MDL_PROCESS_PARAMETERS)
          case SS_CALL_MDL_PROCESS_PARAMETERS:
            if (ssGetmdlProcessParameters(S) != NULL) {
                sfcnProcessParameters(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlProcessParameters");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

#         if S_FUNCTION_LEVEL == 2 && defined(MDL_EXT_MODE_EXEC)
          case SS_CALL_MDL_EXT_MODE_EXEC:
            if (ssGetmdlExtModeExec(S) != NULL) {
                sfcnExtModeExec(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlExtModeExec");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

          case SS_CALL_MDL_GET_TIME_OF_NEXT_VAR_HIT:
            mdlGetTimeOfNextVarHit(S);
            break;

          case SS_CALL_MDL_OUTPUTS:
#           if S_FUNCTION_LEVEL == 1
                mdlOutputs(ssGetY(S),ssGetX(S),ssLocalGetU(S),S,0);
#           elif !defined(S_FUNCTION_EXPORTS_FUNCTION_CALLS) || defined(MODELREF_EXPORTS_FUNCTION_CALLS)
                if(ssGetMexApiInt1(S) == CONSTANT_TID) {
                    mdlOutputs(S,CONSTANT_TID);
                } else {
                    mdlOutputs(S,0);
                }
#           else
                {
                    ssSetLocalErrorStatus(S, _SimstructErrmsg("SimStructUnexpectedMdlOutputsInExportedFcnCall"));
                }
#           endif
            break;

#         if S_FUNCTION_LEVEL == 1
          case SS_CALL_MDL_UPDATE:
            mdlUpdate(ssGetX(S),ssLocalGetU(S),S,0);
            break;
#         elif defined(MDL_UPDATE)
          case SS_CALL_MDL_UPDATE:
            if (ssGetmdlUpdate(S) != NULL) {
                mdlUpdate(S,0);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlUpdate");
                _ssCopyLocalErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

#         if S_FUNCTION_LEVEL == 1
          case SS_CALL_MDL_DERIVATIVES:
            mdlDerivatives(ssGetdX(S),ssGetX(S),ssLocalGetU(S),S,0);
            break;
#         elif defined(MDL_DERIVATIVES)
          case SS_CALL_MDL_DERIVATIVES:
            if (ssGetmdlDerivatives(S) != NULL) {
                mdlDerivatives(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlDerivatives");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

#         if S_FUNCTION_LEVEL == 2 && defined(MDL_JACOBIAN)
	  case SS_CALL_MDL_JACOBIAN:
            if (ssGetmdlJacobian(S) != NULL) {
                mdlJacobian(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlJacobian");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

#         if S_FUNCTION_LEVEL == 2 && defined(MDL_JACOBIANIRJC)
	  case SS_CALL_MDL_JACOBIANIRJC:
            if (ssGetmdlJacobianIrJc(S) != NULL) {
                mdlJacobianIrJc(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlJacobianIrJc");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

#         if S_FUNCTION_LEVEL == 2 && defined(MDL_PROJECTION)
	  case SS_CALL_MDL_PROJECTION:
            if (ssGetmdlProjection(S) != NULL) {
                mdlProjection(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlProjection");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

#         if S_FUNCTION_LEVEL == 2 && defined(MDL_MASSMATRIX)
	  case SS_CALL_MDL_MASSMATRIX:
            if (ssGetmdlMassMatrix(S) != NULL) {
                mdlMassMatrix(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlMassMatrix");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

#         if S_FUNCTION_LEVEL == 1
          case SS_CALL_MDL_FORCINGFUNCTION:
            mdlDerivatives(ssGetdX(S),ssGetX(S),ssLocalGetU(S),S,0);
            break;
#         elif defined(MDL_FORCINGFUNCTION)
	  case SS_CALL_MDL_FORCINGFUNCTION:
            if (ssGetmdlForcingFunction(S) != NULL) {
                mdlForcingFunction(S);
            } else {
                mdlDerivatives(S);
            }
            break;
#         endif

#         if S_FUNCTION_LEVEL == 2 && defined(MDL_CONSTRAINTS)
	  case SS_CALL_MDL_CONSTRAINTS:
            if (ssGetmdlConstraints(S) != NULL) {
                mdlConstraints(S);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlConstraints");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

#         if S_FUNCTION_LEVEL == 2 && defined(MDL_RTWCG)
	  case SS_CALL_MDL_RTWCG:
            if (ssGetmdlRTWCG(S) != NULL) {
                RTWCGInterface *iObj = (RTWCGInterface *) ssGetMexApiVoidPtr1(S);
                mdlRTWCG(S,iObj);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlRTWCG");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

#if S_FUNCTION_LEVEL == 2 && ( defined(MDL_SIM_STATE) || defined(MDL_OPERATING_POINT) )      
#if defined(MDL_SIM_STATE)
        case SS_CALL_MDL_GET_SIM_STATE:
            if (ssGetSimStateCompliance(S) != USE_CUSTOM_SIM_STATE) {
                (void)sprintf(errmsg,
                              _SimstructErrmsgWithArg(
                                  "Simulink:SFunctions:SimStructUnexpectedFunctionCall",
                                  _StrArg(%s)), "mdlGetSimState");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            } else {
                mxArray** retVal;
                retVal = (mxArray**) ssGetMexApiVoidPtr1(S);
                retVal[0] = __mdlGetSimState(S);
                /* Call into simulink to set the array scope to local */
                if ((S)->mdlInfo->genericFcn) {
                    (*(S)->mdlInfo->genericFcn)(
                        S,
                        GEN_FCN_SET_ARRAYSCOPE_TO_LOCAL,
                        0,
                        retVal[0]);
                }
                break;
            }
	  case SS_CALL_MDL_SET_SIM_STATE:
            if (ssGetSimStateCompliance(S) != USE_CUSTOM_SIM_STATE) {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg(
                                  "Simulink:SFunctions:SimStructUnexpectedFunctionCall",
                                  _StrArg(%s)), "mdlSetSimState");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            } else {
                const mxArray* retVal = (const mxArray*) ssGetMexApiVoidPtr1(S);
                __mdlSetSimState(S, retVal);
                break;
            }
#elif defined(MDL_OPERATING_POINT)
        case SS_CALL_MDL_GET_SIM_STATE:
            if (ssGetOperatingPointCompliance(S) != USE_CUSTOM_OPERATING_POINT) {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg(
                                  "Simulink:SFunctions:SimStructUnexpectedFunctionCall",
                                  _StrArg(%s)), "mdlGetOperatingPoint");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            } else {
                mxArray** retVal;
                retVal = (mxArray**) ssGetMexApiVoidPtr1(S);
                retVal[0] = __mdlGetOperatingPoint(S);
                /* Call into simulink to set the array scope to local */
                if ((S)->mdlInfo->genericFcn) {
                    (*(S)->mdlInfo->genericFcn)(
                        S,
                        GEN_FCN_SET_ARRAYSCOPE_TO_LOCAL,
                        0,
                        retVal[0]);
                }
                break;
            }
	  case SS_CALL_MDL_SET_SIM_STATE:
            if (ssGetOperatingPointCompliance(S) !=  USE_CUSTOM_OPERATING_POINT) {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg(
                                  "Simulink:SFunctions:SimStructUnexpectedFunctionCall",
                                  _StrArg(%s)), "mdlSetOperatingPoint");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            } else {
                const mxArray* retVal = (const mxArray*) ssGetMexApiVoidPtr1(S);
                __mdlSetOperatingPoint(S, retVal);
                break;
            }             
#endif
#         else
          /* Neither OperatingPoint nor SimState save/restore function are defined*/
	  case SS_CALL_MDL_GET_SIM_STATE:
            {
                ssSetErrorStatus(S, _SimstructErrmsg("Simulink:SFunctions:SimStructUnexpectedGetSimState"));
                break;
            }
	  case SS_CALL_MDL_SET_SIM_STATE:
            {
                ssSetErrorStatus(S, _SimstructErrmsg("Simulink:SFunctions:SimStructUnexpectedSetSimState"));
                break;
            }
#         endif

          case SS_CALL_MDL_ZERO_CROSSINGS:
            mdlZeroCrossings(S);
            break;

          case SS_CALL_MDL_TERMINATE:
            __mdlTerminate(S);
            break;

#         if S_FUNCTION_LEVEL == 2 && defined(MDL_SIM_STATUS_CHANGE)
          case SS_CALL_MDL_SIM_STATUS_CHANGE:
            if (ssGetmdlSimStatusChange(S) != NULL) {
                ssSimStatusChangeType simStatus = (ssSimStatusChangeType)ssGetMexApiInt1(S);
                __mdlSimStatusChange(S, simStatus);
            } else {
                (void)sprintf(errmsg, _SimstructErrmsgWithArg("Simulink:SFunctions:SimStructUnexpectedFunctionCall", _StrArg(%s)), "mdlSimStatusChange");
                _ssCopyErrorStatusToBuffer(S, errmsg, strlen(errmsg)+1);
            }
            break;
#         endif

          default:
            {
                ssSetErrorStatus(S, _SimstructErrmsg("Simulink:SFunctions:SimStructInvalidFlagInSimulinkC"));
                break;
            }            
        }

    } else {

#ifdef PROCESS_MEX_SFUNCTION_CMD_LINE_CALL
        if(ProcessMexSfunctionCmdLineCall(nlhs,plhs,nrhs,prhs)) {
            return;
        }
#endif
        _ProcessMexSfunctionCmdLineCall(nlhs,plhs,nrhs,prhs);
    }

} /* end mexFunction */


/*******************************************************************************/
/******************************* DEBUGGING HOOKS *******************************/
/*******************************************************************************/

#if !SS_NDEBUG
#ifdef UNIX
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#else
#pragma warning ( push )
#pragma warning ( disable: 4090 )
#endif
static void ReportIndexOutOfBoundsError(
    const char* file,
    int line,
    const SimStruct* S,
    int idxMin,
    int* idx,
    int idxMaxPlus1,
    const char* fcn)
{
    if (*idx < idxMin || *idx >= idxMaxPlus1) {
        if (ssGetErrorStatus((SimStruct*)S)==NULL) {  /* don't clobber an existing error */
            char errmsg[4096];
            (void)sprintf(
                errmsg, _SimstructErrmsgWithArg(
                    "Simulink:SFunctions:SimStructIndexOutOfBound",
                    _StrArg(%s) _NumArg(%d) _NumArg(%d) _StrArg(%s) _NumArg(%d) _NumArg(%d)),
                file, line, *idx, fcn, idxMin, idxMaxPlus1-1);
            
            _ssCopyErrorStatusToBuffer((SimStruct*)S, errmsg, strlen(errmsg)+1);
        }
        /* prevent invalid access, so that the program behaves predictably */
        *idx = *idx < 0 ? 0 : idxMaxPlus1-1;
    }
}
#ifdef UNIX
#pragma GCC diagnostic pop
#else
#pragma warning ( pop )
#endif

void _ssSetIWorkValueFcn(
    const char* file, int line, SimStruct* S, int iworkIdx, int_T iworkValue)
{
    ReportIndexOutOfBoundsError(file,
                                line,
                                S,
                                0,
                                &iworkIdx,
                                ssGetNumIWork(S),
                                "ssSetIWorkValue");
    S->work.iWork[iworkIdx] = iworkValue;
}


int_T _ssGetIWorkValueFcn(
    const char* file, int line, const SimStruct* S, int iworkIdx)
{
    ReportIndexOutOfBoundsError(file,
                                line,
                                S,
                                0,
                                &iworkIdx,
                                ssGetNumIWork(S),
                                "ssGetIWorkValue");
    return S->work.iWork[iworkIdx];
}

void _ssSetRWorkValueFcn(
    const char* file, int line, SimStruct* S, int rworkIdx, real_T rworkValue)
{
    ReportIndexOutOfBoundsError(file,
                                line,
                                S,
                                0,
                                &rworkIdx,
                                ssGetNumRWork(S),
                                "ssSetRWorkValue");
    S->work.rWork[rworkIdx] = rworkValue;
}

real_T _ssGetRWorkValueFcn(
    const char* file, int line, const SimStruct* S, int rworkIdx)
{
    ReportIndexOutOfBoundsError(file,
                                line,
                                S,
                                0,
                                &rworkIdx,
                                ssGetNumRWork(S),
                                "ssGetRWorkValue");
    return S->work.rWork[rworkIdx];
}

void _ssSetPWorkValueFcn(
    const char* file, int line, SimStruct* S, int pworkIdx, void* pworkValue)
{
    ReportIndexOutOfBoundsError(file,
                                line,
                                S,
                                0,
                                &pworkIdx,
                                ssGetNumPWork(S),
                                "ssSetPWorkValue");
    S->work.pWork[pworkIdx] = pworkValue;
}

void* _ssGetPWorkValueFcn(
    const char* file, int line, const SimStruct* S, int pworkIdx)
{
    ReportIndexOutOfBoundsError(file,
                                line,
                                S,
                                0,
                                &pworkIdx,
                                ssGetNumPWork(S),
                                "ssGetPWorkValue");
    return S->work.pWork[pworkIdx];
}


int_T _ssGetInputPortBufferDstPortFcn(
    const char* file, int line, const SimStruct* S, int port) {
    #if !SS_SFCN_LEVEL_1
    #if !SS_SL_INTERNAL
    ReportIndexOutOfBoundsError(file,
                                line,
                                S,
                                0,
                                &port,
                                ssGetNumInputPorts(S),
                                "ssGetInputPortBufferDstPort");
    #endif
    #endif
    return (S)->portInfo.inputs[(port)].bufferDstPort;
}

#endif
/*========*
 * Undefs *
 *========*/
#undef _mxStrArg
#undef _mxNumArg
#undef _ssFatalError_1
#undef _ssFatalError_2
#undef _ssFatalError_3
#undef _ssFatalError_4
#undef _ssWarning_1
#undef _SimstructErrmsg
#undef _SimstructErrmsgWithArg
#undef _StrArg
#undef _NumArg

/* EOF simulink.c */
                        
