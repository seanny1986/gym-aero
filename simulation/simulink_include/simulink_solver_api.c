/* Copyright 2010 The MathWorks, Inc. */
/*
 * API specific to Simulink solver that needs to be accessed in 
 * Rapid accelerator and Rsim
 */

#include "simulink_solver_api.h"


/* _ssSetStateAbsTol: Set the absolute tolerance for single state */

void _ssSetStateAbsTol(SimStruct *S,
                       const int_T idx,
                       const real_T value){
    /* -1 signifies that Simulink Engine/Solver will set the value
     * based on the config set setting. Just return.
     */
    if (value == -1){
        return;
    } 
    /* If positive and finite : set the value */
    else if ( (value >= 0) && ((value-value) == 0.0)) { 
        ( (S)->states.statesInfo2 )->absTolControl[idx] = SL_SOLVER_TOLERANCE_LOCAL;
        ( (S)->states.statesInfo2 )->absTol[idx] = value;
        return;
    }
    /* Incorrect value */ 
    else {
        ssSetErrorStatus(S, "Incorrect absolute tolerance value.");
    }
}


/* _ssGetAbsTol:  */

real_T* _ssGetAbsTol(SimStruct *S){
    int_T is; /* State Index */
    const int_T nCStates = ssGetNumContStates(S);
    for (is = 0; is < nCStates; ++is){
        ( (S)->states.statesInfo2 )->absTolControl[is] = SL_SOLVER_TOLERANCE_LOCAL;
    }

    return ( (S)->states.statesInfo2 )->absTol;
}

