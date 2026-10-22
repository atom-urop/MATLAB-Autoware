/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * Licensed under the 2-Clause BSD License.
 */

#define S_FUNCTION_NAME   acados_sim_solver_sfunction_atom_4ws_nmpc
#define S_FUNCTION_LEVEL  2

#define MDL_START

// acados
// #include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific
#include "atom_4ws_nmpc_model/atom_4ws_nmpc_model.h"
#include "acados_sim_solver_atom_4ws_nmpc.h"

#include "simstruc.h"

#define SAMPLINGTIME 0.12


typedef struct {
    atom_4ws_nmpc_sim_solver_capsule *capsule;
    double* buffer;
} AcadosSimData;


static void mdlInitializeSizes (SimStruct *S)
{
    // specify the number of continuous and discrete states
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

          

    // specify the number of input ports
    if ( !ssSetNumInputPorts(S, 3) )
        return;

    // specify the number of output ports
    if ( !ssSetNumOutputPorts(S, 1) )
        return;

    // specify dimension information for the input ports
    // x0
    ssSetInputPortVectorDimension(S, 0, 9);
    // u0
    ssSetInputPortVectorDimension(S, 1, 2);
    // parameters
    ssSetInputPortVectorDimension(S, 2, 17);

    // specify dimension information for the output ports
    ssSetOutputPortVectorDimension(S, 0, 9 ); // xnext

    // specify the direct feedthrough status
    // should be set to 1 for all inputs used in mdlOutputs
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);

    // one sample time
    ssSetNumSampleTimes(S, 1);
}


#if defined(MATLAB_MEX_FILE)

#define MDL_SET_INPUT_PORT_DIMENSION_INFO
#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO

static void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if ( !ssSetInputPortDimensionInfo(S, port, dimsInfo) )
         return;
}

static void mdlSetOutputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if ( !ssSetOutputPortDimensionInfo(S, port, dimsInfo) )
         return;
}

#endif /* MATLAB_MEX_FILE */


static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLINGTIME);
    ssSetOffsetTime(S, 0, 0.0);
}


static void mdlStart(SimStruct *S)
{
    AcadosSimData *sim_data = malloc(sizeof(*sim_data));

    // capsule
    sim_data->capsule = atom_4ws_nmpc_acados_sim_solver_create_capsule();
    atom_4ws_nmpc_acados_sim_create(sim_data->capsule);

    // local buffer
    
    sim_data->buffer = malloc(17 * sizeof(double));

    ssSetUserData(S, (void*) sim_data);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    AcadosSimData *sim_data = ssGetUserData(S);
    atom_4ws_nmpc_sim_solver_capsule *capsule = sim_data->capsule;
    double* buffer = sim_data->buffer;

    sim_config *acados_sim_config = atom_4ws_nmpc_acados_get_sim_config(capsule);
    sim_in *acados_sim_in = atom_4ws_nmpc_acados_get_sim_in(capsule);
    sim_out *acados_sim_out = atom_4ws_nmpc_acados_get_sim_out(capsule);
    void *acados_sim_dims = atom_4ws_nmpc_acados_get_sim_dims(capsule);
    // sim_opts * atom_4ws_nmpc_acados_get_sim_opts(capsule);
    // sim_solver * atom_4ws_nmpc_acados_get_sim_solver(capsule);

    InputRealPtrsType in_sign;

    /* go through inputs */
    // initial condition
    in_sign = ssGetInputPortRealSignalPtrs(S, 0);
    for (int i = 0; i < 9; i++)
        buffer[i] = (double)(*in_sign[i]);

    sim_in_set(acados_sim_config, acados_sim_dims,
               acados_sim_in, "x", buffer);


    // ssPrintf("\nin acados sim:\n");
    // for (int i = 0; i < 9; i++) ssPrintf("x0[%d] = %f\n", i, buffer[i]);
    //     ssPrintf("\n");


    // control input - u
    in_sign = ssGetInputPortRealSignalPtrs(S, 1);

    for (int i = 0; i < 2; i++)
        buffer[i] = (double)(*in_sign[i]);

    sim_in_set(acados_sim_config, acados_sim_dims,
               acados_sim_in, "u", buffer);



    // parameters
    in_sign = ssGetInputPortRealSignalPtrs(S, 2);

    for (int i = 0; i < 17; i++)
        buffer[i] = (double)(*in_sign[i]);

    // update value of parameters
    atom_4ws_nmpc_acados_sim_update_params(capsule, buffer, 17);


    /* call solver */
    int acados_status = atom_4ws_nmpc_acados_sim_solve(capsule);


    /* set outputs */
    double *out_x = ssGetOutputPortRealSignal(S, 0);

    // get simulated state
    sim_out_get(acados_sim_config, acados_sim_dims, acados_sim_out,
                "xn", (void *) out_x);

    // ssPrintf("\nacados sim solve: returned %d\n", acados_status);
    // for (int i = 0; i < 9; i++) ssPrintf("x_sim[%d] = %f\n", i, out_x[i]);
    //     ssPrintf("\n");

}


static void mdlTerminate(SimStruct *S)
{
    AcadosSimData *sim_data = ssGetUserData(S);
    atom_4ws_nmpc_sim_solver_capsule *capsule = sim_data->capsule;
    double* buffer = sim_data->buffer;

    atom_4ws_nmpc_acados_sim_free(capsule);
    atom_4ws_nmpc_acados_sim_solver_free_capsule(capsule);
    free(buffer);
    free(sim_data);
}


#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
