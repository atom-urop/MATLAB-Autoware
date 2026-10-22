/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * Licensed under the 2-Clause BSD License.
 */

#define S_FUNCTION_NAME acados_solver_sfunction_atom_4ws_nmpc
#define S_FUNCTION_LEVEL 2

#define MDL_START

// acados
// #include "acados/utils/print.h"
#include "acados_c/sim_interface.h"
#include "acados_c/external_function_interface.h"

// example specific
#include "acados_solver_atom_4ws_nmpc.h"



#include "simstruc.h"

#define SAMPLINGTIME -1


  
  
  
  
  
  
  
  
  
  
  
  
  
  


typedef struct {
    atom_4ws_nmpc_solver_capsule *capsule;
    double* buffer;
} AcadosOcpData;


static void mdlInitializeSizes (SimStruct *S)
{
    // specify the number of continuous and discrete states
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    int N = 15;// Additional input for payload (only when enabled)

  

  

  

  

  

  

  

    // specify the number of input ports
    if ( !ssSetNumInputPorts(S, 9) )
        return;

    // specify the number of output ports
    if ( !ssSetNumOutputPorts(S, 4) )
        return;

    // specify dimension information for the input ports
    // lbx_0
    ssSetInputPortVectorDimension(S, 0, 9);
    // ubx_0
    ssSetInputPortVectorDimension(S, 1, 9);
    // parameter_traj
    ssSetInputPortVectorDimension(S, 2, 272);
    // y_ref_0
    ssSetInputPortVectorDimension(S, 3, 7);


    // y_ref
    ssSetInputPortVectorDimension(S, 4, 98);
    // y_ref_e
    ssSetInputPortVectorDimension(S, 5, 5);
    // x_init
    ssSetInputPortVectorDimension(S, 6, 144);
    // u_init
    ssSetInputPortVectorDimension(S, 7, 30);
    // pi_init
    ssSetInputPortVectorDimension(S, 8, 135);/* specify dimension information for the OUTPUT ports */
    ssSetOutputPortVectorDimension(S, 0, 2 );
    ssSetOutputPortVectorDimension(S, 1, 1 );
    ssSetOutputPortVectorDimension(S, 2, 1);
    ssSetOutputPortVectorDimension(S, 3, 1 );
    // specify the direct feedthrough status
    // should be set to 1 for all inputs used in mdlOutputs
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortDirectFeedThrough(S, 3, 1);
    ssSetInputPortDirectFeedThrough(S, 4, 1);
    ssSetInputPortDirectFeedThrough(S, 5, 1);
    ssSetInputPortDirectFeedThrough(S, 6, 1);
    ssSetInputPortDirectFeedThrough(S, 7, 1);
    ssSetInputPortDirectFeedThrough(S, 8, 1);


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
    AcadosOcpData *ocp_data = malloc(sizeof(*ocp_data));

    // capsule
    ocp_data->capsule = atom_4ws_nmpc_acados_create_capsule();
    atom_4ws_nmpc_acados_create(ocp_data->capsule);

    // buffer  

    

    // local buffer

    ocp_data->buffer = malloc(272 * sizeof(double));

    ssSetUserData(S, (void*)ocp_data);
}


static void mdlOutputs(SimStruct *S, int_T tid)
{
    AcadosOcpData *ocp_data = ssGetUserData(S);
    atom_4ws_nmpc_solver_capsule *capsule = ocp_data->capsule;
    double* buffer = ocp_data->buffer;

    ocp_nlp_config *nlp_config = atom_4ws_nmpc_acados_get_nlp_config(capsule);
    ocp_nlp_dims *nlp_dims = atom_4ws_nmpc_acados_get_nlp_dims(capsule);
    ocp_nlp_in *nlp_in = atom_4ws_nmpc_acados_get_nlp_in(capsule);
    ocp_nlp_out *nlp_out = atom_4ws_nmpc_acados_get_nlp_out(capsule);
    ocp_nlp_solver *nlp_solver = atom_4ws_nmpc_acados_get_nlp_solver(capsule);

    InputRealPtrsType in_sign;

    int N = 15;

    double tmp_double;
    int tmp_offset, tmp_int;

    /* go through inputs */
    // lbx_0
    in_sign = ssGetInputPortRealSignalPtrs(S, 0);
    for (int i = 0; i < 9; i++)
        buffer[i] = (double)(*in_sign[i]);

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", buffer);
    // ubx_0
    in_sign = ssGetInputPortRealSignalPtrs(S, 1);
    for (int i = 0; i < 9; i++)
        buffer[i] = (double)(*in_sign[i]);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", buffer);
    // parameter_traj
    in_sign = ssGetInputPortRealSignalPtrs(S, 2);
    // update value of parameters
    tmp_offset = 0;
    for (int stage = 0; stage <= N; stage++)
    {
        tmp_int = ocp_nlp_dims_get_from_attr(nlp_config, nlp_dims, nlp_out, stage, "p");
        for (int jj = 0; jj < tmp_int; jj++)
        {
            buffer[jj] = (double)(*in_sign[tmp_offset+jj]);
        }
        atom_4ws_nmpc_acados_update_params(capsule, stage, buffer, tmp_int);
        tmp_offset += tmp_int;
    }

  
    // y_ref_0
    in_sign = ssGetInputPortRealSignalPtrs(S, 3);

    for (int i = 0; i < 7; i++)
        buffer[i] = (double)(*in_sign[i]);

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "yref", (void *) buffer);


  
    // y_ref - for stages 1 to N-1
    in_sign = ssGetInputPortRealSignalPtrs(S, 4);

    for (int stage = 1; stage < N; stage++)
    {
        for (int jj = 0; jj < 7; jj++)
            buffer[jj] = (double)(*in_sign[(stage-1)*7+jj]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, stage, "yref", (void *) buffer);
    }

  
    // y_ref_e
    in_sign = ssGetInputPortRealSignalPtrs(S, 5);

    for (int i = 0; i < 5; i++)
        buffer[i] = (double)(*in_sign[i]);

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", (void *) buffer);

    int ignore_inits = 0;
    // ssPrintf("ignore_inits = %d\n", ignore_inits);

    if (ignore_inits == 0)
    {
        // x_init
        in_sign = ssGetInputPortRealSignalPtrs(S, 6);
        tmp_int = ocp_nlp_dims_get_total_from_attr(nlp_config, nlp_dims, nlp_out, "x");
        for (int jj = 0; jj < tmp_int; jj++)
            buffer[jj] = (double)(*in_sign[jj]);
        ocp_nlp_set_all(nlp_solver, nlp_in, nlp_out, "x", (void *) buffer);
        // u_init
        in_sign = ssGetInputPortRealSignalPtrs(S, 7);
        tmp_int = ocp_nlp_dims_get_total_from_attr(nlp_config, nlp_dims, nlp_out, "u");
        for (int jj = 0; jj < tmp_int; jj++)
            buffer[jj] = (double)(*in_sign[jj]);
        ocp_nlp_set_all(nlp_solver, nlp_in, nlp_out, "u", (void *) buffer);
        // pi_init
        in_sign = ssGetInputPortRealSignalPtrs(S, 8);
        tmp_int = ocp_nlp_dims_get_total_from_attr(nlp_config, nlp_dims, nlp_out, "pi");
        for (int jj = 0; jj < tmp_int; jj++)
            buffer[jj] = (double)(*in_sign[jj]);
        ocp_nlp_set_all(nlp_solver, nlp_in, nlp_out, "pi", (void *) buffer);
    }/* call solver */
    int acados_status = atom_4ws_nmpc_acados_solve(capsule);
    // get time
    ocp_nlp_get(nlp_solver, "time_tot", (void *) buffer);
    tmp_double = buffer[0];

    /* set outputs */
    double *out_ptr;
    out_ptr = ssGetOutputPortRealSignal(S, 0);
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", (void *) out_ptr);

  

  

  

  
    out_ptr = ssGetOutputPortRealSignal(S, 1);
    *out_ptr = (double) acados_status;
    out_ptr = ssGetOutputPortRealSignal(S, 2);
    out_ptr[0] = tmp_double;
    out_ptr = ssGetOutputPortRealSignal(S, 3);
    // get sqp iter
    ocp_nlp_get(nlp_solver, "sqp_iter", (void *) &tmp_int);
    *out_ptr = (double) tmp_int;

  

  

  

}

static void mdlTerminate(SimStruct *S)
{
    AcadosOcpData *ocp_data = ssGetUserData(S);
    atom_4ws_nmpc_solver_capsule *capsule = ocp_data->capsule;
    double* buffer = ocp_data->buffer;

    atom_4ws_nmpc_acados_free(capsule);
    atom_4ws_nmpc_acados_free_capsule(capsule);
    free(buffer);
    free(ocp_data);
}


#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
