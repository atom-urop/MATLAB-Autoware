/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * Licensed under the 2-Clause BSD License.
 */

// system
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
// acados
#include "acados_solver_atom_4ws_nmpc.h"

// mex
#include "mex.h"



void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // C_ocp
    long long *ptr;
    const mxArray *C_ocp = prhs[0];

    // capsule
    ptr = (long long *) mxGetData( mxGetField( C_ocp, 0, "capsule" ) );
    atom_4ws_nmpc_solver_capsule *capsule = (atom_4ws_nmpc_solver_capsule *) ptr[0];

    // solve
    atom_4ws_nmpc_acados_solve(capsule);

}
