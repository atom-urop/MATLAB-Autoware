/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * Licensed under the 2-Clause BSD License.
 */


// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/sim_interface.h"
#include "acados_sim_solver_atom_4ws_nmpc.h"

#define NX     ATOM_4WS_NMPC_NX
#define NZ     ATOM_4WS_NMPC_NZ
#define NU     ATOM_4WS_NMPC_NU
#define NP     ATOM_4WS_NMPC_NP


int main()
{
    int status = 0;
    atom_4ws_nmpc_sim_solver_capsule *capsule = atom_4ws_nmpc_acados_sim_solver_create_capsule();
    status = atom_4ws_nmpc_acados_sim_create(capsule);

    if (status)
    {
        printf("acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    sim_config *acados_sim_config = atom_4ws_nmpc_acados_get_sim_config(capsule);
    sim_in *acados_sim_in = atom_4ws_nmpc_acados_get_sim_in(capsule);
    sim_out *acados_sim_out = atom_4ws_nmpc_acados_get_sim_out(capsule);
    void *acados_sim_dims = atom_4ws_nmpc_acados_get_sim_dims(capsule);

    // initial condition
    double x_current[NX];
    x_current[0] = 0.0;
    x_current[1] = 0.0;
    x_current[2] = 0.0;
    x_current[3] = 0.0;
    x_current[4] = 0.0;
    x_current[5] = 0.0;
    x_current[6] = 0.0;
    x_current[7] = 0.0;
    x_current[8] = 0.0;

  
    x_current[0] = 0;
    x_current[1] = 0;
    x_current[2] = 0;
    x_current[3] = 0;
    x_current[4] = 0;
    x_current[5] = 0;
    x_current[6] = 0;
    x_current[7] = 0;
    x_current[8] = 0;
    
  


    // initial value for control input
    double u0[NU];
    u0[0] = 0.0;
    u0[1] = 0.0;
    // set parameters
    double p[NP];
    p[0] = 0;
    p[1] = 1;
    p[2] = -1;
    p[3] = 0;
    p[4] = 0;
    p[5] = 0;
    p[6] = 0;
    p[7] = 1;
    p[8] = -1;
    p[9] = 5;
    p[10] = 5;
    p[11] = 4;
    p[12] = 4;
    p[13] = 0;
    p[14] = 0;
    p[15] = 0;
    p[16] = 0;

    atom_4ws_nmpc_acados_sim_update_params(capsule, p, NP);
  

  


    int n_sim_steps = 3;
    // solve ocp in loop
    for (int ii = 0; ii < n_sim_steps; ii++)
    {
        // set inputs
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "x", x_current);
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "u", u0);

        // solve
        status = atom_4ws_nmpc_acados_sim_solve(capsule);
        if (status != ACADOS_SUCCESS)
        {
            printf("acados_solve() failed with status %d.\n", status);
        }

        // get outputs
        sim_out_get(acados_sim_config, acados_sim_dims,
               acados_sim_out, "x", x_current);

    

        // print solution
        printf("\nx_current, %d\n", ii);
        for (int jj = 0; jj < NX; jj++)
        {
            printf("%e\n", x_current[jj]);
        }
    }

    printf("\nPerformed %d simulation steps with acados integrator successfully.\n\n", n_sim_steps);

    // free solver
    status = atom_4ws_nmpc_acados_sim_free(capsule);
    if (status) {
        printf("atom_4ws_nmpc_acados_sim_free() returned status %d. \n", status);
    }

    atom_4ws_nmpc_acados_sim_solver_free_capsule(capsule);

    return status;
}
