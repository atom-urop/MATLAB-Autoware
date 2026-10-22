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
#include <string.h> // memcpy

// acados
#include "acados_c/external_function_interface.h"
#include "acados_c/sim_interface.h"
#include "acados_c/external_function_interface.h"

#include "acados/sim/sim_common.h"
#include "acados/utils/external_function_generic.h"
#include "acados/utils/print.h"


// example specific
#include "atom_4ws_nmpc_model/atom_4ws_nmpc_model.h"
#include "acados_sim_solver_atom_4ws_nmpc.h"
// initial value of stagewise parameters
static const double p_init[] = {0,1,-1,0,0,0,0,1,-1,5,5,4,4,0,0,0,0,};

// ** solver data **

atom_4ws_nmpc_sim_solver_capsule * atom_4ws_nmpc_acados_sim_solver_create_capsule()
{
    void* capsule_mem = malloc(sizeof(atom_4ws_nmpc_sim_solver_capsule));
    atom_4ws_nmpc_sim_solver_capsule *capsule = (atom_4ws_nmpc_sim_solver_capsule *) capsule_mem;

    return capsule;
}


int atom_4ws_nmpc_acados_sim_solver_free_capsule(atom_4ws_nmpc_sim_solver_capsule * capsule)
{
    free(capsule);
    return 0;
}


int atom_4ws_nmpc_acados_sim_create(atom_4ws_nmpc_sim_solver_capsule * capsule)
{
    // initialize
    const int nx = ATOM_4WS_NMPC_NX;
    const int nu = ATOM_4WS_NMPC_NU;
    const int nz = ATOM_4WS_NMPC_NZ;
    const int np = ATOM_4WS_NMPC_NP;
    bool tmp_bool;

    double Tsim = 0.12;

    capsule->acados_sim_mem = NULL;

    external_function_opts ext_fun_opts;
    external_function_opts_set_to_default(&ext_fun_opts);
    ext_fun_opts.external_workspace = false;

    
    // explicit ode
    capsule->sim_expl_vde_forw = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi));
    capsule->sim_vde_adj_casadi = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi));
    capsule->sim_expl_ode_fun_casadi = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi));
    
        capsule->sim_expl_vde_forw_p = NULL;
    

    capsule->sim_expl_vde_forw->casadi_fun = &atom_4ws_nmpc_expl_vde_forw;
    capsule->sim_expl_vde_forw->casadi_n_in = &atom_4ws_nmpc_expl_vde_forw_n_in;
    capsule->sim_expl_vde_forw->casadi_n_out = &atom_4ws_nmpc_expl_vde_forw_n_out;
    capsule->sim_expl_vde_forw->casadi_sparsity_in = &atom_4ws_nmpc_expl_vde_forw_sparsity_in;
    capsule->sim_expl_vde_forw->casadi_sparsity_out = &atom_4ws_nmpc_expl_vde_forw_sparsity_out;
    capsule->sim_expl_vde_forw->casadi_work = &atom_4ws_nmpc_expl_vde_forw_work;
    external_function_param_casadi_create(capsule->sim_expl_vde_forw, np, &ext_fun_opts);

    capsule->sim_vde_adj_casadi->casadi_fun = &atom_4ws_nmpc_expl_vde_adj;
    capsule->sim_vde_adj_casadi->casadi_n_in = &atom_4ws_nmpc_expl_vde_adj_n_in;
    capsule->sim_vde_adj_casadi->casadi_n_out = &atom_4ws_nmpc_expl_vde_adj_n_out;
    capsule->sim_vde_adj_casadi->casadi_sparsity_in = &atom_4ws_nmpc_expl_vde_adj_sparsity_in;
    capsule->sim_vde_adj_casadi->casadi_sparsity_out = &atom_4ws_nmpc_expl_vde_adj_sparsity_out;
    capsule->sim_vde_adj_casadi->casadi_work = &atom_4ws_nmpc_expl_vde_adj_work;
    external_function_param_casadi_create(capsule->sim_vde_adj_casadi, np, &ext_fun_opts);

    capsule->sim_expl_ode_fun_casadi->casadi_fun = &atom_4ws_nmpc_expl_ode_fun;
    capsule->sim_expl_ode_fun_casadi->casadi_n_in = &atom_4ws_nmpc_expl_ode_fun_n_in;
    capsule->sim_expl_ode_fun_casadi->casadi_n_out = &atom_4ws_nmpc_expl_ode_fun_n_out;
    capsule->sim_expl_ode_fun_casadi->casadi_sparsity_in = &atom_4ws_nmpc_expl_ode_fun_sparsity_in;
    capsule->sim_expl_ode_fun_casadi->casadi_sparsity_out = &atom_4ws_nmpc_expl_ode_fun_sparsity_out;
    capsule->sim_expl_ode_fun_casadi->casadi_work = &atom_4ws_nmpc_expl_ode_fun_work;
    external_function_param_casadi_create(capsule->sim_expl_ode_fun_casadi, np, &ext_fun_opts);

    

    

    // sim plan & config
    sim_solver_plan_t plan;
    plan.sim_solver = ERK;

    // create correct config based on plan
    sim_config * atom_4ws_nmpc_sim_config = sim_config_create(plan);
    capsule->acados_sim_config = atom_4ws_nmpc_sim_config;

    // sim dims
    void *atom_4ws_nmpc_sim_dims = sim_dims_create(atom_4ws_nmpc_sim_config);
    capsule->acados_sim_dims = atom_4ws_nmpc_sim_dims;
    sim_dims_set(atom_4ws_nmpc_sim_config, atom_4ws_nmpc_sim_dims, "nx", &nx);
    sim_dims_set(atom_4ws_nmpc_sim_config, atom_4ws_nmpc_sim_dims, "nu", &nu);
    sim_dims_set(atom_4ws_nmpc_sim_config, atom_4ws_nmpc_sim_dims, "nz", &nz);
    sim_dims_set(atom_4ws_nmpc_sim_config, atom_4ws_nmpc_sim_dims, "np", &np);


    // sim opts
    sim_opts *atom_4ws_nmpc_sim_opts = sim_opts_create(atom_4ws_nmpc_sim_config, atom_4ws_nmpc_sim_dims);
    capsule->acados_sim_opts = atom_4ws_nmpc_sim_opts;
    int tmp_int = 3;
    sim_opts_set(atom_4ws_nmpc_sim_config, atom_4ws_nmpc_sim_opts, "newton_iter", &tmp_int);
    double tmp_double = 0;
    sim_opts_set(atom_4ws_nmpc_sim_config, atom_4ws_nmpc_sim_opts, "newton_tol", &tmp_double);
    sim_collocation_type collocation_type = GAUSS_LEGENDRE;
    sim_opts_set(atom_4ws_nmpc_sim_config, atom_4ws_nmpc_sim_opts, "collocation_type", &collocation_type);

 
    tmp_int = 4;
    sim_opts_set(atom_4ws_nmpc_sim_config, atom_4ws_nmpc_sim_opts, "num_stages", &tmp_int);
    tmp_int = 3;
    sim_opts_set(atom_4ws_nmpc_sim_config, atom_4ws_nmpc_sim_opts, "num_steps", &tmp_int);
    tmp_bool = 0;
    sim_opts_set(atom_4ws_nmpc_sim_config, atom_4ws_nmpc_sim_opts, "jac_reuse", &tmp_bool);


    // sim in / out
    sim_in *atom_4ws_nmpc_sim_in = sim_in_create(atom_4ws_nmpc_sim_config, atom_4ws_nmpc_sim_dims);
    capsule->acados_sim_in = atom_4ws_nmpc_sim_in;
    sim_out *atom_4ws_nmpc_sim_out = sim_out_create(atom_4ws_nmpc_sim_config, atom_4ws_nmpc_sim_dims);
    capsule->acados_sim_out = atom_4ws_nmpc_sim_out;

    sim_in_set(atom_4ws_nmpc_sim_config, atom_4ws_nmpc_sim_dims,
               atom_4ws_nmpc_sim_in, "T", &Tsim);

    // model functions
    atom_4ws_nmpc_sim_config->model_set(atom_4ws_nmpc_sim_in->model,
                 "expl_vde_forw", capsule->sim_expl_vde_forw);
    atom_4ws_nmpc_sim_config->model_set(atom_4ws_nmpc_sim_in->model,
                 "expl_vde_adj", capsule->sim_vde_adj_casadi);
    atom_4ws_nmpc_sim_config->model_set(atom_4ws_nmpc_sim_in->model,
                 "expl_ode_fun", capsule->sim_expl_ode_fun_casadi);
    

    // sim solver
    sim_solver *atom_4ws_nmpc_sim_solver = sim_solver_create(atom_4ws_nmpc_sim_config,
                                               atom_4ws_nmpc_sim_dims, atom_4ws_nmpc_sim_opts, atom_4ws_nmpc_sim_in);
    capsule->acados_sim_solver = atom_4ws_nmpc_sim_solver;

    capsule->acados_sim_mem = atom_4ws_nmpc_sim_solver->mem;


    /* initialize parameter values */
    
    double* p = malloc(np*sizeof(double));
    memcpy(p, p_init, np*sizeof(double));
    atom_4ws_nmpc_acados_sim_update_params(capsule, p, np);
    free(p);


    /* initialize input */
    // x
    double x0[9];
    for (int ii = 0; ii < 9; ii++)
        x0[ii] = 0.0;

    sim_in_set(atom_4ws_nmpc_sim_config, atom_4ws_nmpc_sim_dims,
               atom_4ws_nmpc_sim_in, "x", x0);


    // u
    double u0[2];
    for (int ii = 0; ii < 2; ii++)
        u0[ii] = 0.0;

    sim_in_set(atom_4ws_nmpc_sim_config, atom_4ws_nmpc_sim_dims,
               atom_4ws_nmpc_sim_in, "u", u0);

    // S_forw
    double S_forw[99];
    for (int ii = 0; ii < 99; ii++)
        S_forw[ii] = 0.0;
    for (int ii = 0; ii < 9; ii++)
        S_forw[ii + ii * 9 ] = 1.0;


    sim_in_set(atom_4ws_nmpc_sim_config, atom_4ws_nmpc_sim_dims,
               atom_4ws_nmpc_sim_in, "S_forw", S_forw);

    int status = sim_precompute(atom_4ws_nmpc_sim_solver, atom_4ws_nmpc_sim_in, atom_4ws_nmpc_sim_out);

    return status;
}


int atom_4ws_nmpc_acados_sim_solve(atom_4ws_nmpc_sim_solver_capsule *capsule)
{
    // integrate dynamics using acados sim_solver
    int status = sim_solve(capsule->acados_sim_solver,
                           capsule->acados_sim_in, capsule->acados_sim_out);
    if (status != 0)
        printf("error in atom_4ws_nmpc_acados_sim_solve()! Exiting.\n");

    return status;
}




int atom_4ws_nmpc_acados_sim_free(atom_4ws_nmpc_sim_solver_capsule *capsule)
{
    // free memory
    sim_solver_destroy(capsule->acados_sim_solver);
    sim_in_destroy(capsule->acados_sim_in);
    sim_out_destroy(capsule->acados_sim_out);
    sim_opts_destroy(capsule->acados_sim_opts);
    sim_dims_destroy(capsule->acados_sim_dims);
    sim_config_destroy(capsule->acados_sim_config);

    // free external function
    external_function_param_casadi_free(capsule->sim_expl_vde_forw);
    external_function_param_casadi_free(capsule->sim_vde_adj_casadi);
    external_function_param_casadi_free(capsule->sim_expl_ode_fun_casadi);
    
    free(capsule->sim_expl_vde_forw);
    free(capsule->sim_vde_adj_casadi);
    free(capsule->sim_expl_ode_fun_casadi);
    

    return 0;
}


int atom_4ws_nmpc_acados_sim_update_params(atom_4ws_nmpc_sim_solver_capsule *capsule, double *p, int np)
{
    int status = 0;
    int casadi_np = ATOM_4WS_NMPC_NP;

    if (casadi_np != np) {
        printf("atom_4ws_nmpc_acados_sim_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }
    capsule->sim_expl_vde_forw[0].set_param(capsule->sim_expl_vde_forw, p);
    capsule->sim_vde_adj_casadi[0].set_param(capsule->sim_vde_adj_casadi, p);
    capsule->sim_expl_ode_fun_casadi[0].set_param(capsule->sim_expl_ode_fun_casadi, p);
    

    return status;
}

/* getters pointers to C objects*/
sim_config * atom_4ws_nmpc_acados_get_sim_config(atom_4ws_nmpc_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_config;
};

sim_in * atom_4ws_nmpc_acados_get_sim_in(atom_4ws_nmpc_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_in;
};

sim_out * atom_4ws_nmpc_acados_get_sim_out(atom_4ws_nmpc_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_out;
};

void * atom_4ws_nmpc_acados_get_sim_dims(atom_4ws_nmpc_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_dims;
};

sim_opts * atom_4ws_nmpc_acados_get_sim_opts(atom_4ws_nmpc_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_opts;
};

sim_solver  * atom_4ws_nmpc_acados_get_sim_solver(atom_4ws_nmpc_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_solver;
};

void * atom_4ws_nmpc_acados_get_sim_mem(atom_4ws_nmpc_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_mem;
};

