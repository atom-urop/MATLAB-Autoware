/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * Licensed under the 2-Clause BSD License.
 */

#ifndef ACADOS_SOLVER_atom_4ws_nmpc_H_
#define ACADOS_SOLVER_atom_4ws_nmpc_H_

#include "acados/utils/types.h"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define ATOM_4WS_NMPC_NX     9
#define ATOM_4WS_NMPC_NZ     0
#define ATOM_4WS_NMPC_NU     2
#define ATOM_4WS_NMPC_NP     17
#define ATOM_4WS_NMPC_NP_GLOBAL     0
#define ATOM_4WS_NMPC_NBX    4
#define ATOM_4WS_NMPC_NBX0   9
#define ATOM_4WS_NMPC_NBU    2
#define ATOM_4WS_NMPC_NSBX   0
#define ATOM_4WS_NMPC_NSBU   0
#define ATOM_4WS_NMPC_NSH    16
#define ATOM_4WS_NMPC_NSH0   16
#define ATOM_4WS_NMPC_NSG    0
#define ATOM_4WS_NMPC_NSPHI  0
#define ATOM_4WS_NMPC_NSHN   16
#define ATOM_4WS_NMPC_NSGN   0
#define ATOM_4WS_NMPC_NSPHIN 0
#define ATOM_4WS_NMPC_NSPHI0 0
#define ATOM_4WS_NMPC_NSBXN  0
#define ATOM_4WS_NMPC_NS     16
#define ATOM_4WS_NMPC_NS0    16
#define ATOM_4WS_NMPC_NSN    16
#define ATOM_4WS_NMPC_NG     0
#define ATOM_4WS_NMPC_NBXN   4
#define ATOM_4WS_NMPC_NGN    0
#define ATOM_4WS_NMPC_NY0    7
#define ATOM_4WS_NMPC_NY     7
#define ATOM_4WS_NMPC_NYN    5
#define ATOM_4WS_NMPC_N      15
#define ATOM_4WS_NMPC_NH     16
#define ATOM_4WS_NMPC_NHN    16
#define ATOM_4WS_NMPC_NH0    16
#define ATOM_4WS_NMPC_NPHI0  0
#define ATOM_4WS_NMPC_NPHI   0
#define ATOM_4WS_NMPC_NPHIN  0
#define ATOM_4WS_NMPC_NR     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct atom_4ws_nmpc_solver_capsule
{
    // acados objects
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_out *sens_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
    ocp_nlp_plan_t *nlp_solver_plan;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;

    // number of expected runtime parameters
    unsigned int nlp_np;

    /* external functions */

    // dynamics

    external_function_external_param_casadi *expl_vde_forw;
    external_function_external_param_casadi *expl_vde_forw_p;
    external_function_external_param_casadi *expl_ode_fun;
    external_function_external_param_casadi *expl_vde_adj;




    // cost






    // constraints
    external_function_external_param_casadi *nl_constr_h_fun_jac;
    external_function_external_param_casadi *nl_constr_h_fun;






    external_function_external_param_casadi nl_constr_h_0_fun_jac;
    external_function_external_param_casadi nl_constr_h_0_fun;






    external_function_external_param_casadi nl_constr_h_e_fun_jac;
    external_function_external_param_casadi nl_constr_h_e_fun;




} atom_4ws_nmpc_solver_capsule;

ACADOS_SYMBOL_EXPORT atom_4ws_nmpc_solver_capsule * atom_4ws_nmpc_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int atom_4ws_nmpc_acados_free_capsule(atom_4ws_nmpc_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int atom_4ws_nmpc_acados_create(atom_4ws_nmpc_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int atom_4ws_nmpc_acados_reset(atom_4ws_nmpc_solver_capsule* capsule, int reset_qp_solver_mem, int reset_numerical_values, int reset_solver_options, int reset_x_to_x0_bar);

/**
 * Generic version of atom_4ws_nmpc_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
ACADOS_SYMBOL_EXPORT int atom_4ws_nmpc_acados_create_with_discretization(atom_4ws_nmpc_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
ACADOS_SYMBOL_EXPORT int atom_4ws_nmpc_acados_update_time_steps(atom_4ws_nmpc_solver_capsule * capsule, int N, double* new_time_steps);
/**
 * This function is used for updating an already initialized solver with a different number of qp_cond_N.
 */
ACADOS_SYMBOL_EXPORT int atom_4ws_nmpc_acados_update_qp_solver_cond_N(atom_4ws_nmpc_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int atom_4ws_nmpc_acados_update_params(atom_4ws_nmpc_solver_capsule * capsule, int stage, double *value, int np);
ACADOS_SYMBOL_EXPORT int atom_4ws_nmpc_acados_update_params_sparse(atom_4ws_nmpc_solver_capsule * capsule, int stage, int *idx, double *p, int n_update);
ACADOS_SYMBOL_EXPORT int atom_4ws_nmpc_acados_set_p_global_and_precompute_dependencies(atom_4ws_nmpc_solver_capsule* capsule, double* data, int data_len);

ACADOS_SYMBOL_EXPORT int atom_4ws_nmpc_acados_solve(atom_4ws_nmpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int atom_4ws_nmpc_acados_setup_qp_matrices_and_factorize(atom_4ws_nmpc_solver_capsule* capsule);



ACADOS_SYMBOL_EXPORT int atom_4ws_nmpc_acados_free(atom_4ws_nmpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void atom_4ws_nmpc_acados_print_stats(atom_4ws_nmpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int atom_4ws_nmpc_acados_custom_update(atom_4ws_nmpc_solver_capsule* capsule, double* data, int data_len);

ACADOS_SYMBOL_EXPORT ocp_nlp_in *atom_4ws_nmpc_acados_get_nlp_in(atom_4ws_nmpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *atom_4ws_nmpc_acados_get_nlp_out(atom_4ws_nmpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *atom_4ws_nmpc_acados_get_sens_out(atom_4ws_nmpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver *atom_4ws_nmpc_acados_get_nlp_solver(atom_4ws_nmpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config *atom_4ws_nmpc_acados_get_nlp_config(atom_4ws_nmpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void *atom_4ws_nmpc_acados_get_nlp_opts(atom_4ws_nmpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims *atom_4ws_nmpc_acados_get_nlp_dims(atom_4ws_nmpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t *atom_4ws_nmpc_acados_get_nlp_plan(atom_4ws_nmpc_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_atom_4ws_nmpc_H_
