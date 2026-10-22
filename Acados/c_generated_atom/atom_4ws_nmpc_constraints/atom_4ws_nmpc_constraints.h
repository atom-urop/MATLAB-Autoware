/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * Licensed under the 2-Clause BSD License.
 */

#ifndef atom_4ws_nmpc_CONSTRAINTS
#define atom_4ws_nmpc_CONSTRAINTS

#ifdef __cplusplus
extern "C" {
#endif









int atom_4ws_nmpc_constr_h_fun_jac_uxt_zt(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int atom_4ws_nmpc_constr_h_fun_jac_uxt_zt_work(int *, int *, int *, int *);
const int *atom_4ws_nmpc_constr_h_fun_jac_uxt_zt_sparsity_in(int);
const int *atom_4ws_nmpc_constr_h_fun_jac_uxt_zt_sparsity_out(int);
int atom_4ws_nmpc_constr_h_fun_jac_uxt_zt_n_in(void);
int atom_4ws_nmpc_constr_h_fun_jac_uxt_zt_n_out(void);

int atom_4ws_nmpc_constr_h_fun(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int atom_4ws_nmpc_constr_h_fun_work(int *, int *, int *, int *);
const int *atom_4ws_nmpc_constr_h_fun_sparsity_in(int);
const int *atom_4ws_nmpc_constr_h_fun_sparsity_out(int);
int atom_4ws_nmpc_constr_h_fun_n_in(void);
int atom_4ws_nmpc_constr_h_fun_n_out(void);











int atom_4ws_nmpc_constr_h_0_fun_jac_uxt_zt(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int atom_4ws_nmpc_constr_h_0_fun_jac_uxt_zt_work(int *, int *, int *, int *);
const int *atom_4ws_nmpc_constr_h_0_fun_jac_uxt_zt_sparsity_in(int);
const int *atom_4ws_nmpc_constr_h_0_fun_jac_uxt_zt_sparsity_out(int);
int atom_4ws_nmpc_constr_h_0_fun_jac_uxt_zt_n_in(void);
int atom_4ws_nmpc_constr_h_0_fun_jac_uxt_zt_n_out(void);

int atom_4ws_nmpc_constr_h_0_fun(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int atom_4ws_nmpc_constr_h_0_fun_work(int *, int *, int *, int *);
const int *atom_4ws_nmpc_constr_h_0_fun_sparsity_in(int);
const int *atom_4ws_nmpc_constr_h_0_fun_sparsity_out(int);
int atom_4ws_nmpc_constr_h_0_fun_n_in(void);
int atom_4ws_nmpc_constr_h_0_fun_n_out(void);













int atom_4ws_nmpc_constr_h_e_fun_jac_uxt_zt(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int atom_4ws_nmpc_constr_h_e_fun_jac_uxt_zt_work(int *, int *, int *, int *);
const int *atom_4ws_nmpc_constr_h_e_fun_jac_uxt_zt_sparsity_in(int);
const int *atom_4ws_nmpc_constr_h_e_fun_jac_uxt_zt_sparsity_out(int);
int atom_4ws_nmpc_constr_h_e_fun_jac_uxt_zt_n_in(void);
int atom_4ws_nmpc_constr_h_e_fun_jac_uxt_zt_n_out(void);

int atom_4ws_nmpc_constr_h_e_fun(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int atom_4ws_nmpc_constr_h_e_fun_work(int *, int *, int *, int *);
const int *atom_4ws_nmpc_constr_h_e_fun_sparsity_in(int);
const int *atom_4ws_nmpc_constr_h_e_fun_sparsity_out(int);
int atom_4ws_nmpc_constr_h_e_fun_n_in(void);
int atom_4ws_nmpc_constr_h_e_fun_n_out(void);











#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // atom_4ws_nmpc_CONSTRAINTS
