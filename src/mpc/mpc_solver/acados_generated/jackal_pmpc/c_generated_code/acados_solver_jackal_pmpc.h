/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

#ifndef ACADOS_SOLVER_jackal_pmpc_H_
#define ACADOS_SOLVER_jackal_pmpc_H_

#include "acados/utils/types.h"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define JACKAL_PMPC_NX     4
#define JACKAL_PMPC_NZ     0
#define JACKAL_PMPC_NU     2
#define JACKAL_PMPC_NP     80
#define JACKAL_PMPC_NP_GLOBAL     0
#define JACKAL_PMPC_NBX    0
#define JACKAL_PMPC_NBX0   4
#define JACKAL_PMPC_NBU    0
#define JACKAL_PMPC_NSBX   0
#define JACKAL_PMPC_NSBU   0
#define JACKAL_PMPC_NSH    0
#define JACKAL_PMPC_NSH0   0
#define JACKAL_PMPC_NSG    0
#define JACKAL_PMPC_NSPHI  0
#define JACKAL_PMPC_NSHN   0
#define JACKAL_PMPC_NSGN   0
#define JACKAL_PMPC_NSPHIN 0
#define JACKAL_PMPC_NSPHI0 0
#define JACKAL_PMPC_NSBXN  0
#define JACKAL_PMPC_NS     0
#define JACKAL_PMPC_NS0    0
#define JACKAL_PMPC_NSN    0
#define JACKAL_PMPC_NG     0
#define JACKAL_PMPC_NBXN   3
#define JACKAL_PMPC_NGN    0
#define JACKAL_PMPC_NY0    0
#define JACKAL_PMPC_NY     0
#define JACKAL_PMPC_NYN    0
#define JACKAL_PMPC_N      8
#define JACKAL_PMPC_NH     36
#define JACKAL_PMPC_NHN    25
#define JACKAL_PMPC_NH0    12
#define JACKAL_PMPC_NPHI0  0
#define JACKAL_PMPC_NPHI   0
#define JACKAL_PMPC_NPHIN  0
#define JACKAL_PMPC_NR     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct jackal_pmpc_solver_capsule
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

    external_function_external_param_casadi *discr_dyn_phi_fun;
    external_function_external_param_casadi *discr_dyn_phi_fun_jac_ut_xt;


    external_function_external_param_casadi *discr_dyn_phi_fun_jac_ut_xt_hess;


    // cost

    external_function_external_param_casadi *ext_cost_fun;
    external_function_external_param_casadi *ext_cost_fun_jac;
    external_function_external_param_casadi *ext_cost_fun_jac_hess;





    external_function_external_param_casadi ext_cost_0_fun;
    external_function_external_param_casadi ext_cost_0_fun_jac;
    external_function_external_param_casadi ext_cost_0_fun_jac_hess;




    external_function_external_param_casadi ext_cost_e_fun;
    external_function_external_param_casadi ext_cost_e_fun_jac;
    external_function_external_param_casadi ext_cost_e_fun_jac_hess;



    // constraints
    external_function_external_param_casadi *nl_constr_h_fun_jac;
    external_function_external_param_casadi *nl_constr_h_fun;
    external_function_external_param_casadi *nl_constr_h_fun_jac_hess;





    external_function_external_param_casadi nl_constr_h_0_fun_jac;
    external_function_external_param_casadi nl_constr_h_0_fun;
    external_function_external_param_casadi nl_constr_h_0_fun_jac_hess;





    external_function_external_param_casadi nl_constr_h_e_fun_jac;
    external_function_external_param_casadi nl_constr_h_e_fun;
    external_function_external_param_casadi nl_constr_h_e_fun_jac_hess;



} jackal_pmpc_solver_capsule;

ACADOS_SYMBOL_EXPORT jackal_pmpc_solver_capsule * jackal_pmpc_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int jackal_pmpc_acados_free_capsule(jackal_pmpc_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int jackal_pmpc_acados_create(jackal_pmpc_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int jackal_pmpc_acados_reset(jackal_pmpc_solver_capsule* capsule, int reset_qp_solver_mem);

/**
 * Generic version of jackal_pmpc_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
ACADOS_SYMBOL_EXPORT int jackal_pmpc_acados_create_with_discretization(jackal_pmpc_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
ACADOS_SYMBOL_EXPORT int jackal_pmpc_acados_update_time_steps(jackal_pmpc_solver_capsule * capsule, int N, double* new_time_steps);
/**
 * This function is used for updating an already initialized solver with a different number of qp_cond_N.
 */
ACADOS_SYMBOL_EXPORT int jackal_pmpc_acados_update_qp_solver_cond_N(jackal_pmpc_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int jackal_pmpc_acados_update_params(jackal_pmpc_solver_capsule * capsule, int stage, double *value, int np);
ACADOS_SYMBOL_EXPORT int jackal_pmpc_acados_update_params_sparse(jackal_pmpc_solver_capsule * capsule, int stage, int *idx, double *p, int n_update);
ACADOS_SYMBOL_EXPORT int jackal_pmpc_acados_set_p_global_and_precompute_dependencies(jackal_pmpc_solver_capsule* capsule, double* data, int data_len);

ACADOS_SYMBOL_EXPORT int jackal_pmpc_acados_solve(jackal_pmpc_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT void jackal_pmpc_acados_batch_solve(jackal_pmpc_solver_capsule ** capsules, int N_batch);

ACADOS_SYMBOL_EXPORT void jackal_pmpc_acados_batch_set_flat(jackal_pmpc_solver_capsule ** capsules, const char *field, double *data, int N_data, int N_batch);
ACADOS_SYMBOL_EXPORT void jackal_pmpc_acados_batch_get_flat(jackal_pmpc_solver_capsule ** capsules, const char *field, double *data, int N_data, int N_batch);

ACADOS_SYMBOL_EXPORT void jackal_pmpc_acados_batch_eval_solution_sens_adj_p(jackal_pmpc_solver_capsule ** capsules, const char *field, int stage, double *out, int offset, int N_batch);
ACADOS_SYMBOL_EXPORT void jackal_pmpc_acados_batch_eval_params_jac(jackal_pmpc_solver_capsule ** capsules, int N_batch);


ACADOS_SYMBOL_EXPORT int jackal_pmpc_acados_free(jackal_pmpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void jackal_pmpc_acados_print_stats(jackal_pmpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int jackal_pmpc_acados_custom_update(jackal_pmpc_solver_capsule* capsule, double* data, int data_len);


ACADOS_SYMBOL_EXPORT ocp_nlp_in *jackal_pmpc_acados_get_nlp_in(jackal_pmpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *jackal_pmpc_acados_get_nlp_out(jackal_pmpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *jackal_pmpc_acados_get_sens_out(jackal_pmpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver *jackal_pmpc_acados_get_nlp_solver(jackal_pmpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config *jackal_pmpc_acados_get_nlp_config(jackal_pmpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void *jackal_pmpc_acados_get_nlp_opts(jackal_pmpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims *jackal_pmpc_acados_get_nlp_dims(jackal_pmpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t *jackal_pmpc_acados_get_nlp_plan(jackal_pmpc_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_jackal_pmpc_H_
