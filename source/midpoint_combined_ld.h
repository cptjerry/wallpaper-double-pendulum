#ifndef MIDPOINT_COMBINED_LD_H
#define MIDPOINT_COMBINED_LD_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef long double mcld_real;

typedef struct {
    mcld_real beta;
    int max_iter;
    mcld_real tol;
} mcld_solver_opts;

void mcld_default_params(mcld_real params_out[11], mcld_real Ispin1, mcld_real Ispin2);
void mcld_default_solver_opts(mcld_solver_opts *opts);

/* Legendre/Lagrange interfaces: init by (y_{-1}, y_0) */
int mcld_legendre_step_from_pair(
    const mcld_real y_prev[8],
    const mcld_real y_cur[8],
    mcld_real h,
    const mcld_real params[11],
    const mcld_solver_opts *opts,
    mcld_real y_next[8],
    mcld_real z_cur[6],
    mcld_real p_plus[6],
    mcld_real *best_residual,
    int *niters);

int mcld_legendre_integrate_trajectory_from_pair(
    const mcld_real y_prev0[8],
    const mcld_real y0[8],
    mcld_real t_end,
    int n_steps,
    const mcld_real params[11],
    const mcld_solver_opts *opts,
    mcld_real *Y_out, /* (n_steps+2)*8 */
    mcld_real *Z_out, /* (n_steps+1)*6 */
    mcld_real *P_out, /* (n_steps+1)*6 */
    mcld_real *worst_residual,
    int *worst_iters,
    int *total_iters);

int mcld_legendre_integrate_final_from_pair(
    const mcld_real y_prev0[8],
    const mcld_real y0[8],
    mcld_real t_end,
    int n_steps,
    const mcld_real params[11],
    const mcld_solver_opts *opts,
    mcld_real y_prev_final[8],
    mcld_real y_final[8],
    mcld_real z_last[6],
    mcld_real p_last[6],
    mcld_real *worst_residual,
    int *worst_iters,
    int *total_iters);

/* Hamilton interfaces: init by (p_{-1/2}, q_0) */
int mcld_hamilton_step_from_left(
    const mcld_real y_cur[8],
    const mcld_real p_mhalf[6],
    mcld_real h,
    const mcld_real params[11],
    const mcld_solver_opts *opts,
    mcld_real y_next[8],
    mcld_real p_phalf[6],
    mcld_real z_cur[6],
    mcld_real *best_residual,
    int *niters);

int mcld_hamilton_integrate_trajectory_from_left(
    const mcld_real y0[8],
    const mcld_real p_mhalf0[6],
    mcld_real t_end,
    int n_steps,
    const mcld_real params[11],
    const mcld_solver_opts *opts,
    mcld_real *Y_out, /* (n_steps+1)*8 */
    mcld_real *P_out, /* (n_steps+1)*6, half-step momentum sequence */
    mcld_real *Z_out, /* (n_steps)*6 */
    mcld_real *worst_residual,
    int *worst_iters,
    int *total_iters);

int mcld_hamilton_integrate_final_from_left(
    const mcld_real y0[8],
    const mcld_real p_mhalf0[6],
    mcld_real t_end,
    int n_steps,
    const mcld_real params[11],
    const mcld_solver_opts *opts,
    mcld_real y_final[8],
    mcld_real p_final[6],
    mcld_real z_last[6],
    mcld_real *worst_residual,
    int *worst_iters,
    int *total_iters);

/* Hamilton interfaces: init by (q_0, p_{+1/2}) */
int mcld_hamilton_step_from_right(
    const mcld_real y_cur[8],
    const mcld_real p_phalf[6],
    mcld_real h,
    const mcld_real params[11],
    const mcld_solver_opts *opts,
    mcld_real y_next[8],
    mcld_real p_same[6],
    mcld_real z_cur[6],
    mcld_real *best_residual,
    int *niters);

int mcld_hamilton_integrate_trajectory_from_right(
    const mcld_real y0[8],
    const mcld_real p_phalf0[6],
    mcld_real t_end,
    int n_steps,
    const mcld_real params[11],
    const mcld_solver_opts *opts,
    mcld_real *Y_out, /* (n_steps+1)*8 */
    mcld_real *P_out, /* (n_steps)*6, stored p_{k+1/2} */
    mcld_real *Z_out, /* (n_steps)*6 */
    mcld_real *worst_residual,
    int *worst_iters,
    int *total_iters);

int mcld_hamilton_integrate_final_from_right(
    const mcld_real y0[8],
    const mcld_real p_phalf0[6],
    mcld_real t_end,
    int n_steps,
    const mcld_real params[11],
    const mcld_solver_opts *opts,
    mcld_real y_final[8],
    mcld_real p_last[6],
    mcld_real z_last[6],
    mcld_real *worst_residual,
    int *worst_iters,
    int *total_iters);

#ifdef __cplusplus
}
#endif

#endif
