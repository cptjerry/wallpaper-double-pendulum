#include "midpoint_combined_ld.h"
#include <math.h>
#include <string.h>

#ifdef __EMSCRIPTEN__
  #include <emscripten/emscripten.h>
#else
  #define EMSCRIPTEN_KEEPALIVE
#endif

enum {
    MCLD_STEP_LAGRANGE = 0,
    MCLD_STEP_HAMILTON_LEFT = 1,
    MCLD_STEP_HAMILTON_RIGHT = 2
};

enum {
    MCLD_INIT_PAIR = 0,
    MCLD_INIT_HAMILTON_LEFT = 1,
    MCLD_INIT_HAMILTON_RIGHT = 2
};

enum {
    MCLD_CFG_Y_PREV = 0,
    MCLD_CFG_Y_CUR = 1,
    MCLD_CFG_P_LEFT = 2,
    MCLD_CFG_P_RIGHT = 3,
    MCLD_RESTORE_Y = 4,
    MCLD_RESTORE_P = 5,
    MCLD_CFG_OMEGA = 6
};

enum {
    MCLD_FLAG_USE_CFG_OMEGA = 0
};

enum {
    MCLD_PERSIST_Y_CUR = 0,
    MCLD_PERSIST_P_HALF = 1,
    MCLD_PERSIST_TIME = 2,
    MCLD_PERSIST_ENERGY_CURRENT = 3
};

enum {
    MCLD_RESTORE_HEX_Y = 0,
    MCLD_RESTORE_HEX_P = 1
};

typedef struct {
    mcld_real sim_dt;
    mcld_real t;
    mcld_real params[11];
    mcld_solver_opts opts;

    int step_mode;
    int init_mode;
    int initialized;

    mcld_real cfg_y_prev[8];
    mcld_real cfg_y_cur[8];
    mcld_real cfg_p_left[6];
    mcld_real cfg_p_right[6];
    mcld_real cfg_omega[6];
    int cfg_use_omega;
    mcld_real restore_y[8];
    mcld_real restore_p[6];
    mcld_real restore_t;
    mcld_real restore_H;

    mcld_real y_prev[8];
    mcld_real y_cur[8];
    mcld_real y_next[8];
    mcld_real p_half[6];
    mcld_real p_next[6];
    mcld_real z_cur[6];

    int last_lib_rc;
    mcld_real last_best_residual;
    int last_niters;

    mcld_real geom[27];
    mcld_real l1_vis;
    mcld_real l2_vis;

    mcld_real initial_potential;
    mcld_real initial_energy;

    mcld_real interval_y0[8];
    mcld_real interval_y1[8];
    int has_interval;
    int right_bootstrap_pending;
} mcld_wallpaper_state;

static mcld_wallpaper_state G;


static int hex_nibble_value(int ch) {
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'a' && ch <= 'f') return 10 + (ch - 'a');
    if (ch >= 'A' && ch <= 'F') return 10 + (ch - 'A');
    return -1;
}

static void bytes_to_hex_local(const unsigned char *bytes, int nbytes, char *out) {
    static const char HEX_DIGITS[] = "0123456789abcdef";
    for (int i = 0; i < nbytes; ++i) {
        unsigned char b = bytes[i];
        out[2 * i + 0] = HEX_DIGITS[(b >> 4) & 0x0F];
        out[2 * i + 1] = HEX_DIGITS[b & 0x0F];
    }
    out[2 * nbytes] = '\0';
}

static int hex_to_bytes_local(const char *hex, unsigned char *out, int nbytes) {
    if (!hex) return 0;
    const size_t expected_len = (size_t)nbytes * 2u;
    if (strlen(hex) != expected_len) return 0;
    for (int i = 0; i < nbytes; ++i) {
        int hi = hex_nibble_value((unsigned char)hex[2 * i + 0]);
        int lo = hex_nibble_value((unsigned char)hex[2 * i + 1]);
        if (hi < 0 || lo < 0) return 0;
        out[i] = (unsigned char)((hi << 4) | lo);
    }
    return 1;
}

static const char *real_to_hex_cstr_local(mcld_real value) {
    static char out[sizeof(mcld_real) * 2 + 1];
    unsigned char bytes[sizeof(mcld_real)];
    memcpy(bytes, &value, sizeof(bytes));
    bytes_to_hex_local(bytes, (int)sizeof(bytes), out);
    return out;
}

static int real_from_hex_cstr_local(const char *hex, mcld_real *out) {
    if (!out) return 0;
    unsigned char bytes[sizeof(mcld_real)];
    if (!hex_to_bytes_local(hex, bytes, (int)sizeof(bytes))) return 0;
    memcpy(out, bytes, sizeof(bytes));
    return 1;
}

static void q_normalize_to(const mcld_real q[4], mcld_real out[4]) {
    mcld_real n2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (n2 < 1e-30L) {
        out[0] = 1.0L; out[1] = out[2] = out[3] = 0.0L; return;
    }
    mcld_real inv = 1.0L / sqrtl(n2);
    out[0] = q[0] * inv; out[1] = q[1] * inv; out[2] = q[2] * inv; out[3] = q[3] * inv;
}

static void q_to_R_inplace(const mcld_real q[4], mcld_real R[3][3]) {
    mcld_real qq[4];
    q_normalize_to(q, qq);
    mcld_real w = qq[0], x = qq[1], y = qq[2], z = qq[3];
    R[0][0] = 1 - 2*(y*y + z*z); R[0][1] = 2*(x*y - w*z); R[0][2] = 2*(x*z + w*y);
    R[1][0] = 2*(x*y + w*z);     R[1][1] = 1 - 2*(x*x + z*z); R[1][2] = 2*(y*z - w*x);
    R[2][0] = 2*(x*z - w*y);     R[2][1] = 2*(y*z + w*x);     R[2][2] = 1 - 2*(x*x + y*y);
}


static void q_conj_inplace_local(const mcld_real q[4], mcld_real out[4]) { out[0]=q[0]; out[1]=-q[1]; out[2]=-q[2]; out[3]=-q[3]; }

static void q_mul_inplace_local(const mcld_real a[4], const mcld_real b[4], mcld_real out[4]) {
    mcld_real a0=a[0],a1=a[1],a2=a[2],a3=a[3];
    mcld_real b0=b[0],b1=b[1],b2=b[2],b3=b[3];
    out[0]=a0*b0-a1*b1-a2*b2-a3*b3;
    out[1]=a0*b1+a1*b0+a2*b3-a3*b2;
    out[2]=a0*b2-a1*b3+a2*b0+a3*b1;
    out[3]=a0*b3+a1*b2-a2*b1+a3*b0;
}

static void q_log_rotvec_to_local(const mcld_real q[4], mcld_real out[3]) {
    mcld_real qq[4]; q_normalize_to(q, qq);
    mcld_real w=qq[0], vx=qq[1], vy=qq[2], vz=qq[3];
    if (w > 1.0L) w = 1.0L; else if (w < -1.0L) w = -1.0L;
    mcld_real s = sqrtl(vx*vx + vy*vy + vz*vz);
    if (s < 1e-14L) { out[0]=2.0L*vx; out[1]=2.0L*vy; out[2]=2.0L*vz; return; }
    mcld_real ang = 2.0L * atan2l(s, w);
    mcld_real fac = ang / s;
    out[0] = fac * vx; out[1] = fac * vy; out[2] = fac * vz;
}

static void retract_pair_full_to_local(const mcld_real y_base[8], const mcld_real z6[6], mcld_real out[8]);

static void q_exp_rotvec_to_local(const mcld_real v[3], mcld_real out[4]) {
    mcld_real ang = sqrtl(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    mcld_real s;
    if (ang < 1e-14L) {
        mcld_real a2 = ang*ang;
        out[0] = 1.0L - 0.125L*a2;
        s = 0.5L - a2/48.0L;
    } else {
        mcld_real half = 0.5L * ang;
        out[0] = cosl(half);
        s = sinl(half) / ang;
    }
    out[1] = s * v[0]; out[2] = s * v[1]; out[3] = s * v[2];
}

static void hemi_copy4_local(const mcld_real qref[4], const mcld_real qin[4], mcld_real qout[4]) {
    mcld_real qr[4], qi[4];
    q_normalize_to(qref, qr); q_normalize_to(qin, qi);
    mcld_real s = qr[0]*qi[0] + qr[1]*qi[1] + qr[2]*qi[2] + qr[3]*qi[3];
    if (s < 0) { for (int i=0;i<4;++i) qout[i] = -qin[i]; }
    else { for (int i=0;i<4;++i) qout[i] = qin[i]; }
}

static void set_identity_pose(mcld_real y[8]) {
    y[0] = 1.0L; y[1] = 0.0L; y[2] = 0.0L; y[3] = 0.0L;
    y[4] = 1.0L; y[5] = 0.0L; y[6] = 0.0L; y[7] = 0.0L;
}

static void exp_step_from_omega_full_to_local(const mcld_real y0[8], const mcld_real omega6[6], mcld_real h, mcld_real out[8]) {
    mcld_real z6[6];
    for (int i = 0; i < 6; ++i) z6[i] = h * omega6[i];
    retract_pair_full_to_local(y0, z6, out);
}

static void pair_to_half_momentum_exact_local(const mcld_real y_prev[8], const mcld_real y_cur[8], mcld_real out[6]) {
    mcld_real YL[(1 + 2) * 8];
    mcld_real ZL[(1 + 1) * 6];
    mcld_real PL[(1 + 1) * 6];
    mcld_real wr = 0.0L;
    int wi = 0, ti = 0;
    (void)mcld_legendre_integrate_trajectory_from_pair(
        y_prev, y_cur, G.sim_dt, 1,
        G.params, &G.opts,
        YL, ZL, PL, &wr, &wi, &ti
    );
    memcpy(out, PL, 6 * sizeof(mcld_real));
}

static void derive_cfg_momenta_from_omega_exact(void) {
    mcld_real y_forward[8], y_backward[8], omega_neg[6];
    exp_step_from_omega_full_to_local(G.cfg_y_cur, G.cfg_omega, G.sim_dt, y_forward);
    for (int i = 0; i < 6; ++i) omega_neg[i] = -G.cfg_omega[i];
    exp_step_from_omega_full_to_local(G.cfg_y_cur, omega_neg, G.sim_dt, y_backward);
    pair_to_half_momentum_exact_local(y_backward, G.cfg_y_cur, G.cfg_p_left);
    pair_to_half_momentum_exact_local(G.cfg_y_cur, y_forward, G.cfg_p_right);
}

static void reset_cfg_defaults(void) {
    memset(G.cfg_y_prev, 0, sizeof(G.cfg_y_prev));
    memset(G.cfg_y_cur, 0, sizeof(G.cfg_y_cur));
    memset(G.cfg_p_left, 0, sizeof(G.cfg_p_left));
    memset(G.cfg_p_right, 0, sizeof(G.cfg_p_right));
    memset(G.cfg_omega, 0, sizeof(G.cfg_omega));
    G.cfg_use_omega = 0;
    memset(G.restore_y, 0, sizeof(G.restore_y));
    memset(G.restore_p, 0, sizeof(G.restore_p));
    G.restore_t = 0.0L;
    G.restore_H = 0.0L;
    set_identity_pose(G.cfg_y_prev);
    set_identity_pose(G.cfg_y_cur);
}


static mcld_real dot3(const mcld_real a[3], const mcld_real b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static mcld_real norm3_sq(const mcld_real a[3]) {
    return dot3(a, a);
}

static void cross3(const mcld_real a[3], const mcld_real b[3], mcld_real out[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

static void add3(const mcld_real a[3], const mcld_real b[3], mcld_real out[3]) {
    out[0] = a[0] + b[0]; out[1] = a[1] + b[1]; out[2] = a[2] + b[2];
}

static void scale3(const mcld_real a[3], mcld_real s, mcld_real out[3]) {
    out[0] = a[0] * s; out[1] = a[1] * s; out[2] = a[2] * s;
}

static void compute_n_solver_from_y(const mcld_real y[8], mcld_real n1[3], mcld_real n2[3]) {
    mcld_real R1[3][3], R2[3][3];
    q_to_R_inplace(y + 0, R1);
    q_to_R_inplace(y + 4, R2);
    n1[0] = R1[2][0]; n1[1] = R1[2][1]; n1[2] = R1[2][2];
    n2[0] = R2[2][0]; n2[1] = R2[2][1]; n2[2] = R2[2][2];
}

static void compute_energy_from_state(const mcld_real y[8], const mcld_real z[6], mcld_real h, const mcld_real params[11], mcld_real *U_out, mcld_real *H_out) {
    const mcld_real l1 = params[0], l2 = params[1];
    const mcld_real m1 = params[2], m2 = params[3];
    const mcld_real lc1 = params[4], lc2 = params[5];
    const mcld_real Iperp1 = params[6], Iperp2 = params[7];
    const mcld_real Ispin1 = params[8], Ispin2 = params[9];
    const mcld_real g = fabsl(params[10]);

    mcld_real n1[3], n2[3];
    compute_n_solver_from_y(y, n1, n2);

    mcld_real U = -g * (m1 * lc1 * n1[2] + m2 * (l1 * n1[2] + lc2 * n2[2]));

    mcld_real H = U;
    if (h > 1e-18L) {
        mcld_real w1[3] = { z[0] / h, z[1] / h, z[2] / h };
        mcld_real w2[3] = { z[3] / h, z[4] / h, z[5] / h };

        mcld_real vcom1[3], vjoint[3], vcom2_local[3], vcom2[3];
        cross3(w1, n1, vcom1); scale3(vcom1, lc1, vcom1);
        cross3(w1, n1, vjoint); scale3(vjoint, l1, vjoint);
        cross3(w2, n2, vcom2_local); scale3(vcom2_local, lc2, vcom2_local);
        add3(vjoint, vcom2_local, vcom2);

        mcld_real wp1 = dot3(w1, n1);
        mcld_real wp2 = dot3(w2, n2);
        mcld_real w1perp2 = norm3_sq(w1) - wp1 * wp1; if (w1perp2 < 0.0L) w1perp2 = 0.0L;
        mcld_real w2perp2 = norm3_sq(w2) - wp2 * wp2; if (w2perp2 < 0.0L) w2perp2 = 0.0L;

        mcld_real K = 0.5L * m1 * norm3_sq(vcom1)
                    + 0.5L * m2 * norm3_sq(vcom2)
                    + 0.5L * Iperp1 * w1perp2
                    + 0.5L * Iperp2 * w2perp2
                    + 0.5L * Ispin1 * wp1 * wp1
                    + 0.5L * Ispin2 * wp2 * wp2;
        H = U + K;
    }
    if (U_out) *U_out = U;
    if (H_out) *H_out = H;
}

static void compute_qerr_from_y(const mcld_real y[8], mcld_real *q1err, mcld_real *q2err, mcld_real *qmax) {
    mcld_real n1 = y[0]*y[0] + y[1]*y[1] + y[2]*y[2] + y[3]*y[3];
    mcld_real n2 = y[4]*y[4] + y[5]*y[5] + y[6]*y[6] + y[7]*y[7];
    mcld_real e1 = fabsl(n1 - 1.0L);
    mcld_real e2 = fabsl(n2 - 1.0L);
    if (q1err) *q1err = e1;
    if (q2err) *q2err = e2;
    if (qmax) *qmax = (e1 > e2 ? e1 : e2);
}

static void solve4_local(mcld_real A[4][4], const mcld_real b[4], mcld_real x[4]) {
    mcld_real M[4][5];
    for (int i=0;i<4;++i){ for(int j=0;j<4;++j) M[i][j]=A[i][j]; M[i][4]=b[i]; }
    for (int k=0;k<4;++k){
        int piv=k; mcld_real best=fabsl(M[k][k]);
        for(int i=k+1;i<4;++i){ mcld_real v=fabsl(M[i][k]); if(v>best){best=v; piv=i;} }
        if (piv!=k){ for(int j=k;j<5;++j){ mcld_real tmp=M[k][j]; M[k][j]=M[piv][j]; M[piv][j]=tmp; } }
        if (fabsl(M[k][k]) < 1e-18L) M[k][k] = (M[k][k] >= 0 ? 1e-18L : -1e-18L);
        mcld_real inv = 1.0L / M[k][k];
        for (int j=k;j<5;++j) M[k][j] *= inv;
        for (int i=0;i<4;++i){ if(i==k) continue; mcld_real fac=M[i][k]; for(int j=k;j<5;++j) M[i][j] -= fac*M[k][j]; }
    }
    for(int i=0;i<4;++i) x[i]=M[i][4];
}

static void retract_pair_full_to_local(const mcld_real y_base[8], const mcld_real z6[6], mcld_real out[8]) {
    mcld_real rv[3], dq[4], tmpq[4];
    rv[0]=z6[0]; rv[1]=z6[1]; rv[2]=z6[2];
    q_exp_rotvec_to_local(rv, dq);
    q_mul_inplace_local(y_base+0, dq, tmpq);
    memcpy(out+0, tmpq, 4*sizeof(mcld_real));
    rv[0]=z6[3]; rv[1]=z6[4]; rv[2]=z6[5];
    q_exp_rotvec_to_local(rv, dq);
    q_mul_inplace_local(y_base+4, dq, tmpq);
    memcpy(out+4, tmpq, 4*sizeof(mcld_real));
}

static void guess_z_from_half_momentum_local(const mcld_real y_cur[8], const mcld_real p_half[6], mcld_real h, const mcld_real params[11], mcld_real z_guess[6]);
static void interval_energy_unified_local(const mcld_real y0[8], const mcld_real y1[8], mcld_real h, const mcld_real params[11], mcld_real *U_out, mcld_real *H_out);

static void estimate_initial_energy_from_cfg(mcld_real *U0_out, mcld_real *H0_out) {
    mcld_real U0 = 0.0L, H0 = 0.0L;
    if (G.init_mode == MCLD_INIT_PAIR) {
        interval_energy_unified_local(G.cfg_y_prev, G.cfg_y_cur, G.sim_dt, G.params, &U0, &H0);
        memcpy(G.interval_y0, G.cfg_y_prev, sizeof(G.interval_y0));
        memcpy(G.interval_y1, G.cfg_y_cur, sizeof(G.interval_y1));
        G.has_interval = 1;
    } else {
        mcld_real z_est[6];
        const mcld_real *p_half = (G.init_mode == MCLD_INIT_HAMILTON_RIGHT ? G.cfg_p_right : G.cfg_p_left);
        guess_z_from_half_momentum_local(G.cfg_y_cur, p_half, G.sim_dt, G.params, z_est);
        mcld_real y_est[8];
        retract_pair_full_to_local(G.cfg_y_cur, z_est, y_est);
        interval_energy_unified_local(G.cfg_y_cur, y_est, G.sim_dt, G.params, &U0, &H0);
        memcpy(G.interval_y0, G.cfg_y_cur, sizeof(G.interval_y0));
        memcpy(G.interval_y1, y_est, sizeof(G.interval_y1));
        G.has_interval = 1;
    }
    if (U0_out) *U0_out = U0;
    if (H0_out) *H0_out = H0;
}

static void guess_z_from_half_momentum_local(const mcld_real y_cur[8], const mcld_real p_half[6], mcld_real h, const mcld_real params[11], mcld_real z_guess[6]) {
    const mcld_real l1=params[0], m1=params[2], m2=params[3], lc1=params[4], lc2=params[5], Iperp1=params[6], Iperp2=params[7], Ispin1=params[8], Ispin2=params[9];
    mcld_real R1[3][3],R2[3][3],R12[3][3],R21[3][3];
    q_to_R_inplace(y_cur,R1); q_to_R_inplace(y_cur+4,R2);
    for(int i=0;i<3;i++) for(int j=0;j<3;j++){ R12[i][j]=R1[0][i]*R2[0][j]+R1[1][i]*R2[1][j]+R1[2][i]*R2[2][j]; R21[j][i]=R12[i][j]; }
    const mcld_real a1=m1*lc1*lc1 + m2*l1*l1 + Iperp1, a2=m2*lc2*lc2 + Iperp2, ccoef=m2*l1*lc2;
    mcld_real A4[4][4]={{0}}; mcld_real b4[4]={p_half[0],p_half[1],p_half[3],p_half[4]};
    A4[0][0]=a1; A4[0][2]=ccoef*R12[1][1]; A4[0][3]=-ccoef*R12[1][0];
    A4[1][1]=a1; A4[1][2]=-ccoef*R12[0][1]; A4[1][3]=ccoef*R12[0][0];
    A4[2][0]=ccoef*R21[1][1]; A4[2][1]=-ccoef*R21[1][0]; A4[2][2]=a2;
    A4[3][0]=-ccoef*R21[0][1]; A4[3][1]=ccoef*R21[0][0]; A4[3][3]=a2;
    mcld_real u[4]; solve4_local(A4,b4,u);
    z_guess[0]=h*u[0]; z_guess[1]=h*u[1]; z_guess[2]=(fabsl(Ispin1)>1e-18L? h*(p_half[2]/Ispin1):0.0L);
    z_guess[3]=h*u[2]; z_guess[4]=h*u[3]; z_guess[5]=(fabsl(Ispin2)>1e-18L? h*(p_half[5]/Ispin2):0.0L);
}

static void rigid_hamiltonian_full_spin_interval(const mcld_real q1[4], const mcld_real q2[4], const mcld_real om1[3], const mcld_real om2[3], const mcld_real params[11], mcld_real *U_out, mcld_real *H_out) {
    const mcld_real l1=params[0], m1=params[2], m2=params[3], lc1=params[4], lc2=params[5], Iperp1=params[6], Iperp2=params[7], Ispin1=params[8], Ispin2=params[9], ggrav=params[10];
    mcld_real R1[3][3], R2[3][3], R12[3][3];
    q_to_R_inplace(q1, R1); q_to_R_inplace(q2, R2);
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) R12[i][j] = R1[0][i]*R2[0][j] + R1[1][i]*R2[1][j] + R1[2][i]*R2[2][j];
    mcld_real s1[3] = { om1[1], -om1[0], 0.0L };
    mcld_real s2[3] = { om2[1], -om2[0], 0.0L };
    mcld_real r12[3];
    r12[0] = R12[0][0]*s2[0] + R12[0][1]*s2[1] + R12[0][2]*s2[2];
    r12[1] = R12[1][0]*s2[0] + R12[1][1]*s2[1] + R12[1][2]*s2[2];
    r12[2] = R12[2][0]*s2[0] + R12[2][1]*s2[1] + R12[2][2]*s2[2];
    mcld_real n1[3] = { R1[2][0], R1[2][1], R1[2][2] };
    mcld_real n2[3] = { R2[2][0], R2[2][1], R2[2][2] };
    const mcld_real a1 = m1*lc1*lc1 + m2*l1*l1 + Iperp1;
    const mcld_real a2 = m2*lc2*lc2 + Iperp2;
    const mcld_real ccoef = m2*l1*lc2;
    mcld_real T = 0.5L * a1 * (om1[0]*om1[0] + om1[1]*om1[1]);
    T += 0.5L * a2 * (om2[0]*om2[0] + om2[1]*om2[1]);
    T += ccoef * (s1[0]*r12[0] + s1[1]*r12[1] + s1[2]*r12[2]);
    T += 0.5L * Ispin1 * om1[2]*om1[2] + 0.5L * Ispin2 * om2[2]*om2[2];
    mcld_real U = -ggrav * (m1 * lc1 * n1[2] + m2 * (l1 * n1[2] + lc2 * n2[2]));
    if (U_out) *U_out = U; if (H_out) *H_out = T + U;
}

static void interval_energy_unified_local(const mcld_real y0[8], const mcld_real y1[8], mcld_real h, const mcld_real params[11], mcld_real *U_out, mcld_real *H_out) {
    mcld_real ym[8];
    mcld_real qa[4], qc[4], qrel[4], rv[3], halfrv[3], dq[4];
    hemi_copy4_local(y0+0, y1+0, qa); q_conj_inplace_local(y0+0, qc); q_mul_inplace_local(qc, qa, qrel); q_log_rotvec_to_local(qrel, rv);
    halfrv[0]=0.5L*rv[0]; halfrv[1]=0.5L*rv[1]; halfrv[2]=0.5L*rv[2];
    q_exp_rotvec_to_local(halfrv, dq); q_mul_inplace_local(y0+0, dq, ym+0);
    mcld_real om1[3] = { rv[0]/h, rv[1]/h, rv[2]/h };
    hemi_copy4_local(y0+4, y1+4, qa); q_conj_inplace_local(y0+4, qc); q_mul_inplace_local(qc, qa, qrel); q_log_rotvec_to_local(qrel, rv);
    halfrv[0]=0.5L*rv[0]; halfrv[1]=0.5L*rv[1]; halfrv[2]=0.5L*rv[2];
    q_exp_rotvec_to_local(halfrv, dq); q_mul_inplace_local(y0+4, dq, ym+4);
    mcld_real om2[3] = { rv[0]/h, rv[1]/h, rv[2]/h };
    rigid_hamiltonian_full_spin_interval(ym+0, ym+4, om1, om2, params, U_out, H_out);
}

static void update_geometry(void) {
    mcld_real R1[3][3], R2[3][3];
    q_to_R_inplace(G.y_cur + 0, R1);
    q_to_R_inplace(G.y_cur + 4, R2);

    mcld_real u1[3] = { R1[0][0], R1[0][1], -R1[0][2] };
    mcld_real v1[3] = { R1[1][0], R1[1][1], -R1[1][2] };
    mcld_real n1[3] = { R1[2][0], R1[2][1], -R1[2][2] };

    mcld_real u2[3] = { R2[0][0], R2[0][1], -R2[0][2] };
    mcld_real v2[3] = { R2[1][0], R2[1][1], -R2[1][2] };
    mcld_real n2[3] = { R2[2][0], R2[2][1], -R2[2][2] };

    G.geom[0] = 0.0L; G.geom[1] = 0.0L; G.geom[2] = 0.0L;
    G.geom[3] = G.l1_vis * n1[0];
    G.geom[4] = G.l1_vis * n1[1];
    G.geom[5] = G.l1_vis * n1[2];
    G.geom[6] = G.geom[3] + G.l2_vis * n2[0];
    G.geom[7] = G.geom[4] + G.l2_vis * n2[1];
    G.geom[8] = G.geom[5] + G.l2_vis * n2[2];

    G.geom[9]  = n1[0]; G.geom[10] = n1[1]; G.geom[11] = n1[2];
    G.geom[12] = u1[0]; G.geom[13] = u1[1]; G.geom[14] = u1[2];
    G.geom[15] = v1[0]; G.geom[16] = v1[1]; G.geom[17] = v1[2];

    G.geom[18] = n2[0]; G.geom[19] = n2[1]; G.geom[20] = n2[2];
    G.geom[21] = u2[0]; G.geom[22] = u2[1]; G.geom[23] = u2[2];
    G.geom[24] = v2[0]; G.geom[25] = v2[1]; G.geom[26] = v2[2];
}

static void clear_runtime_state(void) {
    memset(G.y_prev, 0, sizeof(G.y_prev));
    memset(G.y_cur, 0, sizeof(G.y_cur));
    memset(G.y_next, 0, sizeof(G.y_next));
    memset(G.p_half, 0, sizeof(G.p_half));
    memset(G.p_next, 0, sizeof(G.p_next));
    memset(G.z_cur, 0, sizeof(G.z_cur));
    G.t = 0.0L;
    G.last_lib_rc = 1;
    G.last_best_residual = 0.0L;
    G.last_niters = 0;
    G.initial_potential = 0.0L;
    G.initial_energy = 0.0L;
    G.has_interval = 0;
    G.right_bootstrap_pending = 0;
}

static int apply_restore_from_saved(void) {
    if (!(G.step_mode == MCLD_STEP_HAMILTON_LEFT || G.step_mode == MCLD_STEP_HAMILTON_RIGHT)) return 2;

    clear_runtime_state();
    memcpy(G.y_prev, G.restore_y, sizeof(G.y_prev));
    memcpy(G.y_cur, G.restore_y, sizeof(G.y_cur));
    memcpy(G.p_half, G.restore_p, sizeof(G.p_half));
    guess_z_from_half_momentum_local(G.y_cur, G.p_half, G.sim_dt, G.params, G.z_cur);
    G.t = (G.restore_t >= 0.0L ? G.restore_t : 0.0L);
    G.initialized = 1;
    G.has_interval = 0;
    G.right_bootstrap_pending = 0;
    compute_energy_from_state(G.y_cur, G.z_cur, G.sim_dt, G.params, &G.initial_potential, NULL);
    if (isfinite(G.restore_H)) G.initial_energy = G.restore_H;
    else compute_energy_from_state(G.y_cur, G.z_cur, G.sim_dt, G.params, NULL, &G.initial_energy);
    update_geometry();
    return 0;
}

static int apply_init_from_cfg(void) {
    clear_runtime_state();

    if (G.cfg_use_omega) derive_cfg_momenta_from_omega_exact();

    if (G.step_mode == MCLD_STEP_LAGRANGE) {
        if (G.init_mode == MCLD_INIT_PAIR) {
            memcpy(G.y_prev, G.cfg_y_prev, sizeof(G.y_prev));
            memcpy(G.y_cur, G.cfg_y_cur, sizeof(G.y_cur));
            G.has_interval = 0;
            G.t = 0.0L;
            G.initialized = 1;
            update_geometry();
            estimate_initial_energy_from_cfg(&G.initial_potential, &G.initial_energy);
            return 0;
        } else if (G.init_mode == MCLD_INIT_HAMILTON_LEFT) {
            mcld_real y1[8], pplus[6], z0[6], wr = 0.0L, H0 = 0.0L, U0 = 0.0L;
            int nit = 0;
            int lib_rc;
            lib_rc = mcld_hamilton_step_from_left(
                G.cfg_y_cur, G.cfg_p_left, G.sim_dt, G.params, &G.opts,
                y1, pplus, z0, &wr, &nit
            );
            G.last_lib_rc = lib_rc;
            G.last_best_residual = wr;
            G.last_niters = nit;
            if (lib_rc == 0) return 3;
            memcpy(G.y_prev, G.cfg_y_cur, sizeof(G.y_prev));
            memcpy(G.y_cur, y1, sizeof(G.y_cur));
            memcpy(G.z_cur, z0, sizeof(G.z_cur));
            memcpy(G.interval_y0, G.cfg_y_cur, sizeof(G.interval_y0));
            memcpy(G.interval_y1, y1, sizeof(G.interval_y1));
            G.has_interval = 1;
            G.t = G.sim_dt;
            interval_energy_unified_local(G.cfg_y_cur, y1, G.sim_dt, G.params, &U0, &H0);
            G.initial_potential = U0;
            G.initial_energy = H0;
            G.initialized = 1;
            update_geometry();
            return 0;
        } else if (G.init_mode == MCLD_INIT_HAMILTON_RIGHT) {
            mcld_real y1[8], psame[6], z0[6], wr = 0.0L, H0 = 0.0L, U0 = 0.0L;
            int nit = 0;
            int lib_rc;
            lib_rc = mcld_hamilton_step_from_right(
                G.cfg_y_cur, G.cfg_p_right, G.sim_dt, G.params, &G.opts,
                y1, psame, z0, &wr, &nit
            );
            G.last_lib_rc = lib_rc;
            G.last_best_residual = wr;
            G.last_niters = nit;
            if (lib_rc == 0) return 3;
            memcpy(G.y_prev, G.cfg_y_cur, sizeof(G.y_prev));
            memcpy(G.y_cur, y1, sizeof(G.y_cur));
            memcpy(G.z_cur, z0, sizeof(G.z_cur));
            memcpy(G.interval_y0, G.cfg_y_cur, sizeof(G.interval_y0));
            memcpy(G.interval_y1, y1, sizeof(G.interval_y1));
            G.has_interval = 1;
            G.t = G.sim_dt;
            interval_energy_unified_local(G.cfg_y_cur, y1, G.sim_dt, G.params, &U0, &H0);
            G.initial_potential = U0;
            G.initial_energy = H0;
            G.initialized = 1;
            update_geometry();
            return 0;
        }
        return 2;
    }

    if (G.step_mode == MCLD_STEP_HAMILTON_LEFT) {
        memcpy(G.y_cur, G.cfg_y_cur, sizeof(G.y_cur));
        if (G.init_mode == MCLD_INIT_HAMILTON_LEFT) {
            memcpy(G.p_half, G.cfg_p_left, sizeof(G.p_half));
        } else if (G.init_mode == MCLD_INIT_PAIR) {
            mcld_real YL[(1 + 2) * 8];
            mcld_real ZL[(1 + 1) * 6];
            mcld_real PL[(1 + 1) * 6];
            mcld_real wr = 0.0L;
            int wi = 0, ti = 0;
            int lib_rc = mcld_legendre_integrate_trajectory_from_pair(
                G.cfg_y_prev, G.cfg_y_cur, G.sim_dt, 1,
                G.params, &G.opts,
                YL, ZL, PL, &wr, &wi, &ti
            );
            G.last_lib_rc = lib_rc;
            G.last_best_residual = wr;
            G.last_niters = ti;
            if (lib_rc == 0) return 3;
            memcpy(G.p_half, PL, sizeof(G.p_half));
        } else {
            return 2;
        }
        G.initialized = 1;
        G.right_bootstrap_pending = 0;
        update_geometry();
        estimate_initial_energy_from_cfg(&G.initial_potential, &G.initial_energy);
        return 0;
    }

    if (G.step_mode == MCLD_STEP_HAMILTON_RIGHT) {
        memcpy(G.y_cur, G.cfg_y_cur, sizeof(G.y_cur));
        if (G.init_mode == MCLD_INIT_HAMILTON_RIGHT) {
            memcpy(G.p_half, G.cfg_p_right, sizeof(G.p_half));
        } else if (G.init_mode == MCLD_INIT_PAIR) {
            mcld_real YL[(1 + 2) * 8];
            mcld_real ZL[(1 + 1) * 6];
            mcld_real PL[(1 + 1) * 6];
            mcld_real wr = 0.0L;
            int wi = 0, ti = 0;
            int lib_rc = mcld_legendre_integrate_trajectory_from_pair(
                G.cfg_y_prev, G.cfg_y_cur, G.sim_dt, 1,
                G.params, &G.opts,
                YL, ZL, PL, &wr, &wi, &ti
            );
            G.last_lib_rc = lib_rc;
            G.last_best_residual = wr;
            G.last_niters = ti;
            if (lib_rc == 0) return 3;
            memcpy(G.p_half, PL, sizeof(G.p_half));
        } else {
            return 2;
        }
        G.initialized = 1;
        G.right_bootstrap_pending = 1;
        update_geometry();
        estimate_initial_energy_from_cfg(&G.initial_potential, &G.initial_energy);
        return 0;
    }

    return 2;
}

EMSCRIPTEN_KEEPALIVE
int mcld_wallpaper_init_default(double sim_dt) {
    memset(&G, 0, sizeof(G));
    G.sim_dt = (mcld_real)sim_dt;
    mcld_default_params(G.params, 0.003L, 0.001L);
    mcld_default_solver_opts(&G.opts);
    G.l1_vis = G.params[0];
    G.l2_vis = G.params[1];
    G.step_mode = MCLD_STEP_HAMILTON_LEFT;
    G.init_mode = MCLD_INIT_HAMILTON_LEFT;
    reset_cfg_defaults();
    return apply_init_from_cfg();
}

EMSCRIPTEN_KEEPALIVE
void mcld_wallpaper_configure(int step_mode, int init_mode, double sim_dt, int max_iter, double tol, double beta) {
    if (step_mode >= MCLD_STEP_LAGRANGE && step_mode <= MCLD_STEP_HAMILTON_RIGHT) G.step_mode = step_mode;
    if (init_mode >= MCLD_INIT_PAIR && init_mode <= MCLD_INIT_HAMILTON_RIGHT) G.init_mode = init_mode;
    if (sim_dt > 0.0) G.sim_dt = (mcld_real)sim_dt;
    if (max_iter > 0) G.opts.max_iter = max_iter;
    if (tol > 0.0) G.opts.tol = (mcld_real)tol;
    G.opts.beta = (mcld_real)beta;
}

EMSCRIPTEN_KEEPALIVE
void mcld_wallpaper_config_set_real(int slot, int idx, double value) {
    mcld_real v = (mcld_real)value;
    if (slot == MCLD_CFG_Y_PREV && idx >= 0 && idx < 8) G.cfg_y_prev[idx] = v;
    else if (slot == MCLD_CFG_Y_CUR && idx >= 0 && idx < 8) G.cfg_y_cur[idx] = v;
    else if (slot == MCLD_CFG_P_LEFT && idx >= 0 && idx < 6) G.cfg_p_left[idx] = v;
    else if (slot == MCLD_CFG_P_RIGHT && idx >= 0 && idx < 6) G.cfg_p_right[idx] = v;
    else if (slot == MCLD_CFG_OMEGA && idx >= 0 && idx < 6) G.cfg_omega[idx] = v;
    else if (slot == MCLD_RESTORE_Y && idx >= 0 && idx < 8) G.restore_y[idx] = v;
    else if (slot == MCLD_RESTORE_P && idx >= 0 && idx < 6) G.restore_p[idx] = v;
}

EMSCRIPTEN_KEEPALIVE
void mcld_wallpaper_param_set(int idx, double value) {
    if (idx < 0 || idx >= 11) return;
    G.params[idx] = (mcld_real)value;
}

EMSCRIPTEN_KEEPALIVE
void mcld_wallpaper_config_set_int(int key, int value) {
    if (key == MCLD_FLAG_USE_CFG_OMEGA) G.cfg_use_omega = (value != 0);
}

EMSCRIPTEN_KEEPALIVE
int mcld_wallpaper_apply_init(void) {
    return apply_init_from_cfg();
}

EMSCRIPTEN_KEEPALIVE
void mcld_wallpaper_restore_meta(double t, double H) {
    G.restore_t = (mcld_real)t;
    G.restore_H = (mcld_real)H;
}


EMSCRIPTEN_KEEPALIVE
int mcld_wallpaper_real_byte_size(void) {
    return (int)sizeof(mcld_real);
}

EMSCRIPTEN_KEEPALIVE
const char *mcld_wallpaper_persist_get_hex(int slot, int idx) {
    static const char EMPTY[] = "";
    if (slot == MCLD_PERSIST_Y_CUR) {
        if (idx < 0 || idx >= 8) return EMPTY;
        return real_to_hex_cstr_local(G.y_cur[idx]);
    }
    if (slot == MCLD_PERSIST_P_HALF) {
        if (idx < 0 || idx >= 6) return EMPTY;
        return real_to_hex_cstr_local(G.p_half[idx]);
    }
    if (slot == MCLD_PERSIST_TIME) {
        return real_to_hex_cstr_local(G.t);
    }
    if (slot == MCLD_PERSIST_ENERGY_CURRENT) {
        mcld_real U = 0.0L, H = 0.0L;
        if (G.has_interval) interval_energy_unified_local(G.interval_y0, G.interval_y1, G.sim_dt, G.params, &U, &H);
        else compute_energy_from_state(G.y_cur, G.z_cur, G.sim_dt, G.params, &U, &H);
        return real_to_hex_cstr_local(H);
    }
    return EMPTY;
}

EMSCRIPTEN_KEEPALIVE
int mcld_wallpaper_restore_set_hex(int slot, int idx, const char *hex) {
    mcld_real value = 0.0L;
    if (!real_from_hex_cstr_local(hex, &value)) return 0;
    if (slot == MCLD_RESTORE_HEX_Y) {
        if (idx < 0 || idx >= 8) return 0;
        G.restore_y[idx] = value;
        return 1;
    }
    if (slot == MCLD_RESTORE_HEX_P) {
        if (idx < 0 || idx >= 6) return 0;
        G.restore_p[idx] = value;
        return 1;
    }
    return 0;
}

EMSCRIPTEN_KEEPALIVE
int mcld_wallpaper_restore_meta_hex(const char *t_hex, const char *H_hex) {
    mcld_real t_val = 0.0L;
    mcld_real H_val = 0.0L;
    if (!real_from_hex_cstr_local(t_hex, &t_val)) return 0;
    if (!real_from_hex_cstr_local(H_hex, &H_val)) return 0;
    G.restore_t = t_val;
    G.restore_H = H_val;
    return 1;
}

EMSCRIPTEN_KEEPALIVE
int mcld_wallpaper_apply_restore(void) {
    return apply_restore_from_saved();
}

EMSCRIPTEN_KEEPALIVE
void mcld_wallpaper_set_visual_lengths(double l1, double l2) {
    G.l1_vis = (mcld_real)l1;
    G.l2_vis = (mcld_real)l2;
    update_geometry();
}

EMSCRIPTEN_KEEPALIVE
int mcld_wallpaper_step_n(int n) {
    if (!G.initialized) return 2;
    if (n < 1) return 0;

    int any_warning = 0;
    for (int i = 0; i < n; ++i) {
        mcld_real best_res = 0.0L;
        int niters = 0;
        int lib_rc = 0;
        mcld_real y_old[8];
        memcpy(y_old, G.y_cur, sizeof(y_old));

        if (G.step_mode == MCLD_STEP_LAGRANGE) {
            lib_rc = mcld_legendre_step_from_pair(
                G.y_prev, G.y_cur, G.sim_dt, G.params, &G.opts,
                G.y_next, G.z_cur, G.p_next, &best_res, &niters
            );
            memcpy(G.y_prev, G.y_cur, sizeof(G.y_prev));
            memcpy(G.y_cur, G.y_next, sizeof(G.y_cur));
            memcpy(G.interval_y0, y_old, sizeof(G.interval_y0));
            memcpy(G.interval_y1, G.y_cur, sizeof(G.interval_y1));
            G.has_interval = 1;
        } else if (G.step_mode == MCLD_STEP_HAMILTON_LEFT) {
            lib_rc = mcld_hamilton_step_from_left(
                G.y_cur, G.p_half, G.sim_dt, G.params, &G.opts,
                G.y_next, G.p_next, G.z_cur, &best_res, &niters
            );
            memcpy(G.y_cur, G.y_next, sizeof(G.y_cur));
            memcpy(G.p_half, G.p_next, sizeof(G.p_half));
            memcpy(G.interval_y0, y_old, sizeof(G.interval_y0));
            memcpy(G.interval_y1, G.y_cur, sizeof(G.interval_y1));
            G.has_interval = 1;
        } else if (G.step_mode == MCLD_STEP_HAMILTON_RIGHT) {
            if (G.right_bootstrap_pending) {
                lib_rc = mcld_hamilton_step_from_right(
                    G.y_cur, G.p_half, G.sim_dt, G.params, &G.opts,
                    G.y_next, G.p_next, G.z_cur, &best_res, &niters
                );
                memcpy(G.y_cur, G.y_next, sizeof(G.y_cur));
                memcpy(G.p_half, G.p_next, sizeof(G.p_half));
                G.right_bootstrap_pending = 0;
            } else {
                lib_rc = mcld_hamilton_step_from_left(
                    G.y_cur, G.p_half, G.sim_dt, G.params, &G.opts,
                    G.y_next, G.p_next, G.z_cur, &best_res, &niters
                );
                memcpy(G.y_cur, G.y_next, sizeof(G.y_cur));
                memcpy(G.p_half, G.p_next, sizeof(G.p_half));
            }
            memcpy(G.interval_y0, y_old, sizeof(G.interval_y0));
            memcpy(G.interval_y1, G.y_cur, sizeof(G.interval_y1));
            G.has_interval = 1;
        } else {
            return 2;
        }

        G.last_lib_rc = lib_rc;
        G.last_best_residual = best_res;
        G.last_niters = niters;
        G.t += G.sim_dt;
        if (lib_rc == 0) any_warning = 1;
    }

    update_geometry();
    return any_warning ? 1 : 0;
}

EMSCRIPTEN_KEEPALIVE
double mcld_wallpaper_geometry_at(int idx) {
    if (idx < 0 || idx >= 27) return 0.0;
    return (double)G.geom[idx];
}

EMSCRIPTEN_KEEPALIVE
double mcld_wallpaper_get_time(void) {
    return (double)G.t;
}

EMSCRIPTEN_KEEPALIVE
int mcld_wallpaper_get_last_lib_rc(void) {
    return G.last_lib_rc;
}

EMSCRIPTEN_KEEPALIVE
double mcld_wallpaper_get_last_residual(void) {
    return (double)G.last_best_residual;
}

EMSCRIPTEN_KEEPALIVE
int mcld_wallpaper_get_last_niters(void) {
    return G.last_niters;
}


EMSCRIPTEN_KEEPALIVE
double mcld_wallpaper_y_at(int idx) {
    if (idx < 0 || idx >= 8) return 0.0;
    return (double)G.y_cur[idx];
}

EMSCRIPTEN_KEEPALIVE
double mcld_wallpaper_z_at(int idx) {
    if (idx < 0 || idx >= 6) return 0.0;
    return (double)G.z_cur[idx];
}

EMSCRIPTEN_KEEPALIVE
double mcld_wallpaper_p_at(int idx) {
    if (idx < 0 || idx >= 6) return 0.0;
    return (double)G.p_half[idx];
}

EMSCRIPTEN_KEEPALIVE
double mcld_wallpaper_get_potential_current(void) {
    mcld_real U = 0.0L, H = 0.0L;
    if (G.has_interval) interval_energy_unified_local(G.interval_y0, G.interval_y1, G.sim_dt, G.params, &U, &H);
    else compute_energy_from_state(G.y_cur, G.z_cur, G.sim_dt, G.params, &U, &H);
    return (double)U;
}

EMSCRIPTEN_KEEPALIVE
double mcld_wallpaper_get_energy_current(void) {
    mcld_real U = 0.0L, H = 0.0L;
    if (G.has_interval) interval_energy_unified_local(G.interval_y0, G.interval_y1, G.sim_dt, G.params, &U, &H);
    else compute_energy_from_state(G.y_cur, G.z_cur, G.sim_dt, G.params, &U, &H);
    return (double)H;
}

EMSCRIPTEN_KEEPALIVE
double mcld_wallpaper_get_potential_initial(void) {
    return (double)G.initial_potential;
}

EMSCRIPTEN_KEEPALIVE
double mcld_wallpaper_get_energy_initial(void) {
    return (double)G.initial_energy;
}

EMSCRIPTEN_KEEPALIVE
double mcld_wallpaper_get_qerr(void) {
    mcld_real qmax = 0.0L;
    compute_qerr_from_y(G.y_cur, NULL, NULL, &qmax);
    return (double)qmax;
}

EMSCRIPTEN_KEEPALIVE
double mcld_wallpaper_get_q1err(void) {
    mcld_real q1 = 0.0L;
    compute_qerr_from_y(G.y_cur, &q1, NULL, NULL);
    return (double)q1;
}

EMSCRIPTEN_KEEPALIVE
double mcld_wallpaper_get_q2err(void) {
    mcld_real q2 = 0.0L;
    compute_qerr_from_y(G.y_cur, NULL, &q2, NULL);
    return (double)q2;
}
