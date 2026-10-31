#include "midpoint_combined_ld.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef MDP_MIN
#define MDP_MIN(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef MDP_MAX
#define MDP_MAX(a,b) ((a)>(b)?(a):(b))
#endif

typedef long double ld;

static void copy8(const ld *src, ld *dst){ for(int i=0;i<8;i++) dst[i]=src[i]; }
static void copy6(const ld *src, ld *dst){ for(int i=0;i<6;i++) dst[i]=src[i]; }
static ld norm6_sq(const ld *x){ ld s=0; for(int i=0;i<6;i++) s += x[i]*x[i]; return s; }

void mcld_default_params(ld params_out[11], ld Ispin1, ld Ispin2){
    ld p[11] = {0.5L,1.0L,3.0L,1.0L,0.25L,0.5L,0.04L,0.05L,Ispin1,Ispin2,9.81L};
    for(int i=0;i<11;i++) params_out[i]=p[i];
}
void mcld_default_solver_opts(mcld_solver_opts *opts){
    if(!opts) return;
    opts->beta = 1.0L;
    opts->max_iter = 6;
    opts->tol = 1.5e-11L;
}

static void q_normalize_to(const ld q[4], ld out[4]){
    ld n2 = q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3];
    if(n2 < 1e-30L){ out[0]=1; out[1]=out[2]=out[3]=0; return; }
    ld inv = 1.0L/sqrtl(n2);
    for(int i=0;i<4;i++) out[i]=q[i]*inv;
}
static void q_conj_inplace(const ld q[4], ld out[4]){ out[0]=q[0]; out[1]=-q[1]; out[2]=-q[2]; out[3]=-q[3]; }
static void q_mul_inplace(const ld a[4], const ld b[4], ld out[4]){
    ld a0=a[0],a1=a[1],a2=a[2],a3=a[3];
    ld b0=b[0],b1=b[1],b2=b[2],b3=b[3];
    out[0]=a0*b0-a1*b1-a2*b2-a3*b3;
    out[1]=a0*b1+a1*b0+a2*b3-a3*b2;
    out[2]=a0*b2-a1*b3+a2*b0+a3*b1;
    out[3]=a0*b3+a1*b2-a2*b1+a3*b0;
}
static void q_exp_rotvec_to(const ld v[3], ld out[4]){
    ld ang = sqrtl(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    ld s;
    if(ang < 1e-14L){
        ld a2 = ang*ang;
        out[0]=1.0L-0.125L*a2;
        s = 0.5L - a2/48.0L;
    } else {
        ld half = 0.5L*ang;
        out[0]=cosl(half);
        s = sinl(half)/ang;
    }
    out[1]=s*v[0]; out[2]=s*v[1]; out[3]=s*v[2];
}
static void q_log_rotvec_to(const ld q[4], ld out[3]){
    ld qq[4]; q_normalize_to(q,qq);
    ld w=qq[0], vx=qq[1], vy=qq[2], vz=qq[3];
    if(w>1) w=1; else if(w<-1) w=-1;
    ld s = sqrtl(vx*vx+vy*vy+vz*vz);
    if(s < 1e-14L){ out[0]=2*vx; out[1]=2*vy; out[2]=2*vz; return; }
    ld ang = 2.0L*atan2l(s,w);
    ld fac = ang/s;
    out[0]=fac*vx; out[1]=fac*vy; out[2]=fac*vz;
}
static void hemi_copy4(const ld qref[4], const ld qin[4], ld qout[4]){
    ld qr[4], qi[4]; q_normalize_to(qref,qr); q_normalize_to(qin,qi);
    ld s=0; for(int i=0;i<4;i++) s += qr[i]*qi[i];
    if(s<0){ for(int i=0;i<4;i++) qout[i]=-qin[i]; }
    else { for(int i=0;i<4;i++) qout[i]=qin[i]; }
}
static void q_to_R_inplace(const ld q[4], ld R[3][3]){
    ld qq[4]; q_normalize_to(q,qq);
    ld w=qq[0], x=qq[1], y=qq[2], z=qq[3];
    R[0][0]=1-2*(y*y+z*z); R[0][1]=2*(x*y-w*z); R[0][2]=2*(x*z+w*y);
    R[1][0]=2*(x*y+w*z); R[1][1]=1-2*(x*x+z*z); R[1][2]=2*(y*z-w*x);
    R[2][0]=2*(x*z-w*y); R[2][1]=2*(y*z+w*x); R[2][2]=1-2*(x*x+y*y);
}
static void cross3(const ld a[3], const ld b[3], ld out[3]){
    out[0]=a[1]*b[2]-a[2]*b[1];
    out[1]=a[2]*b[0]-a[0]*b[2];
    out[2]=a[0]*b[1]-a[1]*b[0];
}
static void matvec3(const ld M[3][3], const ld v[3], ld out[3]){
    for(int i=0;i<3;i++) out[i]=M[i][0]*v[0]+M[i][1]*v[1]+M[i][2]*v[2];
}
static void matTvec3(const ld M[3][3], const ld v[3], ld out[3]){
    for(int i=0;i<3;i++) out[i]=M[0][i]*v[0]+M[1][i]*v[1]+M[2][i]*v[2];
}
static void matmul3(const ld A[3][3], const ld B[3][3], ld C[3][3]){
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) C[i][j]=A[i][0]*B[0][j]+A[i][1]*B[1][j]+A[i][2]*B[2][j];
}
static void hat_inplace(const ld v[3], ld H[3][3]){
    H[0][0]=0; H[0][1]=-v[2]; H[0][2]= v[1];
    H[1][0]=v[2]; H[1][1]=0; H[1][2]=-v[0];
    H[2][0]=-v[1]; H[2][1]=v[0]; H[2][2]=0;
}
static void Jr_to(const ld phi[3], ld J[3][3]){
    ld theta = sqrtl(phi[0]*phi[0]+phi[1]*phi[1]+phi[2]*phi[2]);
    ld H[3][3], H2[3][3]; hat_inplace(phi,H); matmul3(H,H,H2);
    if(theta < 1e-12L){
        for(int i=0;i<3;i++) for(int j=0;j<3;j++) J[i][j]=(i==j?1.0L:0.0L)-0.5L*H[i][j]+(1.0L/6.0L)*H2[i][j];
        return;
    }
    ld a = (1-cosl(theta))/(theta*theta);
    ld b = (theta-sinl(theta))/(theta*theta*theta);
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) J[i][j]=(i==j?1.0L:0.0L)-a*H[i][j]+b*H2[i][j];
}
static void dJr_to(const ld phi[3], const ld eta[3], ld dJ[3][3]){
    ld theta = sqrtl(phi[0]*phi[0]+phi[1]*phi[1]+phi[2]*phi[2]);
    ld H[3][3], dH[3][3], H2[3][3], dHH[3][3], HdH[3][3];
    hat_inplace(phi,H); hat_inplace(eta,dH); matmul3(H,H,H2); matmul3(dH,H,dHH); matmul3(H,dH,HdH);
    if(theta < 1e-8L){
        for(int i=0;i<3;i++) for(int j=0;j<3;j++) dJ[i][j]=-0.5L*dH[i][j]+(1.0L/6.0L)*(dHH[i][j]+HdH[i][j]);
        return;
    }
    ld a=(1-cosl(theta))/(theta*theta), b=(theta-sinl(theta))/(theta*theta*theta);
    ld da=(theta*sinl(theta)-2*(1-cosl(theta)))/(theta*theta*theta);
    ld db=(theta*(1-cosl(theta))-3*(theta-sinl(theta)))/(theta*theta*theta*theta);
    ld coeff=(phi[0]*eta[0]+phi[1]*eta[1]+phi[2]*eta[2])/theta;
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) dJ[i][j] = -da*coeff*H[i][j] - a*dH[i][j] + db*coeff*H2[i][j] + b*(dHH[i][j]+HdH[i][j]);
}
static void solve3(const ld A[3][3], const ld b[3], ld x[3]){
    ld M[3][4];
    for(int i=0;i<3;i++){ for(int j=0;j<3;j++) M[i][j]=A[i][j]; M[i][3]=b[i]; }
    for(int k=0;k<3;k++){
        int piv=k; ld best=fabsl(M[k][k]);
        for(int i=k+1;i<3;i++){ ld v=fabsl(M[i][k]); if(v>best){best=v; piv=i;} }
        if(piv!=k) for(int j=k;j<4;j++){ ld tmp=M[k][j]; M[k][j]=M[piv][j]; M[piv][j]=tmp; }
        if(fabsl(M[k][k])<1e-24L) M[k][k]=(M[k][k]>=0?1e-24L:-1e-24L);
        ld inv=1.0L/M[k][k];
        for(int j=k;j<4;j++) M[k][j]*=inv;
        for(int i=0;i<3;i++) if(i!=k){ ld fac=M[i][k]; for(int j=k;j<4;j++) M[i][j]-=fac*M[k][j]; }
    }
    x[0]=M[0][3]; x[1]=M[1][3]; x[2]=M[2][3];
}
static void invT_matvec3(const ld A[3][3], const ld b[3], ld x[3]){
    ld AT[3][3]; for(int i=0;i<3;i++) for(int j=0;j<3;j++) AT[i][j]=A[j][i]; solve3(AT,b,x);
}
static void dJr_invT_times_vec(const ld phi[3], const ld eta[3], const ld vec[3], ld out[3]){
    ld J[3][3], dJ[3][3], tmp[3], tmp2[3];
    Jr_to(phi,J); dJr_to(phi,eta,dJ); invT_matvec3(J,vec,tmp);
    tmp2[0]=dJ[0][0]*tmp[0]+dJ[1][0]*tmp[1]+dJ[2][0]*tmp[2];
    tmp2[1]=dJ[0][1]*tmp[0]+dJ[1][1]*tmp[1]+dJ[2][1]*tmp[2];
    tmp2[2]=dJ[0][2]*tmp[0]+dJ[1][2]*tmp[1]+dJ[2][2]*tmp[2];
    invT_matvec3(J,tmp2,out); out[0]=-out[0]; out[1]=-out[1]; out[2]=-out[2];
}

static void retract_pair_full_to(const ld y_base[8], const ld z6[6], ld out[8]){
    ld rv[3], dq[4], tmpq[4];
    rv[0]=z6[0]; rv[1]=z6[1]; rv[2]=z6[2]; q_exp_rotvec_to(rv,dq); q_mul_inplace(y_base,dq,tmpq); for(int i=0;i<4;i++) out[i]=tmpq[i];
    rv[0]=z6[3]; rv[1]=z6[4]; rv[2]=z6[5]; q_exp_rotvec_to(rv,dq); q_mul_inplace(y_base+4,dq,tmpq); for(int i=0;i<4;i++) out[4+i]=tmpq[i];
}
static void predictor_secant_to(const ld prevprev[6], const ld prev[6], ld beta, ld out[6]){ for(int i=0;i<6;i++) out[i]=prev[i]+beta*(prev[i]-prevprev[i]); }
static void exp_step_from_omega_full_to(const ld y0[8], const ld om1_0[3], const ld om2_0[3], ld h, ld out[8]){
    ld z[6] = {h*om1_0[0],h*om1_0[1],h*om1_0[2],h*om2_0[0],h*om2_0[1],h*om2_0[2]};
    retract_pair_full_to(y0,z,out);
}

static void build_interval_cache_full(const ld y0[8], const ld y1[8], ld h, const ld params[11],
                              ld phi1[3], ld phi2[3], ld qm1[4], ld qm2[4], ld om1[3], ld om2[3],
                              ld R1[3][3], ld R2[3][3], ld R12[3][3], ld R21[3][3],
                              ld s1[3], ld s2[3], ld n1[3], ld n2[3], ld r12[3], ld r21[3],
                              ld mu1[3], ld mu2[3], ld g1[3], ld g2[3],
                              ld J1[3][3], ld J2[3][3], ld S1[3][3], ld S2[3][3], ld Rfull1[3][3], ld Rfull2[3][3], ld Rhalf1[3][3], ld Rhalf2[3][3],
                              ld eta1[3], ld eta2[3], ld D1_1[3], ld D1_2[3], ld D2_1[3], ld D2_2[3]){
    ld l1=params[0], m1=params[2], m2=params[3], lc1=params[4], lc2=params[5], Iperp1=params[6], Iperp2=params[7], Ispin1=params[8], Ispin2=params[9], ggrav=params[10];
    ld qa[4], qc[4], qrel[4], rv[3], dq[4], tmp[3];

    hemi_copy4(y0, y1, qa); q_conj_inplace(y0,qc); q_mul_inplace(qc,qa,qrel); q_log_rotvec_to(qrel,phi1);
    hemi_copy4(y0+4, y1+4, qa); q_conj_inplace(y0+4,qc); q_mul_inplace(qc,qa,qrel); q_log_rotvec_to(qrel,phi2);

    rv[0]=0.5L*phi1[0]; rv[1]=0.5L*phi1[1]; rv[2]=0.5L*phi1[2]; q_exp_rotvec_to(rv,dq); q_mul_inplace(y0,dq,qm1);
    rv[0]=0.5L*phi2[0]; rv[1]=0.5L*phi2[1]; rv[2]=0.5L*phi2[2]; q_exp_rotvec_to(rv,dq); q_mul_inplace(y0+4,dq,qm2);

    for(int i=0;i<3;i++){ om1[i]=phi1[i]/h; om2[i]=phi2[i]/h; }
    q_to_R_inplace(qm1,R1); q_to_R_inplace(qm2,R2);
    for(int i=0;i<3;i++) for(int j=0;j<3;j++){
        R12[i][j]=R1[0][i]*R2[0][j]+R1[1][i]*R2[1][j]+R1[2][i]*R2[2][j];
        R21[i][j]=R2[0][i]*R1[0][j]+R2[1][i]*R1[1][j]+R2[2][i]*R1[2][j];
    }
    s1[0]=om1[1]; s1[1]=-om1[0]; s1[2]=0;
    s2[0]=om2[1]; s2[1]=-om2[0]; s2[2]=0;
    n1[0]=R1[2][0]; n1[1]=R1[2][1]; n1[2]=R1[2][2];
    n2[0]=R2[2][0]; n2[1]=R2[2][1]; n2[2]=R2[2][2];
    matvec3(R12,s2,r12); matvec3(R21,s1,r21);

    ld a1 = m1*lc1*lc1 + m2*l1*l1 + Iperp1;
    ld a2 = m2*lc2*lc2 + Iperp2;
    ld ccoef = m2*l1*lc2;
    ld alpha1 = ggrav*(m1*lc1 + m2*l1);
    ld alpha2 = ggrav*m2*lc2;
    mu1[0]=a1*om1[0] - ccoef*(R12[1][0]*s2[0]+R12[1][1]*s2[1]+R12[1][2]*s2[2]);
    mu1[1]=a1*om1[1] + ccoef*(R12[0][0]*s2[0]+R12[0][1]*s2[1]+R12[0][2]*s2[2]);
    mu1[2]=Ispin1*om1[2];
    mu2[0]=a2*om2[0] - ccoef*(R21[1][0]*s1[0]+R21[1][1]*s1[1]+R21[1][2]*s1[2]);
    mu2[1]=a2*om2[1] + ccoef*(R21[0][0]*s1[0]+R21[0][1]*s1[1]+R21[0][2]*s1[2]);
    mu2[2]=Ispin2*om2[2];

    g1[0]=-alpha1*n1[1]; g1[1]=alpha1*n1[0]; g1[2]=0; cross3(s1,r12,tmp); g1[0]+=ccoef*tmp[0]; g1[1]+=ccoef*tmp[1]; g1[2]+=ccoef*tmp[2];
    g2[0]=-alpha2*n2[1]; g2[1]=alpha2*n2[0]; g2[2]=0; cross3(s2,r21,tmp); g2[0]+=ccoef*tmp[0]; g2[1]+=ccoef*tmp[1]; g2[2]+=ccoef*tmp[2];

    Jr_to(phi1,J1); Jr_to(phi2,J2);
    rv[0]=0.5L*phi1[0]; rv[1]=0.5L*phi1[1]; rv[2]=0.5L*phi1[2]; Jr_to(rv,S1);
    rv[0]=0.5L*phi2[0]; rv[1]=0.5L*phi2[1]; rv[2]=0.5L*phi2[2]; Jr_to(rv,S2);
    q_exp_rotvec_to(phi1,dq); q_to_R_inplace(dq,Rfull1);
    q_exp_rotvec_to(phi2,dq); q_to_R_inplace(dq,Rfull2);
    rv[0]=0.5L*phi1[0]; rv[1]=0.5L*phi1[1]; rv[2]=0.5L*phi1[2]; q_exp_rotvec_to(rv,dq); q_to_R_inplace(dq,Rhalf1);
    rv[0]=0.5L*phi2[0]; rv[1]=0.5L*phi2[1]; rv[2]=0.5L*phi2[2]; q_exp_rotvec_to(rv,dq); q_to_R_inplace(dq,Rhalf2);

    eta1[0]=mu1[0]+0.5L*h*(S1[0][0]*g1[0]+S1[1][0]*g1[1]+S1[2][0]*g1[2]);
    eta1[1]=mu1[1]+0.5L*h*(S1[0][1]*g1[0]+S1[1][1]*g1[1]+S1[2][1]*g1[2]);
    eta1[2]=mu1[2]+0.5L*h*(S1[0][2]*g1[0]+S1[1][2]*g1[1]+S1[2][2]*g1[2]);
    eta2[0]=mu2[0]+0.5L*h*(S2[0][0]*g2[0]+S2[1][0]*g2[1]+S2[2][0]*g2[2]);
    eta2[1]=mu2[1]+0.5L*h*(S2[0][1]*g2[0]+S2[1][1]*g2[1]+S2[2][1]*g2[2]);
    eta2[2]=mu2[2]+0.5L*h*(S2[0][2]*g2[0]+S2[1][2]*g2[1]+S2[2][2]*g2[2]);

    invT_matvec3(J1,eta1,D2_1); invT_matvec3(J2,eta2,D2_2);
    matvec3(Rhalf1,g1,tmp); for(int i=0;i<3;i++) D1_1[i]=h*tmp[i]; matvec3(Rfull1,D2_1,tmp); for(int i=0;i<3;i++) D1_1[i]-=tmp[i];
    matvec3(Rhalf2,g2,tmp); for(int i=0;i<3;i++) D1_2[i]=h*tmp[i]; matvec3(Rfull2,D2_2,tmp); for(int i=0;i<3;i++) D1_2[i]-=tmp[i];
}

static void rod_delta_quantities(ld h, const ld dphi[3],
                         const ld phi[3], const ld S[3][3], const ld Rhalf[3][3], const ld Rfull[3][3], const ld g[3], const ld eta[3], const ld D2[3], const ld rself[3], const ld nself[3], const ld sself[3],
                         ld acoef, ld ccoef, ld alpha, ld Ispin,
                         const ld Jself[3][3], ld dD1[3], ld dD2[3]){
    ld v[3], tmp[3]; matvec3(S,dphi,v); for(int i=0;i<3;i++) v[i]*=0.5L;
    ld domega0=dphi[0]/h, domega1=dphi[1]/h, domega2=dphi[2]/h, ds0=domega1, ds1=-domega0;
    cross3(v,rself,tmp);
    ld dmu0=acoef*domega0+ccoef*tmp[1], dmu1=acoef*domega1-ccoef*tmp[0], dmu2=Ispin*domega2;
    ld tmpn0=nself[1]*v[2]-nself[2]*v[1], tmpn1=nself[2]*v[0]-nself[0]*v[2];
    ld dg0=-alpha*tmpn1, dg1=alpha*tmpn0, dg2=0.0L;
    ld tx0 = ds1*rself[2], tx1 = -ds0*rself[2], tx2 = ds0*rself[1]-ds1*rself[0];
    dg0 += ccoef*tx0; dg1 += ccoef*tx1; dg2 += ccoef*tx2;
    tmp[0]=-tmp[0]; tmp[1]=-tmp[1]; tmp[2]=-tmp[2];
    tx0=sself[1]*tmp[2]-sself[2]*tmp[1]; tx1=sself[2]*tmp[0]-sself[0]*tmp[2]; tx2=sself[0]*tmp[1]-sself[1]*tmp[0];
    dg0 += ccoef*tx0; dg1 += ccoef*tx1; dg2 += ccoef*tx2;
    ld halfphi[3]={0.5L*phi[0],0.5L*phi[1],0.5L*phi[2]}, halfe[3]={0.5L*dphi[0],0.5L*dphi[1],0.5L*dphi[2]}, dJ[3][3];
    dJr_to(halfphi,halfe,dJ);
    ld dSTg0=dJ[0][0]*g[0]+dJ[1][0]*g[1]+dJ[2][0]*g[2];
    ld dSTg1=dJ[0][1]*g[0]+dJ[1][1]*g[1]+dJ[2][1]*g[2];
    ld dSTg2=dJ[0][2]*g[0]+dJ[1][2]*g[1]+dJ[2][2]*g[2];
    ld deta[3];
    deta[0]=dmu0+0.5L*h*(dSTg0+S[0][0]*dg0+S[1][0]*dg1+S[2][0]*dg2);
    deta[1]=dmu1+0.5L*h*(dSTg1+S[0][1]*dg0+S[1][1]*dg1+S[2][1]*dg2);
    deta[2]=dmu2+0.5L*h*(dSTg2+S[0][2]*dg0+S[1][2]*dg1+S[2][2]*dg2);
    ld dJinvTeta[3]; dJr_invT_times_vec(phi,dphi,eta,dJinvTeta); invT_matvec3(Jself,deta,dD2);
    for(int i=0;i<3;i++) dD2[i]+=dJinvTeta[i];
    tmp[0]=v[1]*g[2]-v[2]*g[1]+dg0; tmp[1]=v[2]*g[0]-v[0]*g[2]+dg1; tmp[2]=v[0]*g[1]-v[1]*g[0]+dg2;
    matvec3(Rhalf,tmp,dD1); for(int i=0;i<3;i++) dD1[i]*=h;
    ld u[3]; matvec3(Jself,dphi,u);
    tmp[0]=u[1]*D2[2]-u[2]*D2[1]+dD2[0]; tmp[1]=u[2]*D2[0]-u[0]*D2[2]+dD2[1]; tmp[2]=u[0]*D2[1]-u[1]*D2[0]+dD2[2];
    matvec3(Rfull,tmp,u); for(int i=0;i<3;i++) dD1[i]-=u[i];
}

static void cross_rod_effect(ld h, const ld dphi[3], const ld Rcross[3][3], const ld sself[3], const ld sother[3], const ld Sself[3][3], const ld Sother[3][3], const ld Rhalf[3][3], const ld Rfull[3][3], const ld eta[3], const ld D2[3], const ld Jself[3][3], ld ccoef, ld dD1[3], ld dD2[3]){
    ld v[3]; matvec3(Sother,dphi,v); for(int i=0;i<3;i++) v[i]*=0.5L;
    ld ds0=dphi[1]/h, ds1=-dphi[0]/h;
    ld tmp[3], tmp2[3];
    tmp[0]=v[1]*sother[2]-v[2]*sother[1]+ds0;
    tmp[1]=v[2]*sother[0]-v[0]*sother[2]+ds1;
    tmp[2]=v[0]*sother[1]-v[1]*sother[0];
    matvec3(Rcross,tmp,tmp2);
    ld dmu0=-ccoef*tmp2[1], dmu1=ccoef*tmp2[0];
    ld dg0=ccoef*(sself[1]*tmp2[2]-sself[2]*tmp2[1]);
    ld dg1=ccoef*(sself[2]*tmp2[0]-sself[0]*tmp2[2]);
    ld dg2=ccoef*(sself[0]*tmp2[1]-sself[1]*tmp2[0]);
    ld deta[3];
    deta[0]=dmu0+0.5L*h*(Sself[0][0]*dg0+Sself[1][0]*dg1+Sself[2][0]*dg2);
    deta[1]=dmu1+0.5L*h*(Sself[0][1]*dg0+Sself[1][1]*dg1+Sself[2][1]*dg2);
    deta[2]=0.5L*h*(Sself[0][2]*dg0+Sself[1][2]*dg1+Sself[2][2]*dg2);
    invT_matvec3(Jself,deta,dD2);
    tmp[0]=dg0; tmp[1]=dg1; tmp[2]=dg2; matvec3(Rhalf,tmp,dD1); for(int i=0;i<3;i++) dD1[i]*=h; matvec3(Rfull,dD2,tmp); for(int i=0;i<3;i++) dD1[i]-=tmp[i];
}

static void endpoint_derivatives_full_cached(const ld y0[8], const ld y1[8], ld h, const ld params[11],
                                     ld phi1[3], ld phi2[3], ld qm1[4], ld qm2[4], ld om1[3], ld om2[3],
                                     ld R1[3][3], ld R2[3][3], ld R12[3][3], ld R21[3][3],
                                     ld s1[3], ld s2[3], ld n1[3], ld n2[3], ld r12[3], ld r21[3],
                                     ld mu1[3], ld mu2[3], ld g1[3], ld g2[3],
                                     ld J1[3][3], ld J2[3][3], ld S1[3][3], ld S2[3][3], ld Rfull1[3][3], ld Rfull2[3][3], ld Rhalf1[3][3], ld Rhalf2[3][3],
                                     ld eta1[3], ld eta2[3], ld D1_1[3], ld D1_2[3], ld D2_1[3], ld D2_2[3],
                                     ld D1[6], ld D2[6]){
    build_interval_cache_full(y0,y1,h,params,phi1,phi2,qm1,qm2,om1,om2,R1,R2,R12,R21,s1,s2,n1,n2,r12,r21,mu1,mu2,g1,g2,J1,J2,S1,S2,Rfull1,Rfull2,Rhalf1,Rhalf2,eta1,eta2,D1_1,D1_2,D2_1,D2_2);
    for(int i=0;i<3;i++){ D1[i]=D1_1[i]; D1[3+i]=D1_2[i]; D2[i]=D2_1[i]; D2[3+i]=D2_2[i]; }
}
static void residual_full_cached(const ld z6[6], const ld y_prev[8], const ld y_cur[8], ld h, const ld params[11], const ld D2_prev[6],
                             ld y_next[8],
                             ld phi1[3], ld phi2[3], ld qm1[4], ld qm2[4], ld om1[3], ld om2[3],
                             ld R1[3][3], ld R2[3][3], ld R12[3][3], ld R21[3][3],
                             ld s1[3], ld s2[3], ld n1[3], ld n2[3], ld r12[3], ld r21[3],
                             ld mu1[3], ld mu2[3], ld g1[3], ld g2[3],
                             ld J1[3][3], ld J2[3][3], ld S1[3][3], ld S2[3][3], ld Rfull1[3][3], ld Rfull2[3][3], ld Rhalf1[3][3], ld Rhalf2[3][3],
                             ld eta1[3], ld eta2[3], ld D1_1[3], ld D1_2[3], ld D2_1[3], ld D2_2[3],
                             ld D1tmp[6], ld D2tmp[6], ld r[6]){
    (void)y_prev; (void)D2tmp;
    retract_pair_full_to(y_cur,z6,y_next);
    endpoint_derivatives_full_cached(y_cur,y_next,h,params,phi1,phi2,qm1,qm2,om1,om2,R1,R2,R12,R21,s1,s2,n1,n2,r12,r21,mu1,mu2,g1,g2,J1,J2,S1,S2,Rfull1,Rfull2,Rhalf1,Rhalf2,eta1,eta2,D1_1,D1_2,D2_1,D2_2,D1tmp,D2tmp);
    for(int i=0;i<6;i++) r[i]=D2_prev[i]+D1tmp[i];
}
static void jacobian_full_cached(ld h,
                         const ld phi1[3], const ld phi2[3], const ld s1[3], const ld s2[3], const ld n1[3], const ld n2[3], const ld r12[3], const ld r21[3],
                         const ld R12[3][3], const ld R21[3][3],
                         const ld mu1[3], const ld mu2[3], const ld g1[3], const ld g2[3],
                         const ld J1[3][3], const ld J2[3][3], const ld S1[3][3], const ld S2[3][3], const ld Rfull1[3][3], const ld Rfull2[3][3], const ld Rhalf1[3][3], const ld Rhalf2[3][3],
                         const ld eta1[3], const ld eta2[3], const ld D2_1[3], const ld D2_2[3],
                         const ld params[11], ld J[6][6]){
    ld Ispin1=params[8], Ispin2=params[9];
    ld e[3], dD1[3], dD2[3], dX1[3], dX2[3];
    ld acoef1=params[2]*params[4]*params[4] + params[3]*params[0]*params[0] + params[6];
    ld acoef2=params[3]*params[5]*params[5] + params[7];
    ld ccoef=params[3]*params[0]*params[5];
    ld alpha1=params[10]*(params[2]*params[4] + params[3]*params[0]);
    ld alpha2=params[10]*(params[3]*params[5]);
    for(int i=0;i<6;i++) for(int j=0;j<6;j++) J[i][j]=0.0L;
    for(int j=0;j<3;j++){
        e[0]=e[1]=e[2]=0; e[j]=1;
        rod_delta_quantities(h,e,phi1,S1,Rhalf1,Rfull1,g1,eta1,D2_1,r12,n1,s1,acoef1,ccoef,alpha1,Ispin1,J1,dD1,dD2);
        cross_rod_effect(h,e,R21,s2,s1,S2,S1,Rhalf2,Rfull2,eta2,D2_2,J2,ccoef,dX2,dX1);
        for(int i=0;i<3;i++){ J[i][j]=dD1[i]; J[3+i][j]=dX2[i]; }
    }
    for(int j=0;j<3;j++){
        e[0]=e[1]=e[2]=0; e[j]=1;
        cross_rod_effect(h,e,R12,s1,s2,S1,S2,Rhalf1,Rfull1,eta1,D2_1,J1,ccoef,dX1,dX2);
        rod_delta_quantities(h,e,phi2,S2,Rhalf2,Rfull2,g2,eta2,D2_2,r21,n2,s2,acoef2,ccoef,alpha2,Ispin2,J2,dD1,dD2);
        for(int i=0;i<3;i++){ J[i][3+j]=dX1[i]; J[3+i][3+j]=dD1[i]; }
    }
}


static void solve_2x2_block(ld A00,ld A01,ld A10,ld A11, ld b0, ld b1, ld x[2]){
    ld det=A00*A11-A01*A10; if(fabsl(det)<1e-24L) det=(det>=0?1e-24L:-1e-24L); x[0]=(A11*b0-A01*b1)/det; x[1]=(-A10*b0+A00*b1)/det;
}
static void solve_blocklu4_to(const ld M[4][4], const ld rhs[4], ld x[4]){
    ld A00=M[0][0],A01=M[0][1],A10=M[1][0],A11=M[1][1];
    ld B00=M[0][2],B01=M[0][3],B10=M[1][2],B11=M[1][3];
    ld C00=M[2][0],C01=M[2][1],C10=M[3][0],C11=M[3][1];
    ld D00=M[2][2],D01=M[2][3],D10=M[3][2],D11=M[3][3];
    ld y[2]; solve_2x2_block(A00,A01,A10,A11,rhs[0],rhs[1],y);
    ld detA=A00*A11-A01*A10; if(fabsl(detA)<1e-24L) detA=(detA>=0?1e-24L:-1e-24L);
    ld invA00=A11/detA, invA01=-A01/detA, invA10=-A10/detA, invA11=A00/detA;
    ld T00=invA00*B00+invA01*B10, T01=invA00*B01+invA01*B11, T10=invA10*B00+invA11*B10, T11=invA10*B01+invA11*B11;
    ld S00=D00-(C00*T00+C01*T10), S01=D01-(C00*T01+C01*T11), S10=D10-(C10*T00+C11*T10), S11=D11-(C10*T01+C11*T11);
    ld rhsS0=rhs[2]-(C00*y[0]+C01*y[1]), rhsS1=rhs[3]-(C10*y[0]+C11*y[1]);
    ld z[2]; solve_2x2_block(S00,S01,S10,S11,rhsS0,rhsS1,z);
    ld rhsA0=rhs[0]-(B00*z[0]+B01*z[1]), rhsA1=rhs[1]-(B10*z[0]+B11*z[1]);
    ld x0[2]; solve_2x2_block(A00,A01,A10,A11,rhsA0,rhsA1,x0); x[0]=x0[0]; x[1]=x0[1]; x[2]=z[0]; x[3]=z[1];
}
static void active_info_from_params(const ld params[11], int active_idx[6], ld scales[6], int *nactive, int *n1, int *n2){
    ld l1=params[0], m1=params[2], m2=params[3], lc1=params[4], lc2=params[5], Iperp1=params[6], Iperp2=params[7], Ispin1=params[8], Ispin2=params[9];
    ld a1 = m1*lc1*lc1 + m2*l1*l1 + Iperp1;
    ld a2 = m2*lc2*lc2 + Iperp2;
    for(int i=0;i<6;i++){ active_idx[i]=-1; scales[i]=1.0L; }
    int nn1 = (Ispin1!=0.0L)?3:2;
    int nn2 = (Ispin2!=0.0L)?3:2;
    int idx=0;
    active_idx[idx]=0; idx++;
    active_idx[idx]=1; idx++;
    if(nn1==3){ active_idx[idx]=2; scales[idx]=sqrtl(a1/Ispin1); idx++; }
    active_idx[idx]=3; idx++;
    active_idx[idx]=4; idx++;
    if(nn2==3){ active_idx[idx]=5; scales[idx]=sqrtl(a2/Ispin2); idx++; }
    *nactive=idx; *n1=nn1; *n2=nn2;
}

static ld residual_norm_active_sq(const ld r[6], const int active_idx[6], int nactive){
    ld s=0.0L; for(int i=0;i<nactive;i++){ ld v=r[active_idx[i]]; s += v*v; } return s;
}

static void solve2(const ld A[2][2], const ld b[2], ld x[2]){ solve_2x2_block(A[0][0],A[0][1],A[1][0],A[1][1],b[0],b[1],x); }

static void solve_scaled_active_blockschur(const ld J[6][6], const ld r[6], const int active_idx[6], const ld scales[6], int nactive, int n1, int n2, ld dx[6]){
    ld A[3][3]={{0}},B[3][3]={{0}},C[3][3]={{0}},D[3][3]={{0}},S[3][3]={{0}},invAB[3][3]={{0}};
    ld rhs1[3]={0}, rhs2[3]={0}, invAr1[3]={0}, x1[3]={0}, x2[3]={0}, tmpv[3]={0};
    for(int i=0;i<n1;i++){
        int ii=active_idx[i]; ld si=scales[i]; rhs1[i] = -si*r[ii];
        for(int j=0;j<n1;j++){ int jj=active_idx[j]; A[i][j] = si*J[ii][jj]*scales[j]; }
        for(int j=0;j<n2;j++){ int jj=active_idx[n1+j]; B[i][j] = si*J[ii][jj]*scales[n1+j]; }
    }
    for(int i=0;i<n2;i++){
        int ii=active_idx[n1+i]; ld si=scales[n1+i]; rhs2[i] = -si*r[ii];
        for(int j=0;j<n1;j++){ int jj=active_idx[j]; C[i][j] = si*J[ii][jj]*scales[j]; }
        for(int j=0;j<n2;j++){ int jj=active_idx[n1+j]; D[i][j] = si*J[ii][jj]*scales[n1+j]; }
    }
    if(n1==2){ ld A2[2][2]={{A[0][0],A[0][1]},{A[1][0],A[1][1]}}; ld b2[2]={rhs1[0],rhs1[1]}, x2s[2]; solve2(A2,b2,x2s); invAr1[0]=x2s[0]; invAr1[1]=x2s[1];
        for(int j=0;j<n2;j++){ ld bb[2]={B[0][j],B[1][j]}; solve2(A2,bb,x2s); invAB[0][j]=x2s[0]; invAB[1][j]=x2s[1]; }
    } else { solve3(A,rhs1,invAr1); for(int j=0;j<n2;j++){ tmpv[0]=B[0][j]; tmpv[1]=B[1][j]; tmpv[2]=B[2][j]; solve3(A,tmpv,x1); invAB[0][j]=x1[0]; invAB[1][j]=x1[1]; invAB[2][j]=x1[2]; } }
    for(int i=0;i<n2;i++){
        ld s=0.0L; for(int k=0;k<n1;k++) s += C[i][k]*invAr1[k]; rhs2[i] -= s;
        for(int j=0;j<n2;j++){ s=0.0L; for(int k=0;k<n1;k++) s += C[i][k]*invAB[k][j]; S[i][j] = D[i][j] - s; }
    }
    if(n2==2){ ld S2[2][2]={{S[0][0],S[0][1]},{S[1][0],S[1][1]}}, b2[2]={rhs2[0],rhs2[1]}, xs[2]; solve2(S2,b2,xs); x2[0]=xs[0]; x2[1]=xs[1]; }
    else solve3(S,rhs2,x2);
    for(int i=0;i<n1;i++){ ld s=0.0L; for(int j=0;j<n2;j++) s += invAB[i][j]*x2[j]; x1[i] = invAr1[i] - s; }
    for(int i=0;i<6;i++) dx[i]=0.0L;
    for(int i=0;i<n1;i++) dx[active_idx[i]] = scales[i]*x1[i];
    for(int i=0;i<n2;i++) dx[active_idx[n1+i]] = scales[n1+i]*x2[i];
}

static void relative_z_from_pair(const ld y_prev[8], const ld y_cur[8], ld z_out[6]){
    ld qa[4], qc[4], qrel[4], rv[3];
    hemi_copy4(y_prev, y_cur, qa); q_conj_inplace(y_prev, qc); q_mul_inplace(qc, qa, qrel); q_log_rotvec_to(qrel, rv); z_out[0]=rv[0]; z_out[1]=rv[1]; z_out[2]=rv[2];
    hemi_copy4(y_prev+4, y_cur+4, qa); q_conj_inplace(y_prev+4, qc); q_mul_inplace(qc, qa, qrel); q_log_rotvec_to(qrel, rv); z_out[3]=rv[0]; z_out[4]=rv[1]; z_out[5]=rv[2];
}

static void p_from_pair(const ld y_prev[8], const ld y_cur[8], ld h, const ld params[11], ld pout[6]){
    ld phi1[3],phi2[3],om1[3],om2[3],s1[3],s2[3],n1[3],n2[3],r12[3],r21[3],mu1[3],mu2[3],g1[3],g2[3],eta1[3],eta2[3],D1_1[3],D1_2[3],D2_1[3],D2_2[3];
    ld qm1[4],qm2[4], R1[3][3],R2[3][3],R12[3][3],R21[3][3],J1[3][3],J2[3][3],S1[3][3],S2[3][3],Rfull1[3][3],Rfull2[3][3],Rhalf1[3][3],Rhalf2[3][3], D1tmp[6], D2tmp[6];
    endpoint_derivatives_full_cached(y_prev,y_cur,h,params,phi1,phi2,qm1,qm2,om1,om2,R1,R2,R12,R21,s1,s2,n1,n2,r12,r21,mu1,mu2,g1,g2,J1,J2,S1,S2,Rfull1,Rfull2,Rhalf1,Rhalf2,eta1,eta2,D1_1,D1_2,D2_1,D2_2,D1tmp,D2tmp);
    copy6(D2tmp,pout);
}

static void guess_z_from_half_momentum(const ld y_cur[8], const ld p_half[6], ld h, const ld params[11], ld z_guess[6]){
    ld l1=params[0], m1=params[2], m2=params[3], lc1=params[4], lc2=params[5], Iperp1=params[6], Iperp2=params[7], Ispin1=params[8], Ispin2=params[9];
    ld R1[3][3],R2[3][3],R12[3][3],R21[3][3]; q_to_R_inplace(y_cur,R1); q_to_R_inplace(y_cur+4,R2);
    for(int i=0;i<3;i++) for(int j=0;j<3;j++){ R12[i][j]=R1[0][i]*R2[0][j]+R1[1][i]*R2[1][j]+R1[2][i]*R2[2][j]; R21[j][i]=R12[i][j]; }
    ld a1=m1*lc1*lc1 + m2*l1*l1 + Iperp1, a2=m2*lc2*lc2 + Iperp2, ccoef=m2*l1*lc2;
    ld A4[4][4]={{0}}; ld b4[4]={p_half[0],p_half[1],p_half[3],p_half[4]};
    A4[0][0]=a1; A4[0][2]=ccoef*R12[1][1]; A4[0][3]=-ccoef*R12[1][0];
    A4[1][1]=a1; A4[1][2]=-ccoef*R12[0][1]; A4[1][3]=ccoef*R12[0][0];
    A4[2][0]=ccoef*R21[1][1]; A4[2][1]=-ccoef*R21[1][0]; A4[2][2]=a2;
    A4[3][0]=-ccoef*R21[0][1]; A4[3][1]=ccoef*R21[0][0]; A4[3][3]=a2;
    ld u[4]; solve_blocklu4_to(A4,b4,u);
    z_guess[0]=h*u[0]; z_guess[1]=h*u[1]; z_guess[2]=(Ispin1!=0.0L? h*(p_half[2]/Ispin1):0.0L);
    z_guess[3]=h*u[2]; z_guess[4]=h*u[3]; z_guess[5]=(Ispin2!=0.0L? h*(p_half[5]/Ispin2):0.0L);
}

static void residual_hamiltonian_left(const ld z6[6], const ld y_cur[8], ld h, const ld params[11], const ld p_cur[6],
                                      ld y_next[8], ld r[6],
                                      ld phi1[3],ld phi2[3],ld qm1[4],ld qm2[4],ld om1[3],ld om2[3],ld R1[3][3],ld R2[3][3],ld R12[3][3],ld R21[3][3],
                                      ld s1[3],ld s2[3],ld n1[3],ld n2[3],ld r12[3],ld r21[3],ld mu1[3],ld mu2[3],ld g1[3],ld g2[3],ld J1[3][3],ld J2[3][3],ld S1[3][3],ld S2[3][3],ld Rfull1[3][3],ld Rfull2[3][3],ld Rhalf1[3][3],ld Rhalf2[3][3],ld eta1[3],ld eta2[3],ld D1_1[3],ld D1_2[3],ld D2_1[3],ld D2_2[3],ld D1tmp[6],ld D2tmp[6]){
    retract_pair_full_to(y_cur,z6,y_next);
    endpoint_derivatives_full_cached(y_cur,y_next,h,params,phi1,phi2,qm1,qm2,om1,om2,R1,R2,R12,R21,s1,s2,n1,n2,r12,r21,mu1,mu2,g1,g2,J1,J2,S1,S2,Rfull1,Rfull2,Rhalf1,Rhalf2,eta1,eta2,D1_1,D1_2,D2_1,D2_2,D1tmp,D2tmp);
    for(int i=0;i<6;i++) r[i] = p_cur[i] + D1tmp[i];
}

static void residual_hamiltonian_right(const ld z6[6], const ld y_cur[8], ld h, const ld params[11], const ld p_half[6],
                                      ld y_next[8], ld r[6],
                                      ld phi1[3],ld phi2[3],ld qm1[4],ld qm2[4],ld om1[3],ld om2[3],ld R1[3][3],ld R2[3][3],ld R12[3][3],ld R21[3][3],
                                      ld s1[3],ld s2[3],ld n1[3],ld n2[3],ld r12[3],ld r21[3],ld mu1[3],ld mu2[3],ld g1[3],ld g2[3],ld J1[3][3],ld J2[3][3],ld S1[3][3],ld S2[3][3],ld Rfull1[3][3],ld Rfull2[3][3],ld Rhalf1[3][3],ld Rhalf2[3][3],ld eta1[3],ld eta2[3],ld D1_1[3],ld D1_2[3],ld D2_1[3],ld D2_2[3],ld D1tmp[6],ld D2tmp[6]){
    retract_pair_full_to(y_cur,z6,y_next);
    endpoint_derivatives_full_cached(y_cur,y_next,h,params,phi1,phi2,qm1,qm2,om1,om2,R1,R2,R12,R21,s1,s2,n1,n2,r12,r21,mu1,mu2,g1,g2,J1,J2,S1,S2,Rfull1,Rfull2,Rhalf1,Rhalf2,eta1,eta2,D1_1,D1_2,D2_1,D2_2,D1tmp,D2tmp);
    for(int i=0;i<6;i++) r[i] = D2tmp[i] - p_half[i];
}

static void jacobian_right_full_cached(ld h,
                         const ld phi1[3], const ld phi2[3], const ld s1[3], const ld s2[3], const ld n1[3], const ld n2[3], const ld r12[3], const ld r21[3],
                         const ld R12[3][3], const ld R21[3][3],
                         const ld mu1[3], const ld mu2[3], const ld g1[3], const ld g2[3],
                         const ld J1[3][3], const ld J2[3][3], const ld S1[3][3], const ld S2[3][3], const ld Rfull1[3][3], const ld Rfull2[3][3], const ld Rhalf1[3][3], const ld Rhalf2[3][3],
                         const ld eta1[3], const ld eta2[3], const ld D2_1[3], const ld D2_2[3],
                         const ld params[11], ld J[6][6]){
    ld Ispin1=params[8], Ispin2=params[9];
    ld e[3], dD1[3], dD2[3], dX1[3], dX2[3];
    ld acoef1=params[2]*params[4]*params[4] + params[3]*params[0]*params[0] + params[6];
    ld acoef2=params[3]*params[5]*params[5] + params[7];
    ld ccoef=params[3]*params[0]*params[5];
    ld alpha1=params[10]*(params[2]*params[4] + params[3]*params[0]);
    ld alpha2=params[10]*(params[3]*params[5]);
    for(int i=0;i<6;i++) for(int j=0;j<6;j++) J[i][j]=0.0L;
    for(int j=0;j<3;j++){
        e[0]=e[1]=e[2]=0; e[j]=1;
        rod_delta_quantities(h,e,phi1,S1,Rhalf1,Rfull1,g1,eta1,D2_1,r12,n1,s1,acoef1,ccoef,alpha1,Ispin1,J1,dD1,dD2);
        cross_rod_effect(h,e,R21,s2,s1,S2,S1,Rhalf2,Rfull2,eta2,D2_2,J2,ccoef,dX2,dX1);
        for(int i=0;i<3;i++){ J[i][j]=dD2[i]; J[3+i][j]=dX1[i]; }
    }
    for(int j=0;j<3;j++){
        e[0]=e[1]=e[2]=0; e[j]=1;
        cross_rod_effect(h,e,R12,s1,s2,S1,S2,Rhalf1,Rfull1,eta1,D2_1,J1,ccoef,dX1,dX2);
        rod_delta_quantities(h,e,phi2,S2,Rhalf2,Rfull2,g2,eta2,D2_2,r21,n2,s2,acoef2,ccoef,alpha2,Ispin2,J2,dD1,dD2);
        for(int i=0;i<3;i++){ J[i][3+j]=dX2[i]; J[3+i][3+j]=dD2[i]; }
    }
}

static int solve_next_legendre_scaled(const ld y_prev[8], const ld y_cur[8], ld h, const ld params[11], const ld z_prevprev[6], const ld z_prev[6], const mcld_solver_opts *opts,
                                    ld y_next[8], ld z_out[6], const ld D2_prev[6], ld *best_residual, int *niters){
    int active_idx[6], nactive,n1a,n2a; ld scales[6]; active_info_from_params(params,active_idx,scales,&nactive,&n1a,&n2a);
    ld phi1[3],phi2[3],om1[3],om2[3],s1[3],s2[3],n1[3],n2[3],r12[3],r21[3],mu1[3],mu2[3],g1[3],g2[3],eta1[3],eta2[3],D1_1[3],D1_2[3],D2_1[3],D2_2[3];
    ld qm1[4],qm2[4], R1[3][3],R2[3][3],R12[3][3],R21[3][3],J1[3][3],J2[3][3],S1[3][3],S2[3][3],Rfull1[3][3],Rfull2[3][3],Rhalf1[3][3],Rhalf2[3][3];
    ld D1tmp[6], D2tmp[6], r[6], J[6][6], dx[6], best_y[8], best_z[6];
    predictor_secant_to(z_prevprev,z_prev,opts->beta,z_out); retract_pair_full_to(y_cur,z_out,y_next); copy8(y_next,best_y); copy6(z_out,best_z);
    ld best_rn2=1e300L; int ok=0, nit=0;
    for(int it=0; it<opts->max_iter; ++it){ nit=it+1;
        residual_full_cached(z_out,y_prev,y_cur,h,params,D2_prev,y_next,phi1,phi2,qm1,qm2,om1,om2,R1,R2,R12,R21,s1,s2,n1,n2,r12,r21,mu1,mu2,g1,g2,J1,J2,S1,S2,Rfull1,Rfull2,Rhalf1,Rhalf2,eta1,eta2,D1_1,D1_2,D2_1,D2_2,D1tmp,D2tmp,r);
        jacobian_full_cached(h,phi1,phi2,s1,s2,n1,n2,r12,r21,R12,R21,mu1,mu2,g1,g2,J1,J2,S1,S2,Rfull1,Rfull2,Rhalf1,Rhalf2,eta1,eta2,D2_1,D2_2,params,J);
        ld rn2 = residual_norm_active_sq(r, active_idx, nactive);
        if(rn2<best_rn2){ best_rn2=rn2; copy8(y_next,best_y); copy6(z_out,best_z);} if(rn2<=opts->tol*opts->tol){ ok=1; break; }
        solve_scaled_active_blockschur(J,r,active_idx,scales,nactive,n1a,n2a,dx);
        for(int i=0;i<6;i++) z_out[i]+=dx[i]; if(n1a==2) z_out[2]=0; if(n2a==2) z_out[5]=0; retract_pair_full_to(y_cur,z_out,y_next);
    }
    if(!ok){ copy8(best_y,y_next); copy6(best_z,z_out); if(best_rn2<=opts->tol*opts->tol) ok=1; }
    if(best_residual) *best_residual=sqrtl(best_rn2); if(niters) *niters=nit; return ok;
}

static int solve_hamiltonian_left_scaled(const ld y_cur[8], const ld p_cur[6], ld h, const ld params[11], const ld z_guess[6], const mcld_solver_opts *opts,
                                    ld y_next[8], ld z_out[6], ld p_next[6], ld *best_residual, int *niters){
    int active_idx[6], nactive,n1a,n2a; ld scales[6]; active_info_from_params(params,active_idx,scales,&nactive,&n1a,&n2a);
    ld phi1[3],phi2[3],om1[3],om2[3],s1[3],s2[3],n1[3],n2[3],r12[3],r21[3],mu1[3],mu2[3],g1[3],g2[3],eta1[3],eta2[3],D1_1[3],D1_2[3],D2_1[3],D2_2[3];
    ld qm1[4],qm2[4], R1[3][3],R2[3][3],R12[3][3],R21[3][3],J1[3][3],J2[3][3],S1[3][3],S2[3][3],Rfull1[3][3],Rfull2[3][3],Rhalf1[3][3],Rhalf2[3][3];
    ld D1tmp[6], D2tmp[6], r[6], J[6][6], dx[6], best_y[8], best_z[6];
    copy6(z_guess,z_out); retract_pair_full_to(y_cur,z_out,y_next); copy8(y_next,best_y); copy6(z_out,best_z);
    ld best_rn2=1e300L; int ok=0, nit=0;
    for(int it=0; it<opts->max_iter; ++it){ nit=it+1;
        residual_hamiltonian_left(z_out,y_cur,h,params,p_cur,y_next,r,phi1,phi2,qm1,qm2,om1,om2,R1,R2,R12,R21,s1,s2,n1,n2,r12,r21,mu1,mu2,g1,g2,J1,J2,S1,S2,Rfull1,Rfull2,Rhalf1,Rhalf2,eta1,eta2,D1_1,D1_2,D2_1,D2_2,D1tmp,D2tmp);
        jacobian_full_cached(h,phi1,phi2,s1,s2,n1,n2,r12,r21,R12,R21,mu1,mu2,g1,g2,J1,J2,S1,S2,Rfull1,Rfull2,Rhalf1,Rhalf2,eta1,eta2,D2_1,D2_2,params,J);
        ld rn2 = residual_norm_active_sq(r, active_idx, nactive);
        if(rn2<best_rn2){ best_rn2=rn2; copy8(y_next,best_y); copy6(z_out,best_z);} if(rn2<=opts->tol*opts->tol){ ok=1; break; }
        solve_scaled_active_blockschur(J,r,active_idx,scales,nactive,n1a,n2a,dx);
        for(int i=0;i<6;i++) z_out[i]+=dx[i]; if(n1a==2) z_out[2]=0; if(n2a==2) z_out[5]=0; retract_pair_full_to(y_cur,z_out,y_next);
    }
    if(!ok){ copy8(best_y,y_next); copy6(best_z,z_out); if(best_rn2<=opts->tol*opts->tol) ok=1; }
    p_from_pair(y_cur,y_next,h,params,p_next);
    if(best_residual) *best_residual=sqrtl(best_rn2); if(niters) *niters=nit; return ok;
}

static int solve_hamiltonian_right_scaled(const ld y_cur[8], const ld p_half[6], ld h, const ld params[11], const ld z_guess[6], const mcld_solver_opts *opts,
                                    ld y_next[8], ld z_out[6], ld *best_residual, int *niters){
    int active_idx[6], nactive,n1a,n2a; ld scales[6]; active_info_from_params(params,active_idx,scales,&nactive,&n1a,&n2a);
    ld phi1[3],phi2[3],om1[3],om2[3],s1[3],s2[3],n1[3],n2[3],r12[3],r21[3],mu1[3],mu2[3],g1[3],g2[3],eta1[3],eta2[3],D1_1[3],D1_2[3],D2_1[3],D2_2[3];
    ld qm1[4],qm2[4], R1[3][3],R2[3][3],R12[3][3],R21[3][3],J1[3][3],J2[3][3],S1[3][3],S2[3][3],Rfull1[3][3],Rfull2[3][3],Rhalf1[3][3],Rhalf2[3][3];
    ld D1tmp[6], D2tmp[6], r[6], J[6][6], dx[6], best_y[8], best_z[6];
    copy6(z_guess,z_out); retract_pair_full_to(y_cur,z_out,y_next); copy8(y_next,best_y); copy6(z_out,best_z);
    ld best_rn2=1e300L; int ok=0, nit=0;
    for(int it=0; it<opts->max_iter; ++it){ nit=it+1;
        residual_hamiltonian_right(z_out,y_cur,h,params,p_half,y_next,r,phi1,phi2,qm1,qm2,om1,om2,R1,R2,R12,R21,s1,s2,n1,n2,r12,r21,mu1,mu2,g1,g2,J1,J2,S1,S2,Rfull1,Rfull2,Rhalf1,Rhalf2,eta1,eta2,D1_1,D1_2,D2_1,D2_2,D1tmp,D2tmp);
        jacobian_right_full_cached(h,phi1,phi2,s1,s2,n1,n2,r12,r21,R12,R21,mu1,mu2,g1,g2,J1,J2,S1,S2,Rfull1,Rfull2,Rhalf1,Rhalf2,eta1,eta2,D2_1,D2_2,params,J);
        ld rn2 = residual_norm_active_sq(r, active_idx, nactive);
        if(rn2<best_rn2){ best_rn2=rn2; copy8(y_next,best_y); copy6(z_out,best_z);} if(rn2<=opts->tol*opts->tol){ ok=1; break; }
        solve_scaled_active_blockschur(J,r,active_idx,scales,nactive,n1a,n2a,dx);
        for(int i=0;i<6;i++) z_out[i]+=dx[i]; if(n1a==2) z_out[2]=0; if(n2a==2) z_out[5]=0; retract_pair_full_to(y_cur,z_out,y_next);
    }
    if(!ok){ copy8(best_y,y_next); copy6(best_z,z_out); if(best_rn2<=opts->tol*opts->tol) ok=1; }
    if(best_residual) *best_residual=sqrtl(best_rn2); if(niters) *niters=nit; return ok;
}

int mcld_legendre_step_from_pair(const ld y_prev[8], const ld y_cur[8], ld h, const ld params[11], const mcld_solver_opts *opts,
    ld y_next[8], ld z_cur[6], ld p_plus[6], ld *best_residual, int *niters){
    mcld_solver_opts local; if(!opts){ mcld_default_solver_opts(&local); opts=&local; }
    ld z_pair[6], p_prev[6]; relative_z_from_pair(y_prev,y_cur,z_pair); p_from_pair(y_prev,y_cur,h,params,p_prev);
    int ok = solve_next_legendre_scaled(y_prev,y_cur,h,params,z_pair,z_pair,opts,y_next,z_cur,p_prev,best_residual,niters);
    p_from_pair(y_cur,y_next,h,params,p_plus); return ok;
}

int mcld_legendre_integrate_trajectory_from_pair(const ld y_prev0[8], const ld y0[8], ld t_end, int n_steps, const ld params[11], const mcld_solver_opts *opts,
    ld *Y_out, ld *Z_out, ld *P_out, ld *worst_residual, int *worst_iters, int *total_iters){
    if(n_steps<1||!Y_out||!Z_out||!P_out) return 0; mcld_solver_opts local; if(!opts){ mcld_default_solver_opts(&local); opts=&local; }
    ld h=t_end/(ld)n_steps; for(int i=0;i<8;i++){ Y_out[i]=y_prev0[i]; Y_out[8+i]=y0[i]; }
    relative_z_from_pair(y_prev0,y0,Z_out); p_from_pair(y_prev0,y0,h,params,P_out);
    int ok_all=1, worst_it=0, tot=0; ld worst=0.0L;
    for(int k=0;k<n_steps;k++){
        ld res; int nit; int ok = mcld_legendre_step_from_pair(&Y_out[k*8], &Y_out[(k+1)*8], h, params, opts, &Y_out[(k+2)*8], &Z_out[(k+1)*6], &P_out[(k+1)*6], &res, &nit);
        if(!ok) ok_all=0; if(res>worst) worst=res; if(nit>worst_it) worst_it=nit; tot += nit;
    }
    if(worst_residual) *worst_residual=worst; if(worst_iters) *worst_iters=worst_it; if(total_iters) *total_iters=tot; return ok_all;
}

int mcld_legendre_integrate_final_from_pair(const ld y_prev0[8], const ld y0[8], ld t_end, int n_steps, const ld params[11], const mcld_solver_opts *opts,
    ld y_prev_final[8], ld y_final[8], ld z_last[6], ld p_last[6], ld *worst_residual, int *worst_iters, int *total_iters){
    if(n_steps<1) return 0; ld *Y=(ld*)calloc((size_t)(n_steps+2)*8,sizeof(ld)); ld *Z=(ld*)calloc((size_t)(n_steps+1)*6,sizeof(ld)); ld *P=(ld*)calloc((size_t)(n_steps+1)*6,sizeof(ld));
    int ok = mcld_legendre_integrate_trajectory_from_pair(y_prev0,y0,t_end,n_steps,params,opts,Y,Z,P,worst_residual,worst_iters,total_iters);
    copy8(&Y[n_steps*8], y_prev_final); copy8(&Y[(n_steps+1)*8], y_final); copy6(&Z[n_steps*6], z_last); copy6(&P[n_steps*6], p_last); free(Y); free(Z); free(P); return ok;
}

int mcld_hamilton_step_from_left(const ld y_cur[8], const ld p_mhalf[6], ld h, const ld params[11], const mcld_solver_opts *opts,
    ld y_next[8], ld p_phalf[6], ld z_cur[6], ld *best_residual, int *niters){
    mcld_solver_opts local; if(!opts){ mcld_default_solver_opts(&local); opts=&local; }
    ld z_guess[6]; guess_z_from_half_momentum(y_cur,p_mhalf,h,params,z_guess);
    return solve_hamiltonian_left_scaled(y_cur,p_mhalf,h,params,z_guess,opts,y_next,z_cur,p_phalf,best_residual,niters);
}

int mcld_hamilton_integrate_trajectory_from_left(const ld y0[8], const ld p_mhalf0[6], ld t_end, int n_steps, const ld params[11], const mcld_solver_opts *opts,
    ld *Y_out, ld *P_out, ld *Z_out, ld *worst_residual, int *worst_iters, int *total_iters){
    if(n_steps<1||!Y_out||!P_out||!Z_out) return 0; mcld_solver_opts local; if(!opts){ mcld_default_solver_opts(&local); opts=&local; }
    ld h=t_end/(ld)n_steps; copy8(y0,Y_out); copy6(p_mhalf0,P_out);
    ld y_next[8], p_next[6], z_prev[6], z_prevprev[6], z_guess[6], res; int nit;
    int ok_all = mcld_hamilton_step_from_left(y0,p_mhalf0,h,params,opts,y_next,p_next,Z_out,&res,&nit);
    copy8(y_next,&Y_out[8]); copy6(p_next,&P_out[6]); copy6(Z_out,z_prev); copy6(Z_out,z_prevprev);
    int worst_it=nit, tot=nit; ld worst=res;
    for(int k=1;k<n_steps;k++){
        predictor_secant_to(z_prevprev,z_prev,opts->beta,z_guess);
        int ok = solve_hamiltonian_left_scaled(&Y_out[k*8], &P_out[k*6], h, params, z_guess, opts, y_next, &Z_out[k*6], p_next, &res, &nit);
        if(!ok) ok_all=0; copy8(y_next,&Y_out[(k+1)*8]); copy6(p_next,&P_out[(k+1)*6]); copy6(z_prev,z_prevprev); copy6(&Z_out[k*6],z_prev); if(res>worst) worst=res; if(nit>worst_it) worst_it=nit; tot+=nit;
    }
    if(worst_residual) *worst_residual=worst; if(worst_iters) *worst_iters=worst_it; if(total_iters) *total_iters=tot; return ok_all;
}

int mcld_hamilton_integrate_final_from_left(const ld y0[8], const ld p_mhalf0[6], ld t_end, int n_steps, const ld params[11], const mcld_solver_opts *opts,
    ld y_final[8], ld p_final[6], ld z_last[6], ld *worst_residual, int *worst_iters, int *total_iters){
    if(n_steps<1) return 0; ld *Y=(ld*)calloc((size_t)(n_steps+1)*8,sizeof(ld)); ld *P=(ld*)calloc((size_t)(n_steps+1)*6,sizeof(ld)); ld *Z=(ld*)calloc((size_t)n_steps*6,sizeof(ld));
    int ok = mcld_hamilton_integrate_trajectory_from_left(y0,p_mhalf0,t_end,n_steps,params,opts,Y,P,Z,worst_residual,worst_iters,total_iters);
    copy8(&Y[n_steps*8], y_final); copy6(&P[n_steps*6], p_final); copy6(&Z[(n_steps-1)*6], z_last); free(Y); free(P); free(Z); return ok;
}

int mcld_hamilton_step_from_right(const ld y_cur[8], const ld p_phalf[6], ld h, const ld params[11], const mcld_solver_opts *opts,
    ld y_next[8], ld p_same[6], ld z_cur[6], ld *best_residual, int *niters){
    mcld_solver_opts local; if(!opts){ mcld_default_solver_opts(&local); opts=&local; }
    ld z_guess[6]; guess_z_from_half_momentum(y_cur,p_phalf,h,params,z_guess);
    int ok = solve_hamiltonian_right_scaled(y_cur,p_phalf,h,params,z_guess,opts,y_next,z_cur,best_residual,niters); copy6(p_phalf,p_same); return ok;
}

int mcld_hamilton_integrate_trajectory_from_right(const ld y0[8], const ld p_phalf0[6], ld t_end, int n_steps, const ld params[11], const mcld_solver_opts *opts,
    ld *Y_out, ld *P_out, ld *Z_out, ld *worst_residual, int *worst_iters, int *total_iters){
    if(n_steps<1||!Y_out||!P_out||!Z_out) return 0; mcld_solver_opts local; if(!opts){ mcld_default_solver_opts(&local); opts=&local; }
    ld h=t_end/(ld)n_steps; copy8(y0,Y_out); copy6(p_phalf0,P_out);
    ld y_next[8], z_prev[6], z_prevprev[6], z_guess[6], p_next[6], res; int nit;
    int ok_all = mcld_hamilton_step_from_right(y0,p_phalf0,h,params,opts,y_next,p_next,Z_out,&res,&nit);
    copy8(y_next,&Y_out[8]);
    copy6(p_phalf0,&P_out[6]);
    copy6(Z_out,z_prev); copy6(Z_out,z_prevprev);
    int worst_it=nit, tot=nit; ld worst=res;
    for(int k=1;k<n_steps;k++){
        predictor_secant_to(z_prevprev,z_prev,opts->beta,z_guess);
        int ok = solve_hamiltonian_left_scaled(&Y_out[k*8], &P_out[k*6], h, params, z_guess, opts, y_next, &Z_out[k*6], p_next, &res, &nit);
        if(!ok) ok_all=0;
        copy8(y_next,&Y_out[(k+1)*8]);
        copy6(p_next,&P_out[(k+1)*6]);
        copy6(z_prev,z_prevprev);
        copy6(&Z_out[k*6],z_prev);
        if(res>worst) worst=res;
        if(nit>worst_it) worst_it=nit;
        tot+=nit;
    }
    if(worst_residual) *worst_residual=worst; if(worst_iters) *worst_iters=worst_it; if(total_iters) *total_iters=tot; return ok_all;
}

int mcld_hamilton_integrate_final_from_right(const ld y0[8], const ld p_phalf0[6], ld t_end, int n_steps, const ld params[11], const mcld_solver_opts *opts,
    ld y_final[8], ld p_last[6], ld z_last[6], ld *worst_residual, int *worst_iters, int *total_iters){
    if(n_steps<1) return 0; ld *Y=(ld*)calloc((size_t)(n_steps+1)*8,sizeof(ld)); ld *P=(ld*)calloc((size_t)n_steps*6,sizeof(ld)); ld *Z=(ld*)calloc((size_t)n_steps*6,sizeof(ld));
    int ok = mcld_hamilton_integrate_trajectory_from_right(y0,p_phalf0,t_end,n_steps,params,opts,Y,P,Z,worst_residual,worst_iters,total_iters);
    copy8(&Y[n_steps*8], y_final); copy6(&P[(n_steps-1)*6], p_last); copy6(&Z[(n_steps-1)*6], z_last); free(Y); free(P); free(Z); return ok;
}
