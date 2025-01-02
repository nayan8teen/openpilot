#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_176698825170516537) {
   out_176698825170516537[0] = delta_x[0] + nom_x[0];
   out_176698825170516537[1] = delta_x[1] + nom_x[1];
   out_176698825170516537[2] = delta_x[2] + nom_x[2];
   out_176698825170516537[3] = delta_x[3] + nom_x[3];
   out_176698825170516537[4] = delta_x[4] + nom_x[4];
   out_176698825170516537[5] = delta_x[5] + nom_x[5];
   out_176698825170516537[6] = delta_x[6] + nom_x[6];
   out_176698825170516537[7] = delta_x[7] + nom_x[7];
   out_176698825170516537[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7598527359371644641) {
   out_7598527359371644641[0] = -nom_x[0] + true_x[0];
   out_7598527359371644641[1] = -nom_x[1] + true_x[1];
   out_7598527359371644641[2] = -nom_x[2] + true_x[2];
   out_7598527359371644641[3] = -nom_x[3] + true_x[3];
   out_7598527359371644641[4] = -nom_x[4] + true_x[4];
   out_7598527359371644641[5] = -nom_x[5] + true_x[5];
   out_7598527359371644641[6] = -nom_x[6] + true_x[6];
   out_7598527359371644641[7] = -nom_x[7] + true_x[7];
   out_7598527359371644641[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3793825779596937896) {
   out_3793825779596937896[0] = 1.0;
   out_3793825779596937896[1] = 0;
   out_3793825779596937896[2] = 0;
   out_3793825779596937896[3] = 0;
   out_3793825779596937896[4] = 0;
   out_3793825779596937896[5] = 0;
   out_3793825779596937896[6] = 0;
   out_3793825779596937896[7] = 0;
   out_3793825779596937896[8] = 0;
   out_3793825779596937896[9] = 0;
   out_3793825779596937896[10] = 1.0;
   out_3793825779596937896[11] = 0;
   out_3793825779596937896[12] = 0;
   out_3793825779596937896[13] = 0;
   out_3793825779596937896[14] = 0;
   out_3793825779596937896[15] = 0;
   out_3793825779596937896[16] = 0;
   out_3793825779596937896[17] = 0;
   out_3793825779596937896[18] = 0;
   out_3793825779596937896[19] = 0;
   out_3793825779596937896[20] = 1.0;
   out_3793825779596937896[21] = 0;
   out_3793825779596937896[22] = 0;
   out_3793825779596937896[23] = 0;
   out_3793825779596937896[24] = 0;
   out_3793825779596937896[25] = 0;
   out_3793825779596937896[26] = 0;
   out_3793825779596937896[27] = 0;
   out_3793825779596937896[28] = 0;
   out_3793825779596937896[29] = 0;
   out_3793825779596937896[30] = 1.0;
   out_3793825779596937896[31] = 0;
   out_3793825779596937896[32] = 0;
   out_3793825779596937896[33] = 0;
   out_3793825779596937896[34] = 0;
   out_3793825779596937896[35] = 0;
   out_3793825779596937896[36] = 0;
   out_3793825779596937896[37] = 0;
   out_3793825779596937896[38] = 0;
   out_3793825779596937896[39] = 0;
   out_3793825779596937896[40] = 1.0;
   out_3793825779596937896[41] = 0;
   out_3793825779596937896[42] = 0;
   out_3793825779596937896[43] = 0;
   out_3793825779596937896[44] = 0;
   out_3793825779596937896[45] = 0;
   out_3793825779596937896[46] = 0;
   out_3793825779596937896[47] = 0;
   out_3793825779596937896[48] = 0;
   out_3793825779596937896[49] = 0;
   out_3793825779596937896[50] = 1.0;
   out_3793825779596937896[51] = 0;
   out_3793825779596937896[52] = 0;
   out_3793825779596937896[53] = 0;
   out_3793825779596937896[54] = 0;
   out_3793825779596937896[55] = 0;
   out_3793825779596937896[56] = 0;
   out_3793825779596937896[57] = 0;
   out_3793825779596937896[58] = 0;
   out_3793825779596937896[59] = 0;
   out_3793825779596937896[60] = 1.0;
   out_3793825779596937896[61] = 0;
   out_3793825779596937896[62] = 0;
   out_3793825779596937896[63] = 0;
   out_3793825779596937896[64] = 0;
   out_3793825779596937896[65] = 0;
   out_3793825779596937896[66] = 0;
   out_3793825779596937896[67] = 0;
   out_3793825779596937896[68] = 0;
   out_3793825779596937896[69] = 0;
   out_3793825779596937896[70] = 1.0;
   out_3793825779596937896[71] = 0;
   out_3793825779596937896[72] = 0;
   out_3793825779596937896[73] = 0;
   out_3793825779596937896[74] = 0;
   out_3793825779596937896[75] = 0;
   out_3793825779596937896[76] = 0;
   out_3793825779596937896[77] = 0;
   out_3793825779596937896[78] = 0;
   out_3793825779596937896[79] = 0;
   out_3793825779596937896[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2702386721019875926) {
   out_2702386721019875926[0] = state[0];
   out_2702386721019875926[1] = state[1];
   out_2702386721019875926[2] = state[2];
   out_2702386721019875926[3] = state[3];
   out_2702386721019875926[4] = state[4];
   out_2702386721019875926[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2702386721019875926[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2702386721019875926[7] = state[7];
   out_2702386721019875926[8] = state[8];
}
void F_fun(double *state, double dt, double *out_4271806009047526976) {
   out_4271806009047526976[0] = 1;
   out_4271806009047526976[1] = 0;
   out_4271806009047526976[2] = 0;
   out_4271806009047526976[3] = 0;
   out_4271806009047526976[4] = 0;
   out_4271806009047526976[5] = 0;
   out_4271806009047526976[6] = 0;
   out_4271806009047526976[7] = 0;
   out_4271806009047526976[8] = 0;
   out_4271806009047526976[9] = 0;
   out_4271806009047526976[10] = 1;
   out_4271806009047526976[11] = 0;
   out_4271806009047526976[12] = 0;
   out_4271806009047526976[13] = 0;
   out_4271806009047526976[14] = 0;
   out_4271806009047526976[15] = 0;
   out_4271806009047526976[16] = 0;
   out_4271806009047526976[17] = 0;
   out_4271806009047526976[18] = 0;
   out_4271806009047526976[19] = 0;
   out_4271806009047526976[20] = 1;
   out_4271806009047526976[21] = 0;
   out_4271806009047526976[22] = 0;
   out_4271806009047526976[23] = 0;
   out_4271806009047526976[24] = 0;
   out_4271806009047526976[25] = 0;
   out_4271806009047526976[26] = 0;
   out_4271806009047526976[27] = 0;
   out_4271806009047526976[28] = 0;
   out_4271806009047526976[29] = 0;
   out_4271806009047526976[30] = 1;
   out_4271806009047526976[31] = 0;
   out_4271806009047526976[32] = 0;
   out_4271806009047526976[33] = 0;
   out_4271806009047526976[34] = 0;
   out_4271806009047526976[35] = 0;
   out_4271806009047526976[36] = 0;
   out_4271806009047526976[37] = 0;
   out_4271806009047526976[38] = 0;
   out_4271806009047526976[39] = 0;
   out_4271806009047526976[40] = 1;
   out_4271806009047526976[41] = 0;
   out_4271806009047526976[42] = 0;
   out_4271806009047526976[43] = 0;
   out_4271806009047526976[44] = 0;
   out_4271806009047526976[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4271806009047526976[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4271806009047526976[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4271806009047526976[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4271806009047526976[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4271806009047526976[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4271806009047526976[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4271806009047526976[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4271806009047526976[53] = -9.8000000000000007*dt;
   out_4271806009047526976[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4271806009047526976[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4271806009047526976[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4271806009047526976[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4271806009047526976[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4271806009047526976[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4271806009047526976[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4271806009047526976[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4271806009047526976[62] = 0;
   out_4271806009047526976[63] = 0;
   out_4271806009047526976[64] = 0;
   out_4271806009047526976[65] = 0;
   out_4271806009047526976[66] = 0;
   out_4271806009047526976[67] = 0;
   out_4271806009047526976[68] = 0;
   out_4271806009047526976[69] = 0;
   out_4271806009047526976[70] = 1;
   out_4271806009047526976[71] = 0;
   out_4271806009047526976[72] = 0;
   out_4271806009047526976[73] = 0;
   out_4271806009047526976[74] = 0;
   out_4271806009047526976[75] = 0;
   out_4271806009047526976[76] = 0;
   out_4271806009047526976[77] = 0;
   out_4271806009047526976[78] = 0;
   out_4271806009047526976[79] = 0;
   out_4271806009047526976[80] = 1;
}
void h_25(double *state, double *unused, double *out_8182502757940358065) {
   out_8182502757940358065[0] = state[6];
}
void H_25(double *state, double *unused, double *out_756789830708659099) {
   out_756789830708659099[0] = 0;
   out_756789830708659099[1] = 0;
   out_756789830708659099[2] = 0;
   out_756789830708659099[3] = 0;
   out_756789830708659099[4] = 0;
   out_756789830708659099[5] = 0;
   out_756789830708659099[6] = 1;
   out_756789830708659099[7] = 0;
   out_756789830708659099[8] = 0;
}
void h_24(double *state, double *unused, double *out_7859470344960632505) {
   out_7859470344960632505[0] = state[4];
   out_7859470344960632505[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1601786439492354498) {
   out_1601786439492354498[0] = 0;
   out_1601786439492354498[1] = 0;
   out_1601786439492354498[2] = 0;
   out_1601786439492354498[3] = 0;
   out_1601786439492354498[4] = 1;
   out_1601786439492354498[5] = 0;
   out_1601786439492354498[6] = 0;
   out_1601786439492354498[7] = 0;
   out_1601786439492354498[8] = 0;
   out_1601786439492354498[9] = 0;
   out_1601786439492354498[10] = 0;
   out_1601786439492354498[11] = 0;
   out_1601786439492354498[12] = 0;
   out_1601786439492354498[13] = 0;
   out_1601786439492354498[14] = 1;
   out_1601786439492354498[15] = 0;
   out_1601786439492354498[16] = 0;
   out_1601786439492354498[17] = 0;
}
void h_30(double *state, double *unused, double *out_2382968665015144366) {
   out_2382968665015144366[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3275122789215907726) {
   out_3275122789215907726[0] = 0;
   out_3275122789215907726[1] = 0;
   out_3275122789215907726[2] = 0;
   out_3275122789215907726[3] = 0;
   out_3275122789215907726[4] = 1;
   out_3275122789215907726[5] = 0;
   out_3275122789215907726[6] = 0;
   out_3275122789215907726[7] = 0;
   out_3275122789215907726[8] = 0;
}
void h_26(double *state, double *unused, double *out_7384641932675990011) {
   out_7384641932675990011[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2984713488165397125) {
   out_2984713488165397125[0] = 0;
   out_2984713488165397125[1] = 0;
   out_2984713488165397125[2] = 0;
   out_2984713488165397125[3] = 0;
   out_2984713488165397125[4] = 0;
   out_2984713488165397125[5] = 0;
   out_2984713488165397125[6] = 0;
   out_2984713488165397125[7] = 1;
   out_2984713488165397125[8] = 0;
}
void h_27(double *state, double *unused, double *out_1783461240526508919) {
   out_1783461240526508919[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1100359477415482815) {
   out_1100359477415482815[0] = 0;
   out_1100359477415482815[1] = 0;
   out_1100359477415482815[2] = 0;
   out_1100359477415482815[3] = 1;
   out_1100359477415482815[4] = 0;
   out_1100359477415482815[5] = 0;
   out_1100359477415482815[6] = 0;
   out_1100359477415482815[7] = 0;
   out_1100359477415482815[8] = 0;
}
void h_29(double *state, double *unused, double *out_5419754716459477051) {
   out_5419754716459477051[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3785354133530299910) {
   out_3785354133530299910[0] = 0;
   out_3785354133530299910[1] = 1;
   out_3785354133530299910[2] = 0;
   out_3785354133530299910[3] = 0;
   out_3785354133530299910[4] = 0;
   out_3785354133530299910[5] = 0;
   out_3785354133530299910[6] = 0;
   out_3785354133530299910[7] = 0;
   out_3785354133530299910[8] = 0;
}
void h_28(double *state, double *unused, double *out_839951387624846484) {
   out_839951387624846484[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1297044883539230664) {
   out_1297044883539230664[0] = 1;
   out_1297044883539230664[1] = 0;
   out_1297044883539230664[2] = 0;
   out_1297044883539230664[3] = 0;
   out_1297044883539230664[4] = 0;
   out_1297044883539230664[5] = 0;
   out_1297044883539230664[6] = 0;
   out_1297044883539230664[7] = 0;
   out_1297044883539230664[8] = 0;
}
void h_31(double *state, double *unused, double *out_4545053229686335715) {
   out_4545053229686335715[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3610921590398748601) {
   out_3610921590398748601[0] = 0;
   out_3610921590398748601[1] = 0;
   out_3610921590398748601[2] = 0;
   out_3610921590398748601[3] = 0;
   out_3610921590398748601[4] = 0;
   out_3610921590398748601[5] = 0;
   out_3610921590398748601[6] = 0;
   out_3610921590398748601[7] = 0;
   out_3610921590398748601[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_176698825170516537) {
  err_fun(nom_x, delta_x, out_176698825170516537);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7598527359371644641) {
  inv_err_fun(nom_x, true_x, out_7598527359371644641);
}
void car_H_mod_fun(double *state, double *out_3793825779596937896) {
  H_mod_fun(state, out_3793825779596937896);
}
void car_f_fun(double *state, double dt, double *out_2702386721019875926) {
  f_fun(state,  dt, out_2702386721019875926);
}
void car_F_fun(double *state, double dt, double *out_4271806009047526976) {
  F_fun(state,  dt, out_4271806009047526976);
}
void car_h_25(double *state, double *unused, double *out_8182502757940358065) {
  h_25(state, unused, out_8182502757940358065);
}
void car_H_25(double *state, double *unused, double *out_756789830708659099) {
  H_25(state, unused, out_756789830708659099);
}
void car_h_24(double *state, double *unused, double *out_7859470344960632505) {
  h_24(state, unused, out_7859470344960632505);
}
void car_H_24(double *state, double *unused, double *out_1601786439492354498) {
  H_24(state, unused, out_1601786439492354498);
}
void car_h_30(double *state, double *unused, double *out_2382968665015144366) {
  h_30(state, unused, out_2382968665015144366);
}
void car_H_30(double *state, double *unused, double *out_3275122789215907726) {
  H_30(state, unused, out_3275122789215907726);
}
void car_h_26(double *state, double *unused, double *out_7384641932675990011) {
  h_26(state, unused, out_7384641932675990011);
}
void car_H_26(double *state, double *unused, double *out_2984713488165397125) {
  H_26(state, unused, out_2984713488165397125);
}
void car_h_27(double *state, double *unused, double *out_1783461240526508919) {
  h_27(state, unused, out_1783461240526508919);
}
void car_H_27(double *state, double *unused, double *out_1100359477415482815) {
  H_27(state, unused, out_1100359477415482815);
}
void car_h_29(double *state, double *unused, double *out_5419754716459477051) {
  h_29(state, unused, out_5419754716459477051);
}
void car_H_29(double *state, double *unused, double *out_3785354133530299910) {
  H_29(state, unused, out_3785354133530299910);
}
void car_h_28(double *state, double *unused, double *out_839951387624846484) {
  h_28(state, unused, out_839951387624846484);
}
void car_H_28(double *state, double *unused, double *out_1297044883539230664) {
  H_28(state, unused, out_1297044883539230664);
}
void car_h_31(double *state, double *unused, double *out_4545053229686335715) {
  h_31(state, unused, out_4545053229686335715);
}
void car_H_31(double *state, double *unused, double *out_3610921590398748601) {
  H_31(state, unused, out_3610921590398748601);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
