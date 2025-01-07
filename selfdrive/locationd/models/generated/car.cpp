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
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7660645607411207093) {
   out_7660645607411207093[0] = delta_x[0] + nom_x[0];
   out_7660645607411207093[1] = delta_x[1] + nom_x[1];
   out_7660645607411207093[2] = delta_x[2] + nom_x[2];
   out_7660645607411207093[3] = delta_x[3] + nom_x[3];
   out_7660645607411207093[4] = delta_x[4] + nom_x[4];
   out_7660645607411207093[5] = delta_x[5] + nom_x[5];
   out_7660645607411207093[6] = delta_x[6] + nom_x[6];
   out_7660645607411207093[7] = delta_x[7] + nom_x[7];
   out_7660645607411207093[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7452998826362234250) {
   out_7452998826362234250[0] = -nom_x[0] + true_x[0];
   out_7452998826362234250[1] = -nom_x[1] + true_x[1];
   out_7452998826362234250[2] = -nom_x[2] + true_x[2];
   out_7452998826362234250[3] = -nom_x[3] + true_x[3];
   out_7452998826362234250[4] = -nom_x[4] + true_x[4];
   out_7452998826362234250[5] = -nom_x[5] + true_x[5];
   out_7452998826362234250[6] = -nom_x[6] + true_x[6];
   out_7452998826362234250[7] = -nom_x[7] + true_x[7];
   out_7452998826362234250[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1990581163377215491) {
   out_1990581163377215491[0] = 1.0;
   out_1990581163377215491[1] = 0.0;
   out_1990581163377215491[2] = 0.0;
   out_1990581163377215491[3] = 0.0;
   out_1990581163377215491[4] = 0.0;
   out_1990581163377215491[5] = 0.0;
   out_1990581163377215491[6] = 0.0;
   out_1990581163377215491[7] = 0.0;
   out_1990581163377215491[8] = 0.0;
   out_1990581163377215491[9] = 0.0;
   out_1990581163377215491[10] = 1.0;
   out_1990581163377215491[11] = 0.0;
   out_1990581163377215491[12] = 0.0;
   out_1990581163377215491[13] = 0.0;
   out_1990581163377215491[14] = 0.0;
   out_1990581163377215491[15] = 0.0;
   out_1990581163377215491[16] = 0.0;
   out_1990581163377215491[17] = 0.0;
   out_1990581163377215491[18] = 0.0;
   out_1990581163377215491[19] = 0.0;
   out_1990581163377215491[20] = 1.0;
   out_1990581163377215491[21] = 0.0;
   out_1990581163377215491[22] = 0.0;
   out_1990581163377215491[23] = 0.0;
   out_1990581163377215491[24] = 0.0;
   out_1990581163377215491[25] = 0.0;
   out_1990581163377215491[26] = 0.0;
   out_1990581163377215491[27] = 0.0;
   out_1990581163377215491[28] = 0.0;
   out_1990581163377215491[29] = 0.0;
   out_1990581163377215491[30] = 1.0;
   out_1990581163377215491[31] = 0.0;
   out_1990581163377215491[32] = 0.0;
   out_1990581163377215491[33] = 0.0;
   out_1990581163377215491[34] = 0.0;
   out_1990581163377215491[35] = 0.0;
   out_1990581163377215491[36] = 0.0;
   out_1990581163377215491[37] = 0.0;
   out_1990581163377215491[38] = 0.0;
   out_1990581163377215491[39] = 0.0;
   out_1990581163377215491[40] = 1.0;
   out_1990581163377215491[41] = 0.0;
   out_1990581163377215491[42] = 0.0;
   out_1990581163377215491[43] = 0.0;
   out_1990581163377215491[44] = 0.0;
   out_1990581163377215491[45] = 0.0;
   out_1990581163377215491[46] = 0.0;
   out_1990581163377215491[47] = 0.0;
   out_1990581163377215491[48] = 0.0;
   out_1990581163377215491[49] = 0.0;
   out_1990581163377215491[50] = 1.0;
   out_1990581163377215491[51] = 0.0;
   out_1990581163377215491[52] = 0.0;
   out_1990581163377215491[53] = 0.0;
   out_1990581163377215491[54] = 0.0;
   out_1990581163377215491[55] = 0.0;
   out_1990581163377215491[56] = 0.0;
   out_1990581163377215491[57] = 0.0;
   out_1990581163377215491[58] = 0.0;
   out_1990581163377215491[59] = 0.0;
   out_1990581163377215491[60] = 1.0;
   out_1990581163377215491[61] = 0.0;
   out_1990581163377215491[62] = 0.0;
   out_1990581163377215491[63] = 0.0;
   out_1990581163377215491[64] = 0.0;
   out_1990581163377215491[65] = 0.0;
   out_1990581163377215491[66] = 0.0;
   out_1990581163377215491[67] = 0.0;
   out_1990581163377215491[68] = 0.0;
   out_1990581163377215491[69] = 0.0;
   out_1990581163377215491[70] = 1.0;
   out_1990581163377215491[71] = 0.0;
   out_1990581163377215491[72] = 0.0;
   out_1990581163377215491[73] = 0.0;
   out_1990581163377215491[74] = 0.0;
   out_1990581163377215491[75] = 0.0;
   out_1990581163377215491[76] = 0.0;
   out_1990581163377215491[77] = 0.0;
   out_1990581163377215491[78] = 0.0;
   out_1990581163377215491[79] = 0.0;
   out_1990581163377215491[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_511413503708044544) {
   out_511413503708044544[0] = state[0];
   out_511413503708044544[1] = state[1];
   out_511413503708044544[2] = state[2];
   out_511413503708044544[3] = state[3];
   out_511413503708044544[4] = state[4];
   out_511413503708044544[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_511413503708044544[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_511413503708044544[7] = state[7];
   out_511413503708044544[8] = state[8];
}
void F_fun(double *state, double dt, double *out_4436007195428765095) {
   out_4436007195428765095[0] = 1;
   out_4436007195428765095[1] = 0;
   out_4436007195428765095[2] = 0;
   out_4436007195428765095[3] = 0;
   out_4436007195428765095[4] = 0;
   out_4436007195428765095[5] = 0;
   out_4436007195428765095[6] = 0;
   out_4436007195428765095[7] = 0;
   out_4436007195428765095[8] = 0;
   out_4436007195428765095[9] = 0;
   out_4436007195428765095[10] = 1;
   out_4436007195428765095[11] = 0;
   out_4436007195428765095[12] = 0;
   out_4436007195428765095[13] = 0;
   out_4436007195428765095[14] = 0;
   out_4436007195428765095[15] = 0;
   out_4436007195428765095[16] = 0;
   out_4436007195428765095[17] = 0;
   out_4436007195428765095[18] = 0;
   out_4436007195428765095[19] = 0;
   out_4436007195428765095[20] = 1;
   out_4436007195428765095[21] = 0;
   out_4436007195428765095[22] = 0;
   out_4436007195428765095[23] = 0;
   out_4436007195428765095[24] = 0;
   out_4436007195428765095[25] = 0;
   out_4436007195428765095[26] = 0;
   out_4436007195428765095[27] = 0;
   out_4436007195428765095[28] = 0;
   out_4436007195428765095[29] = 0;
   out_4436007195428765095[30] = 1;
   out_4436007195428765095[31] = 0;
   out_4436007195428765095[32] = 0;
   out_4436007195428765095[33] = 0;
   out_4436007195428765095[34] = 0;
   out_4436007195428765095[35] = 0;
   out_4436007195428765095[36] = 0;
   out_4436007195428765095[37] = 0;
   out_4436007195428765095[38] = 0;
   out_4436007195428765095[39] = 0;
   out_4436007195428765095[40] = 1;
   out_4436007195428765095[41] = 0;
   out_4436007195428765095[42] = 0;
   out_4436007195428765095[43] = 0;
   out_4436007195428765095[44] = 0;
   out_4436007195428765095[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4436007195428765095[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4436007195428765095[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4436007195428765095[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4436007195428765095[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4436007195428765095[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4436007195428765095[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4436007195428765095[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4436007195428765095[53] = -9.8000000000000007*dt;
   out_4436007195428765095[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4436007195428765095[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4436007195428765095[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4436007195428765095[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4436007195428765095[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4436007195428765095[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4436007195428765095[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4436007195428765095[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4436007195428765095[62] = 0;
   out_4436007195428765095[63] = 0;
   out_4436007195428765095[64] = 0;
   out_4436007195428765095[65] = 0;
   out_4436007195428765095[66] = 0;
   out_4436007195428765095[67] = 0;
   out_4436007195428765095[68] = 0;
   out_4436007195428765095[69] = 0;
   out_4436007195428765095[70] = 1;
   out_4436007195428765095[71] = 0;
   out_4436007195428765095[72] = 0;
   out_4436007195428765095[73] = 0;
   out_4436007195428765095[74] = 0;
   out_4436007195428765095[75] = 0;
   out_4436007195428765095[76] = 0;
   out_4436007195428765095[77] = 0;
   out_4436007195428765095[78] = 0;
   out_4436007195428765095[79] = 0;
   out_4436007195428765095[80] = 1;
}
void h_25(double *state, double *unused, double *out_8649919120135218251) {
   out_8649919120135218251[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8534824877358476320) {
   out_8534824877358476320[0] = 0;
   out_8534824877358476320[1] = 0;
   out_8534824877358476320[2] = 0;
   out_8534824877358476320[3] = 0;
   out_8534824877358476320[4] = 0;
   out_8534824877358476320[5] = 0;
   out_8534824877358476320[6] = 1;
   out_8534824877358476320[7] = 0;
   out_8534824877358476320[8] = 0;
}
void h_24(double *state, double *unused, double *out_3596809887910607438) {
   out_3596809887910607438[0] = state[4];
   out_3596809887910607438[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4976899278556153184) {
   out_4976899278556153184[0] = 0;
   out_4976899278556153184[1] = 0;
   out_4976899278556153184[2] = 0;
   out_4976899278556153184[3] = 0;
   out_4976899278556153184[4] = 1;
   out_4976899278556153184[5] = 0;
   out_4976899278556153184[6] = 0;
   out_4976899278556153184[7] = 0;
   out_4976899278556153184[8] = 0;
   out_4976899278556153184[9] = 0;
   out_4976899278556153184[10] = 0;
   out_4976899278556153184[11] = 0;
   out_4976899278556153184[12] = 0;
   out_4976899278556153184[13] = 0;
   out_4976899278556153184[14] = 1;
   out_4976899278556153184[15] = 0;
   out_4976899278556153184[16] = 0;
   out_4976899278556153184[17] = 0;
}
void h_30(double *state, double *unused, double *out_756979625250694892) {
   out_756979625250694892[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6016491918851227693) {
   out_6016491918851227693[0] = 0;
   out_6016491918851227693[1] = 0;
   out_6016491918851227693[2] = 0;
   out_6016491918851227693[3] = 0;
   out_6016491918851227693[4] = 1;
   out_6016491918851227693[5] = 0;
   out_6016491918851227693[6] = 0;
   out_6016491918851227693[7] = 0;
   out_6016491918851227693[8] = 0;
}
void h_26(double *state, double *unused, double *out_2132870208787992361) {
   out_2132870208787992361[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6170415877477019072) {
   out_6170415877477019072[0] = 0;
   out_6170415877477019072[1] = 0;
   out_6170415877477019072[2] = 0;
   out_6170415877477019072[3] = 0;
   out_6170415877477019072[4] = 0;
   out_6170415877477019072[5] = 0;
   out_6170415877477019072[6] = 0;
   out_6170415877477019072[7] = 1;
   out_6170415877477019072[8] = 0;
}
void h_27(double *state, double *unused, double *out_4795151685913487720) {
   out_4795151685913487720[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8191255230651652604) {
   out_8191255230651652604[0] = 0;
   out_8191255230651652604[1] = 0;
   out_8191255230651652604[2] = 0;
   out_8191255230651652604[3] = 1;
   out_8191255230651652604[4] = 0;
   out_8191255230651652604[5] = 0;
   out_8191255230651652604[6] = 0;
   out_8191255230651652604[7] = 0;
   out_8191255230651652604[8] = 0;
}
void h_29(double *state, double *unused, double *out_6245783078542370887) {
   out_6245783078542370887[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5506260574536835509) {
   out_5506260574536835509[0] = 0;
   out_5506260574536835509[1] = 1;
   out_5506260574536835509[2] = 0;
   out_5506260574536835509[3] = 0;
   out_5506260574536835509[4] = 0;
   out_5506260574536835509[5] = 0;
   out_5506260574536835509[6] = 0;
   out_5506260574536835509[7] = 0;
   out_5506260574536835509[8] = 0;
}
void h_28(double *state, double *unused, double *out_7831937737357820405) {
   out_7831937737357820405[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7858084482103185533) {
   out_7858084482103185533[0] = 1;
   out_7858084482103185533[1] = 0;
   out_7858084482103185533[2] = 0;
   out_7858084482103185533[3] = 0;
   out_7858084482103185533[4] = 0;
   out_7858084482103185533[5] = 0;
   out_7858084482103185533[6] = 0;
   out_7858084482103185533[7] = 0;
   out_7858084482103185533[8] = 0;
}
void h_31(double *state, double *unused, double *out_5012469591881195901) {
   out_5012469591881195901[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5544207775243667596) {
   out_5544207775243667596[0] = 0;
   out_5544207775243667596[1] = 0;
   out_5544207775243667596[2] = 0;
   out_5544207775243667596[3] = 0;
   out_5544207775243667596[4] = 0;
   out_5544207775243667596[5] = 0;
   out_5544207775243667596[6] = 0;
   out_5544207775243667596[7] = 0;
   out_5544207775243667596[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7660645607411207093) {
  err_fun(nom_x, delta_x, out_7660645607411207093);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7452998826362234250) {
  inv_err_fun(nom_x, true_x, out_7452998826362234250);
}
void car_H_mod_fun(double *state, double *out_1990581163377215491) {
  H_mod_fun(state, out_1990581163377215491);
}
void car_f_fun(double *state, double dt, double *out_511413503708044544) {
  f_fun(state,  dt, out_511413503708044544);
}
void car_F_fun(double *state, double dt, double *out_4436007195428765095) {
  F_fun(state,  dt, out_4436007195428765095);
}
void car_h_25(double *state, double *unused, double *out_8649919120135218251) {
  h_25(state, unused, out_8649919120135218251);
}
void car_H_25(double *state, double *unused, double *out_8534824877358476320) {
  H_25(state, unused, out_8534824877358476320);
}
void car_h_24(double *state, double *unused, double *out_3596809887910607438) {
  h_24(state, unused, out_3596809887910607438);
}
void car_H_24(double *state, double *unused, double *out_4976899278556153184) {
  H_24(state, unused, out_4976899278556153184);
}
void car_h_30(double *state, double *unused, double *out_756979625250694892) {
  h_30(state, unused, out_756979625250694892);
}
void car_H_30(double *state, double *unused, double *out_6016491918851227693) {
  H_30(state, unused, out_6016491918851227693);
}
void car_h_26(double *state, double *unused, double *out_2132870208787992361) {
  h_26(state, unused, out_2132870208787992361);
}
void car_H_26(double *state, double *unused, double *out_6170415877477019072) {
  H_26(state, unused, out_6170415877477019072);
}
void car_h_27(double *state, double *unused, double *out_4795151685913487720) {
  h_27(state, unused, out_4795151685913487720);
}
void car_H_27(double *state, double *unused, double *out_8191255230651652604) {
  H_27(state, unused, out_8191255230651652604);
}
void car_h_29(double *state, double *unused, double *out_6245783078542370887) {
  h_29(state, unused, out_6245783078542370887);
}
void car_H_29(double *state, double *unused, double *out_5506260574536835509) {
  H_29(state, unused, out_5506260574536835509);
}
void car_h_28(double *state, double *unused, double *out_7831937737357820405) {
  h_28(state, unused, out_7831937737357820405);
}
void car_H_28(double *state, double *unused, double *out_7858084482103185533) {
  H_28(state, unused, out_7858084482103185533);
}
void car_h_31(double *state, double *unused, double *out_5012469591881195901) {
  h_31(state, unused, out_5012469591881195901);
}
void car_H_31(double *state, double *unused, double *out_5544207775243667596) {
  H_31(state, unused, out_5544207775243667596);
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
