#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_7660645607411207093);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7452998826362234250);
void car_H_mod_fun(double *state, double *out_1990581163377215491);
void car_f_fun(double *state, double dt, double *out_511413503708044544);
void car_F_fun(double *state, double dt, double *out_4436007195428765095);
void car_h_25(double *state, double *unused, double *out_8649919120135218251);
void car_H_25(double *state, double *unused, double *out_8534824877358476320);
void car_h_24(double *state, double *unused, double *out_3596809887910607438);
void car_H_24(double *state, double *unused, double *out_4976899278556153184);
void car_h_30(double *state, double *unused, double *out_756979625250694892);
void car_H_30(double *state, double *unused, double *out_6016491918851227693);
void car_h_26(double *state, double *unused, double *out_2132870208787992361);
void car_H_26(double *state, double *unused, double *out_6170415877477019072);
void car_h_27(double *state, double *unused, double *out_4795151685913487720);
void car_H_27(double *state, double *unused, double *out_8191255230651652604);
void car_h_29(double *state, double *unused, double *out_6245783078542370887);
void car_H_29(double *state, double *unused, double *out_5506260574536835509);
void car_h_28(double *state, double *unused, double *out_7831937737357820405);
void car_H_28(double *state, double *unused, double *out_7858084482103185533);
void car_h_31(double *state, double *unused, double *out_5012469591881195901);
void car_H_31(double *state, double *unused, double *out_5544207775243667596);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}