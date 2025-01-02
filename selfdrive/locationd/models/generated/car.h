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
void car_err_fun(double *nom_x, double *delta_x, double *out_176698825170516537);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7598527359371644641);
void car_H_mod_fun(double *state, double *out_3793825779596937896);
void car_f_fun(double *state, double dt, double *out_2702386721019875926);
void car_F_fun(double *state, double dt, double *out_4271806009047526976);
void car_h_25(double *state, double *unused, double *out_8182502757940358065);
void car_H_25(double *state, double *unused, double *out_756789830708659099);
void car_h_24(double *state, double *unused, double *out_7859470344960632505);
void car_H_24(double *state, double *unused, double *out_1601786439492354498);
void car_h_30(double *state, double *unused, double *out_2382968665015144366);
void car_H_30(double *state, double *unused, double *out_3275122789215907726);
void car_h_26(double *state, double *unused, double *out_7384641932675990011);
void car_H_26(double *state, double *unused, double *out_2984713488165397125);
void car_h_27(double *state, double *unused, double *out_1783461240526508919);
void car_H_27(double *state, double *unused, double *out_1100359477415482815);
void car_h_29(double *state, double *unused, double *out_5419754716459477051);
void car_H_29(double *state, double *unused, double *out_3785354133530299910);
void car_h_28(double *state, double *unused, double *out_839951387624846484);
void car_H_28(double *state, double *unused, double *out_1297044883539230664);
void car_h_31(double *state, double *unused, double *out_4545053229686335715);
void car_H_31(double *state, double *unused, double *out_3610921590398748601);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}