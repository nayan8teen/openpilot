#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_8839413807771918444);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2623138420433239329);
void pose_H_mod_fun(double *state, double *out_56233860221111886);
void pose_f_fun(double *state, double dt, double *out_9100409772207968194);
void pose_F_fun(double *state, double dt, double *out_2832904802220701503);
void pose_h_4(double *state, double *unused, double *out_1541878422273639399);
void pose_H_4(double *state, double *unused, double *out_3217988218569837367);
void pose_h_10(double *state, double *unused, double *out_4286388877957954392);
void pose_H_10(double *state, double *unused, double *out_387235683895473982);
void pose_h_13(double *state, double *unused, double *out_771705969490129500);
void pose_H_13(double *state, double *unused, double *out_5714393237504566);
void pose_h_14(double *state, double *unused, double *out_3822203570969529950);
void pose_H_14(double *state, double *unused, double *out_745252637769647162);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}