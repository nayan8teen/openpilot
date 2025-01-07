#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_7649446209494791890);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6089283145118711550);
void pose_H_mod_fun(double *state, double *out_6735777404239993353);
void pose_f_fun(double *state, double dt, double *out_4270074648832455255);
void pose_F_fun(double *state, double dt, double *out_7572464312966254438);
void pose_h_4(double *state, double *unused, double *out_5919563708435201550);
void pose_H_4(double *state, double *unused, double *out_8703345635194391827);
void pose_h_10(double *state, double *unused, double *out_6637153998626932523);
void pose_H_10(double *state, double *unused, double *out_7474852757966042504);
void pose_h_13(double *state, double *unused, double *out_5074029213282930289);
void pose_H_13(double *state, double *unused, double *out_5909642975212635765);
void pose_h_14(double *state, double *unused, double *out_2948074967593073683);
void pose_H_14(double *state, double *unused, double *out_6660610006219787493);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}