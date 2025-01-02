#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_6429159793512833762);
void live_err_fun(double *nom_x, double *delta_x, double *out_2432843096470106444);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2543099680316541581);
void live_H_mod_fun(double *state, double *out_9002166145602040971);
void live_f_fun(double *state, double dt, double *out_6635994192670462837);
void live_F_fun(double *state, double dt, double *out_6046334446440756465);
void live_h_4(double *state, double *unused, double *out_3410853152139542647);
void live_H_4(double *state, double *unused, double *out_8330222645038671939);
void live_h_9(double *state, double *unused, double *out_1923027391768355318);
void live_H_9(double *state, double *unused, double *out_8089032998409081294);
void live_h_10(double *state, double *unused, double *out_112815387236844413);
void live_H_10(double *state, double *unused, double *out_7285093679921105941);
void live_h_12(double *state, double *unused, double *out_5922558555567008708);
void live_H_12(double *state, double *unused, double *out_3310766237006710144);
void live_h_35(double *state, double *unused, double *out_5746639093108787682);
void live_H_35(double *state, double *unused, double *out_4963560587666064563);
void live_h_32(double *state, double *unused, double *out_5062780743687619119);
void live_H_32(double *state, double *unused, double *out_2258536074764314626);
void live_h_13(double *state, double *unused, double *out_9192172467598087044);
void live_H_13(double *state, double *unused, double *out_6327205684658223159);
void live_h_14(double *state, double *unused, double *out_1923027391768355318);
void live_H_14(double *state, double *unused, double *out_8089032998409081294);
void live_h_33(double *state, double *unused, double *out_8743410660370455364);
void live_H_33(double *state, double *unused, double *out_1813003583027206959);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}