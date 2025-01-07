#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7649446209494791890) {
   out_7649446209494791890[0] = delta_x[0] + nom_x[0];
   out_7649446209494791890[1] = delta_x[1] + nom_x[1];
   out_7649446209494791890[2] = delta_x[2] + nom_x[2];
   out_7649446209494791890[3] = delta_x[3] + nom_x[3];
   out_7649446209494791890[4] = delta_x[4] + nom_x[4];
   out_7649446209494791890[5] = delta_x[5] + nom_x[5];
   out_7649446209494791890[6] = delta_x[6] + nom_x[6];
   out_7649446209494791890[7] = delta_x[7] + nom_x[7];
   out_7649446209494791890[8] = delta_x[8] + nom_x[8];
   out_7649446209494791890[9] = delta_x[9] + nom_x[9];
   out_7649446209494791890[10] = delta_x[10] + nom_x[10];
   out_7649446209494791890[11] = delta_x[11] + nom_x[11];
   out_7649446209494791890[12] = delta_x[12] + nom_x[12];
   out_7649446209494791890[13] = delta_x[13] + nom_x[13];
   out_7649446209494791890[14] = delta_x[14] + nom_x[14];
   out_7649446209494791890[15] = delta_x[15] + nom_x[15];
   out_7649446209494791890[16] = delta_x[16] + nom_x[16];
   out_7649446209494791890[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6089283145118711550) {
   out_6089283145118711550[0] = -nom_x[0] + true_x[0];
   out_6089283145118711550[1] = -nom_x[1] + true_x[1];
   out_6089283145118711550[2] = -nom_x[2] + true_x[2];
   out_6089283145118711550[3] = -nom_x[3] + true_x[3];
   out_6089283145118711550[4] = -nom_x[4] + true_x[4];
   out_6089283145118711550[5] = -nom_x[5] + true_x[5];
   out_6089283145118711550[6] = -nom_x[6] + true_x[6];
   out_6089283145118711550[7] = -nom_x[7] + true_x[7];
   out_6089283145118711550[8] = -nom_x[8] + true_x[8];
   out_6089283145118711550[9] = -nom_x[9] + true_x[9];
   out_6089283145118711550[10] = -nom_x[10] + true_x[10];
   out_6089283145118711550[11] = -nom_x[11] + true_x[11];
   out_6089283145118711550[12] = -nom_x[12] + true_x[12];
   out_6089283145118711550[13] = -nom_x[13] + true_x[13];
   out_6089283145118711550[14] = -nom_x[14] + true_x[14];
   out_6089283145118711550[15] = -nom_x[15] + true_x[15];
   out_6089283145118711550[16] = -nom_x[16] + true_x[16];
   out_6089283145118711550[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_6735777404239993353) {
   out_6735777404239993353[0] = 1.0;
   out_6735777404239993353[1] = 0.0;
   out_6735777404239993353[2] = 0.0;
   out_6735777404239993353[3] = 0.0;
   out_6735777404239993353[4] = 0.0;
   out_6735777404239993353[5] = 0.0;
   out_6735777404239993353[6] = 0.0;
   out_6735777404239993353[7] = 0.0;
   out_6735777404239993353[8] = 0.0;
   out_6735777404239993353[9] = 0.0;
   out_6735777404239993353[10] = 0.0;
   out_6735777404239993353[11] = 0.0;
   out_6735777404239993353[12] = 0.0;
   out_6735777404239993353[13] = 0.0;
   out_6735777404239993353[14] = 0.0;
   out_6735777404239993353[15] = 0.0;
   out_6735777404239993353[16] = 0.0;
   out_6735777404239993353[17] = 0.0;
   out_6735777404239993353[18] = 0.0;
   out_6735777404239993353[19] = 1.0;
   out_6735777404239993353[20] = 0.0;
   out_6735777404239993353[21] = 0.0;
   out_6735777404239993353[22] = 0.0;
   out_6735777404239993353[23] = 0.0;
   out_6735777404239993353[24] = 0.0;
   out_6735777404239993353[25] = 0.0;
   out_6735777404239993353[26] = 0.0;
   out_6735777404239993353[27] = 0.0;
   out_6735777404239993353[28] = 0.0;
   out_6735777404239993353[29] = 0.0;
   out_6735777404239993353[30] = 0.0;
   out_6735777404239993353[31] = 0.0;
   out_6735777404239993353[32] = 0.0;
   out_6735777404239993353[33] = 0.0;
   out_6735777404239993353[34] = 0.0;
   out_6735777404239993353[35] = 0.0;
   out_6735777404239993353[36] = 0.0;
   out_6735777404239993353[37] = 0.0;
   out_6735777404239993353[38] = 1.0;
   out_6735777404239993353[39] = 0.0;
   out_6735777404239993353[40] = 0.0;
   out_6735777404239993353[41] = 0.0;
   out_6735777404239993353[42] = 0.0;
   out_6735777404239993353[43] = 0.0;
   out_6735777404239993353[44] = 0.0;
   out_6735777404239993353[45] = 0.0;
   out_6735777404239993353[46] = 0.0;
   out_6735777404239993353[47] = 0.0;
   out_6735777404239993353[48] = 0.0;
   out_6735777404239993353[49] = 0.0;
   out_6735777404239993353[50] = 0.0;
   out_6735777404239993353[51] = 0.0;
   out_6735777404239993353[52] = 0.0;
   out_6735777404239993353[53] = 0.0;
   out_6735777404239993353[54] = 0.0;
   out_6735777404239993353[55] = 0.0;
   out_6735777404239993353[56] = 0.0;
   out_6735777404239993353[57] = 1.0;
   out_6735777404239993353[58] = 0.0;
   out_6735777404239993353[59] = 0.0;
   out_6735777404239993353[60] = 0.0;
   out_6735777404239993353[61] = 0.0;
   out_6735777404239993353[62] = 0.0;
   out_6735777404239993353[63] = 0.0;
   out_6735777404239993353[64] = 0.0;
   out_6735777404239993353[65] = 0.0;
   out_6735777404239993353[66] = 0.0;
   out_6735777404239993353[67] = 0.0;
   out_6735777404239993353[68] = 0.0;
   out_6735777404239993353[69] = 0.0;
   out_6735777404239993353[70] = 0.0;
   out_6735777404239993353[71] = 0.0;
   out_6735777404239993353[72] = 0.0;
   out_6735777404239993353[73] = 0.0;
   out_6735777404239993353[74] = 0.0;
   out_6735777404239993353[75] = 0.0;
   out_6735777404239993353[76] = 1.0;
   out_6735777404239993353[77] = 0.0;
   out_6735777404239993353[78] = 0.0;
   out_6735777404239993353[79] = 0.0;
   out_6735777404239993353[80] = 0.0;
   out_6735777404239993353[81] = 0.0;
   out_6735777404239993353[82] = 0.0;
   out_6735777404239993353[83] = 0.0;
   out_6735777404239993353[84] = 0.0;
   out_6735777404239993353[85] = 0.0;
   out_6735777404239993353[86] = 0.0;
   out_6735777404239993353[87] = 0.0;
   out_6735777404239993353[88] = 0.0;
   out_6735777404239993353[89] = 0.0;
   out_6735777404239993353[90] = 0.0;
   out_6735777404239993353[91] = 0.0;
   out_6735777404239993353[92] = 0.0;
   out_6735777404239993353[93] = 0.0;
   out_6735777404239993353[94] = 0.0;
   out_6735777404239993353[95] = 1.0;
   out_6735777404239993353[96] = 0.0;
   out_6735777404239993353[97] = 0.0;
   out_6735777404239993353[98] = 0.0;
   out_6735777404239993353[99] = 0.0;
   out_6735777404239993353[100] = 0.0;
   out_6735777404239993353[101] = 0.0;
   out_6735777404239993353[102] = 0.0;
   out_6735777404239993353[103] = 0.0;
   out_6735777404239993353[104] = 0.0;
   out_6735777404239993353[105] = 0.0;
   out_6735777404239993353[106] = 0.0;
   out_6735777404239993353[107] = 0.0;
   out_6735777404239993353[108] = 0.0;
   out_6735777404239993353[109] = 0.0;
   out_6735777404239993353[110] = 0.0;
   out_6735777404239993353[111] = 0.0;
   out_6735777404239993353[112] = 0.0;
   out_6735777404239993353[113] = 0.0;
   out_6735777404239993353[114] = 1.0;
   out_6735777404239993353[115] = 0.0;
   out_6735777404239993353[116] = 0.0;
   out_6735777404239993353[117] = 0.0;
   out_6735777404239993353[118] = 0.0;
   out_6735777404239993353[119] = 0.0;
   out_6735777404239993353[120] = 0.0;
   out_6735777404239993353[121] = 0.0;
   out_6735777404239993353[122] = 0.0;
   out_6735777404239993353[123] = 0.0;
   out_6735777404239993353[124] = 0.0;
   out_6735777404239993353[125] = 0.0;
   out_6735777404239993353[126] = 0.0;
   out_6735777404239993353[127] = 0.0;
   out_6735777404239993353[128] = 0.0;
   out_6735777404239993353[129] = 0.0;
   out_6735777404239993353[130] = 0.0;
   out_6735777404239993353[131] = 0.0;
   out_6735777404239993353[132] = 0.0;
   out_6735777404239993353[133] = 1.0;
   out_6735777404239993353[134] = 0.0;
   out_6735777404239993353[135] = 0.0;
   out_6735777404239993353[136] = 0.0;
   out_6735777404239993353[137] = 0.0;
   out_6735777404239993353[138] = 0.0;
   out_6735777404239993353[139] = 0.0;
   out_6735777404239993353[140] = 0.0;
   out_6735777404239993353[141] = 0.0;
   out_6735777404239993353[142] = 0.0;
   out_6735777404239993353[143] = 0.0;
   out_6735777404239993353[144] = 0.0;
   out_6735777404239993353[145] = 0.0;
   out_6735777404239993353[146] = 0.0;
   out_6735777404239993353[147] = 0.0;
   out_6735777404239993353[148] = 0.0;
   out_6735777404239993353[149] = 0.0;
   out_6735777404239993353[150] = 0.0;
   out_6735777404239993353[151] = 0.0;
   out_6735777404239993353[152] = 1.0;
   out_6735777404239993353[153] = 0.0;
   out_6735777404239993353[154] = 0.0;
   out_6735777404239993353[155] = 0.0;
   out_6735777404239993353[156] = 0.0;
   out_6735777404239993353[157] = 0.0;
   out_6735777404239993353[158] = 0.0;
   out_6735777404239993353[159] = 0.0;
   out_6735777404239993353[160] = 0.0;
   out_6735777404239993353[161] = 0.0;
   out_6735777404239993353[162] = 0.0;
   out_6735777404239993353[163] = 0.0;
   out_6735777404239993353[164] = 0.0;
   out_6735777404239993353[165] = 0.0;
   out_6735777404239993353[166] = 0.0;
   out_6735777404239993353[167] = 0.0;
   out_6735777404239993353[168] = 0.0;
   out_6735777404239993353[169] = 0.0;
   out_6735777404239993353[170] = 0.0;
   out_6735777404239993353[171] = 1.0;
   out_6735777404239993353[172] = 0.0;
   out_6735777404239993353[173] = 0.0;
   out_6735777404239993353[174] = 0.0;
   out_6735777404239993353[175] = 0.0;
   out_6735777404239993353[176] = 0.0;
   out_6735777404239993353[177] = 0.0;
   out_6735777404239993353[178] = 0.0;
   out_6735777404239993353[179] = 0.0;
   out_6735777404239993353[180] = 0.0;
   out_6735777404239993353[181] = 0.0;
   out_6735777404239993353[182] = 0.0;
   out_6735777404239993353[183] = 0.0;
   out_6735777404239993353[184] = 0.0;
   out_6735777404239993353[185] = 0.0;
   out_6735777404239993353[186] = 0.0;
   out_6735777404239993353[187] = 0.0;
   out_6735777404239993353[188] = 0.0;
   out_6735777404239993353[189] = 0.0;
   out_6735777404239993353[190] = 1.0;
   out_6735777404239993353[191] = 0.0;
   out_6735777404239993353[192] = 0.0;
   out_6735777404239993353[193] = 0.0;
   out_6735777404239993353[194] = 0.0;
   out_6735777404239993353[195] = 0.0;
   out_6735777404239993353[196] = 0.0;
   out_6735777404239993353[197] = 0.0;
   out_6735777404239993353[198] = 0.0;
   out_6735777404239993353[199] = 0.0;
   out_6735777404239993353[200] = 0.0;
   out_6735777404239993353[201] = 0.0;
   out_6735777404239993353[202] = 0.0;
   out_6735777404239993353[203] = 0.0;
   out_6735777404239993353[204] = 0.0;
   out_6735777404239993353[205] = 0.0;
   out_6735777404239993353[206] = 0.0;
   out_6735777404239993353[207] = 0.0;
   out_6735777404239993353[208] = 0.0;
   out_6735777404239993353[209] = 1.0;
   out_6735777404239993353[210] = 0.0;
   out_6735777404239993353[211] = 0.0;
   out_6735777404239993353[212] = 0.0;
   out_6735777404239993353[213] = 0.0;
   out_6735777404239993353[214] = 0.0;
   out_6735777404239993353[215] = 0.0;
   out_6735777404239993353[216] = 0.0;
   out_6735777404239993353[217] = 0.0;
   out_6735777404239993353[218] = 0.0;
   out_6735777404239993353[219] = 0.0;
   out_6735777404239993353[220] = 0.0;
   out_6735777404239993353[221] = 0.0;
   out_6735777404239993353[222] = 0.0;
   out_6735777404239993353[223] = 0.0;
   out_6735777404239993353[224] = 0.0;
   out_6735777404239993353[225] = 0.0;
   out_6735777404239993353[226] = 0.0;
   out_6735777404239993353[227] = 0.0;
   out_6735777404239993353[228] = 1.0;
   out_6735777404239993353[229] = 0.0;
   out_6735777404239993353[230] = 0.0;
   out_6735777404239993353[231] = 0.0;
   out_6735777404239993353[232] = 0.0;
   out_6735777404239993353[233] = 0.0;
   out_6735777404239993353[234] = 0.0;
   out_6735777404239993353[235] = 0.0;
   out_6735777404239993353[236] = 0.0;
   out_6735777404239993353[237] = 0.0;
   out_6735777404239993353[238] = 0.0;
   out_6735777404239993353[239] = 0.0;
   out_6735777404239993353[240] = 0.0;
   out_6735777404239993353[241] = 0.0;
   out_6735777404239993353[242] = 0.0;
   out_6735777404239993353[243] = 0.0;
   out_6735777404239993353[244] = 0.0;
   out_6735777404239993353[245] = 0.0;
   out_6735777404239993353[246] = 0.0;
   out_6735777404239993353[247] = 1.0;
   out_6735777404239993353[248] = 0.0;
   out_6735777404239993353[249] = 0.0;
   out_6735777404239993353[250] = 0.0;
   out_6735777404239993353[251] = 0.0;
   out_6735777404239993353[252] = 0.0;
   out_6735777404239993353[253] = 0.0;
   out_6735777404239993353[254] = 0.0;
   out_6735777404239993353[255] = 0.0;
   out_6735777404239993353[256] = 0.0;
   out_6735777404239993353[257] = 0.0;
   out_6735777404239993353[258] = 0.0;
   out_6735777404239993353[259] = 0.0;
   out_6735777404239993353[260] = 0.0;
   out_6735777404239993353[261] = 0.0;
   out_6735777404239993353[262] = 0.0;
   out_6735777404239993353[263] = 0.0;
   out_6735777404239993353[264] = 0.0;
   out_6735777404239993353[265] = 0.0;
   out_6735777404239993353[266] = 1.0;
   out_6735777404239993353[267] = 0.0;
   out_6735777404239993353[268] = 0.0;
   out_6735777404239993353[269] = 0.0;
   out_6735777404239993353[270] = 0.0;
   out_6735777404239993353[271] = 0.0;
   out_6735777404239993353[272] = 0.0;
   out_6735777404239993353[273] = 0.0;
   out_6735777404239993353[274] = 0.0;
   out_6735777404239993353[275] = 0.0;
   out_6735777404239993353[276] = 0.0;
   out_6735777404239993353[277] = 0.0;
   out_6735777404239993353[278] = 0.0;
   out_6735777404239993353[279] = 0.0;
   out_6735777404239993353[280] = 0.0;
   out_6735777404239993353[281] = 0.0;
   out_6735777404239993353[282] = 0.0;
   out_6735777404239993353[283] = 0.0;
   out_6735777404239993353[284] = 0.0;
   out_6735777404239993353[285] = 1.0;
   out_6735777404239993353[286] = 0.0;
   out_6735777404239993353[287] = 0.0;
   out_6735777404239993353[288] = 0.0;
   out_6735777404239993353[289] = 0.0;
   out_6735777404239993353[290] = 0.0;
   out_6735777404239993353[291] = 0.0;
   out_6735777404239993353[292] = 0.0;
   out_6735777404239993353[293] = 0.0;
   out_6735777404239993353[294] = 0.0;
   out_6735777404239993353[295] = 0.0;
   out_6735777404239993353[296] = 0.0;
   out_6735777404239993353[297] = 0.0;
   out_6735777404239993353[298] = 0.0;
   out_6735777404239993353[299] = 0.0;
   out_6735777404239993353[300] = 0.0;
   out_6735777404239993353[301] = 0.0;
   out_6735777404239993353[302] = 0.0;
   out_6735777404239993353[303] = 0.0;
   out_6735777404239993353[304] = 1.0;
   out_6735777404239993353[305] = 0.0;
   out_6735777404239993353[306] = 0.0;
   out_6735777404239993353[307] = 0.0;
   out_6735777404239993353[308] = 0.0;
   out_6735777404239993353[309] = 0.0;
   out_6735777404239993353[310] = 0.0;
   out_6735777404239993353[311] = 0.0;
   out_6735777404239993353[312] = 0.0;
   out_6735777404239993353[313] = 0.0;
   out_6735777404239993353[314] = 0.0;
   out_6735777404239993353[315] = 0.0;
   out_6735777404239993353[316] = 0.0;
   out_6735777404239993353[317] = 0.0;
   out_6735777404239993353[318] = 0.0;
   out_6735777404239993353[319] = 0.0;
   out_6735777404239993353[320] = 0.0;
   out_6735777404239993353[321] = 0.0;
   out_6735777404239993353[322] = 0.0;
   out_6735777404239993353[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_4270074648832455255) {
   out_4270074648832455255[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_4270074648832455255[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_4270074648832455255[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_4270074648832455255[3] = dt*state[12] + state[3];
   out_4270074648832455255[4] = dt*state[13] + state[4];
   out_4270074648832455255[5] = dt*state[14] + state[5];
   out_4270074648832455255[6] = state[6];
   out_4270074648832455255[7] = state[7];
   out_4270074648832455255[8] = state[8];
   out_4270074648832455255[9] = state[9];
   out_4270074648832455255[10] = state[10];
   out_4270074648832455255[11] = state[11];
   out_4270074648832455255[12] = state[12];
   out_4270074648832455255[13] = state[13];
   out_4270074648832455255[14] = state[14];
   out_4270074648832455255[15] = state[15];
   out_4270074648832455255[16] = state[16];
   out_4270074648832455255[17] = state[17];
}
void F_fun(double *state, double dt, double *out_7572464312966254438) {
   out_7572464312966254438[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7572464312966254438[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7572464312966254438[2] = 0;
   out_7572464312966254438[3] = 0;
   out_7572464312966254438[4] = 0;
   out_7572464312966254438[5] = 0;
   out_7572464312966254438[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7572464312966254438[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7572464312966254438[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7572464312966254438[9] = 0;
   out_7572464312966254438[10] = 0;
   out_7572464312966254438[11] = 0;
   out_7572464312966254438[12] = 0;
   out_7572464312966254438[13] = 0;
   out_7572464312966254438[14] = 0;
   out_7572464312966254438[15] = 0;
   out_7572464312966254438[16] = 0;
   out_7572464312966254438[17] = 0;
   out_7572464312966254438[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7572464312966254438[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7572464312966254438[20] = 0;
   out_7572464312966254438[21] = 0;
   out_7572464312966254438[22] = 0;
   out_7572464312966254438[23] = 0;
   out_7572464312966254438[24] = 0;
   out_7572464312966254438[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7572464312966254438[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7572464312966254438[27] = 0;
   out_7572464312966254438[28] = 0;
   out_7572464312966254438[29] = 0;
   out_7572464312966254438[30] = 0;
   out_7572464312966254438[31] = 0;
   out_7572464312966254438[32] = 0;
   out_7572464312966254438[33] = 0;
   out_7572464312966254438[34] = 0;
   out_7572464312966254438[35] = 0;
   out_7572464312966254438[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7572464312966254438[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7572464312966254438[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7572464312966254438[39] = 0;
   out_7572464312966254438[40] = 0;
   out_7572464312966254438[41] = 0;
   out_7572464312966254438[42] = 0;
   out_7572464312966254438[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7572464312966254438[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7572464312966254438[45] = 0;
   out_7572464312966254438[46] = 0;
   out_7572464312966254438[47] = 0;
   out_7572464312966254438[48] = 0;
   out_7572464312966254438[49] = 0;
   out_7572464312966254438[50] = 0;
   out_7572464312966254438[51] = 0;
   out_7572464312966254438[52] = 0;
   out_7572464312966254438[53] = 0;
   out_7572464312966254438[54] = 0;
   out_7572464312966254438[55] = 0;
   out_7572464312966254438[56] = 0;
   out_7572464312966254438[57] = 1;
   out_7572464312966254438[58] = 0;
   out_7572464312966254438[59] = 0;
   out_7572464312966254438[60] = 0;
   out_7572464312966254438[61] = 0;
   out_7572464312966254438[62] = 0;
   out_7572464312966254438[63] = 0;
   out_7572464312966254438[64] = 0;
   out_7572464312966254438[65] = 0;
   out_7572464312966254438[66] = dt;
   out_7572464312966254438[67] = 0;
   out_7572464312966254438[68] = 0;
   out_7572464312966254438[69] = 0;
   out_7572464312966254438[70] = 0;
   out_7572464312966254438[71] = 0;
   out_7572464312966254438[72] = 0;
   out_7572464312966254438[73] = 0;
   out_7572464312966254438[74] = 0;
   out_7572464312966254438[75] = 0;
   out_7572464312966254438[76] = 1;
   out_7572464312966254438[77] = 0;
   out_7572464312966254438[78] = 0;
   out_7572464312966254438[79] = 0;
   out_7572464312966254438[80] = 0;
   out_7572464312966254438[81] = 0;
   out_7572464312966254438[82] = 0;
   out_7572464312966254438[83] = 0;
   out_7572464312966254438[84] = 0;
   out_7572464312966254438[85] = dt;
   out_7572464312966254438[86] = 0;
   out_7572464312966254438[87] = 0;
   out_7572464312966254438[88] = 0;
   out_7572464312966254438[89] = 0;
   out_7572464312966254438[90] = 0;
   out_7572464312966254438[91] = 0;
   out_7572464312966254438[92] = 0;
   out_7572464312966254438[93] = 0;
   out_7572464312966254438[94] = 0;
   out_7572464312966254438[95] = 1;
   out_7572464312966254438[96] = 0;
   out_7572464312966254438[97] = 0;
   out_7572464312966254438[98] = 0;
   out_7572464312966254438[99] = 0;
   out_7572464312966254438[100] = 0;
   out_7572464312966254438[101] = 0;
   out_7572464312966254438[102] = 0;
   out_7572464312966254438[103] = 0;
   out_7572464312966254438[104] = dt;
   out_7572464312966254438[105] = 0;
   out_7572464312966254438[106] = 0;
   out_7572464312966254438[107] = 0;
   out_7572464312966254438[108] = 0;
   out_7572464312966254438[109] = 0;
   out_7572464312966254438[110] = 0;
   out_7572464312966254438[111] = 0;
   out_7572464312966254438[112] = 0;
   out_7572464312966254438[113] = 0;
   out_7572464312966254438[114] = 1;
   out_7572464312966254438[115] = 0;
   out_7572464312966254438[116] = 0;
   out_7572464312966254438[117] = 0;
   out_7572464312966254438[118] = 0;
   out_7572464312966254438[119] = 0;
   out_7572464312966254438[120] = 0;
   out_7572464312966254438[121] = 0;
   out_7572464312966254438[122] = 0;
   out_7572464312966254438[123] = 0;
   out_7572464312966254438[124] = 0;
   out_7572464312966254438[125] = 0;
   out_7572464312966254438[126] = 0;
   out_7572464312966254438[127] = 0;
   out_7572464312966254438[128] = 0;
   out_7572464312966254438[129] = 0;
   out_7572464312966254438[130] = 0;
   out_7572464312966254438[131] = 0;
   out_7572464312966254438[132] = 0;
   out_7572464312966254438[133] = 1;
   out_7572464312966254438[134] = 0;
   out_7572464312966254438[135] = 0;
   out_7572464312966254438[136] = 0;
   out_7572464312966254438[137] = 0;
   out_7572464312966254438[138] = 0;
   out_7572464312966254438[139] = 0;
   out_7572464312966254438[140] = 0;
   out_7572464312966254438[141] = 0;
   out_7572464312966254438[142] = 0;
   out_7572464312966254438[143] = 0;
   out_7572464312966254438[144] = 0;
   out_7572464312966254438[145] = 0;
   out_7572464312966254438[146] = 0;
   out_7572464312966254438[147] = 0;
   out_7572464312966254438[148] = 0;
   out_7572464312966254438[149] = 0;
   out_7572464312966254438[150] = 0;
   out_7572464312966254438[151] = 0;
   out_7572464312966254438[152] = 1;
   out_7572464312966254438[153] = 0;
   out_7572464312966254438[154] = 0;
   out_7572464312966254438[155] = 0;
   out_7572464312966254438[156] = 0;
   out_7572464312966254438[157] = 0;
   out_7572464312966254438[158] = 0;
   out_7572464312966254438[159] = 0;
   out_7572464312966254438[160] = 0;
   out_7572464312966254438[161] = 0;
   out_7572464312966254438[162] = 0;
   out_7572464312966254438[163] = 0;
   out_7572464312966254438[164] = 0;
   out_7572464312966254438[165] = 0;
   out_7572464312966254438[166] = 0;
   out_7572464312966254438[167] = 0;
   out_7572464312966254438[168] = 0;
   out_7572464312966254438[169] = 0;
   out_7572464312966254438[170] = 0;
   out_7572464312966254438[171] = 1;
   out_7572464312966254438[172] = 0;
   out_7572464312966254438[173] = 0;
   out_7572464312966254438[174] = 0;
   out_7572464312966254438[175] = 0;
   out_7572464312966254438[176] = 0;
   out_7572464312966254438[177] = 0;
   out_7572464312966254438[178] = 0;
   out_7572464312966254438[179] = 0;
   out_7572464312966254438[180] = 0;
   out_7572464312966254438[181] = 0;
   out_7572464312966254438[182] = 0;
   out_7572464312966254438[183] = 0;
   out_7572464312966254438[184] = 0;
   out_7572464312966254438[185] = 0;
   out_7572464312966254438[186] = 0;
   out_7572464312966254438[187] = 0;
   out_7572464312966254438[188] = 0;
   out_7572464312966254438[189] = 0;
   out_7572464312966254438[190] = 1;
   out_7572464312966254438[191] = 0;
   out_7572464312966254438[192] = 0;
   out_7572464312966254438[193] = 0;
   out_7572464312966254438[194] = 0;
   out_7572464312966254438[195] = 0;
   out_7572464312966254438[196] = 0;
   out_7572464312966254438[197] = 0;
   out_7572464312966254438[198] = 0;
   out_7572464312966254438[199] = 0;
   out_7572464312966254438[200] = 0;
   out_7572464312966254438[201] = 0;
   out_7572464312966254438[202] = 0;
   out_7572464312966254438[203] = 0;
   out_7572464312966254438[204] = 0;
   out_7572464312966254438[205] = 0;
   out_7572464312966254438[206] = 0;
   out_7572464312966254438[207] = 0;
   out_7572464312966254438[208] = 0;
   out_7572464312966254438[209] = 1;
   out_7572464312966254438[210] = 0;
   out_7572464312966254438[211] = 0;
   out_7572464312966254438[212] = 0;
   out_7572464312966254438[213] = 0;
   out_7572464312966254438[214] = 0;
   out_7572464312966254438[215] = 0;
   out_7572464312966254438[216] = 0;
   out_7572464312966254438[217] = 0;
   out_7572464312966254438[218] = 0;
   out_7572464312966254438[219] = 0;
   out_7572464312966254438[220] = 0;
   out_7572464312966254438[221] = 0;
   out_7572464312966254438[222] = 0;
   out_7572464312966254438[223] = 0;
   out_7572464312966254438[224] = 0;
   out_7572464312966254438[225] = 0;
   out_7572464312966254438[226] = 0;
   out_7572464312966254438[227] = 0;
   out_7572464312966254438[228] = 1;
   out_7572464312966254438[229] = 0;
   out_7572464312966254438[230] = 0;
   out_7572464312966254438[231] = 0;
   out_7572464312966254438[232] = 0;
   out_7572464312966254438[233] = 0;
   out_7572464312966254438[234] = 0;
   out_7572464312966254438[235] = 0;
   out_7572464312966254438[236] = 0;
   out_7572464312966254438[237] = 0;
   out_7572464312966254438[238] = 0;
   out_7572464312966254438[239] = 0;
   out_7572464312966254438[240] = 0;
   out_7572464312966254438[241] = 0;
   out_7572464312966254438[242] = 0;
   out_7572464312966254438[243] = 0;
   out_7572464312966254438[244] = 0;
   out_7572464312966254438[245] = 0;
   out_7572464312966254438[246] = 0;
   out_7572464312966254438[247] = 1;
   out_7572464312966254438[248] = 0;
   out_7572464312966254438[249] = 0;
   out_7572464312966254438[250] = 0;
   out_7572464312966254438[251] = 0;
   out_7572464312966254438[252] = 0;
   out_7572464312966254438[253] = 0;
   out_7572464312966254438[254] = 0;
   out_7572464312966254438[255] = 0;
   out_7572464312966254438[256] = 0;
   out_7572464312966254438[257] = 0;
   out_7572464312966254438[258] = 0;
   out_7572464312966254438[259] = 0;
   out_7572464312966254438[260] = 0;
   out_7572464312966254438[261] = 0;
   out_7572464312966254438[262] = 0;
   out_7572464312966254438[263] = 0;
   out_7572464312966254438[264] = 0;
   out_7572464312966254438[265] = 0;
   out_7572464312966254438[266] = 1;
   out_7572464312966254438[267] = 0;
   out_7572464312966254438[268] = 0;
   out_7572464312966254438[269] = 0;
   out_7572464312966254438[270] = 0;
   out_7572464312966254438[271] = 0;
   out_7572464312966254438[272] = 0;
   out_7572464312966254438[273] = 0;
   out_7572464312966254438[274] = 0;
   out_7572464312966254438[275] = 0;
   out_7572464312966254438[276] = 0;
   out_7572464312966254438[277] = 0;
   out_7572464312966254438[278] = 0;
   out_7572464312966254438[279] = 0;
   out_7572464312966254438[280] = 0;
   out_7572464312966254438[281] = 0;
   out_7572464312966254438[282] = 0;
   out_7572464312966254438[283] = 0;
   out_7572464312966254438[284] = 0;
   out_7572464312966254438[285] = 1;
   out_7572464312966254438[286] = 0;
   out_7572464312966254438[287] = 0;
   out_7572464312966254438[288] = 0;
   out_7572464312966254438[289] = 0;
   out_7572464312966254438[290] = 0;
   out_7572464312966254438[291] = 0;
   out_7572464312966254438[292] = 0;
   out_7572464312966254438[293] = 0;
   out_7572464312966254438[294] = 0;
   out_7572464312966254438[295] = 0;
   out_7572464312966254438[296] = 0;
   out_7572464312966254438[297] = 0;
   out_7572464312966254438[298] = 0;
   out_7572464312966254438[299] = 0;
   out_7572464312966254438[300] = 0;
   out_7572464312966254438[301] = 0;
   out_7572464312966254438[302] = 0;
   out_7572464312966254438[303] = 0;
   out_7572464312966254438[304] = 1;
   out_7572464312966254438[305] = 0;
   out_7572464312966254438[306] = 0;
   out_7572464312966254438[307] = 0;
   out_7572464312966254438[308] = 0;
   out_7572464312966254438[309] = 0;
   out_7572464312966254438[310] = 0;
   out_7572464312966254438[311] = 0;
   out_7572464312966254438[312] = 0;
   out_7572464312966254438[313] = 0;
   out_7572464312966254438[314] = 0;
   out_7572464312966254438[315] = 0;
   out_7572464312966254438[316] = 0;
   out_7572464312966254438[317] = 0;
   out_7572464312966254438[318] = 0;
   out_7572464312966254438[319] = 0;
   out_7572464312966254438[320] = 0;
   out_7572464312966254438[321] = 0;
   out_7572464312966254438[322] = 0;
   out_7572464312966254438[323] = 1;
}
void h_4(double *state, double *unused, double *out_5919563708435201550) {
   out_5919563708435201550[0] = state[6] + state[9];
   out_5919563708435201550[1] = state[7] + state[10];
   out_5919563708435201550[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_8703345635194391827) {
   out_8703345635194391827[0] = 0;
   out_8703345635194391827[1] = 0;
   out_8703345635194391827[2] = 0;
   out_8703345635194391827[3] = 0;
   out_8703345635194391827[4] = 0;
   out_8703345635194391827[5] = 0;
   out_8703345635194391827[6] = 1;
   out_8703345635194391827[7] = 0;
   out_8703345635194391827[8] = 0;
   out_8703345635194391827[9] = 1;
   out_8703345635194391827[10] = 0;
   out_8703345635194391827[11] = 0;
   out_8703345635194391827[12] = 0;
   out_8703345635194391827[13] = 0;
   out_8703345635194391827[14] = 0;
   out_8703345635194391827[15] = 0;
   out_8703345635194391827[16] = 0;
   out_8703345635194391827[17] = 0;
   out_8703345635194391827[18] = 0;
   out_8703345635194391827[19] = 0;
   out_8703345635194391827[20] = 0;
   out_8703345635194391827[21] = 0;
   out_8703345635194391827[22] = 0;
   out_8703345635194391827[23] = 0;
   out_8703345635194391827[24] = 0;
   out_8703345635194391827[25] = 1;
   out_8703345635194391827[26] = 0;
   out_8703345635194391827[27] = 0;
   out_8703345635194391827[28] = 1;
   out_8703345635194391827[29] = 0;
   out_8703345635194391827[30] = 0;
   out_8703345635194391827[31] = 0;
   out_8703345635194391827[32] = 0;
   out_8703345635194391827[33] = 0;
   out_8703345635194391827[34] = 0;
   out_8703345635194391827[35] = 0;
   out_8703345635194391827[36] = 0;
   out_8703345635194391827[37] = 0;
   out_8703345635194391827[38] = 0;
   out_8703345635194391827[39] = 0;
   out_8703345635194391827[40] = 0;
   out_8703345635194391827[41] = 0;
   out_8703345635194391827[42] = 0;
   out_8703345635194391827[43] = 0;
   out_8703345635194391827[44] = 1;
   out_8703345635194391827[45] = 0;
   out_8703345635194391827[46] = 0;
   out_8703345635194391827[47] = 1;
   out_8703345635194391827[48] = 0;
   out_8703345635194391827[49] = 0;
   out_8703345635194391827[50] = 0;
   out_8703345635194391827[51] = 0;
   out_8703345635194391827[52] = 0;
   out_8703345635194391827[53] = 0;
}
void h_10(double *state, double *unused, double *out_6637153998626932523) {
   out_6637153998626932523[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_6637153998626932523[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_6637153998626932523[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_7474852757966042504) {
   out_7474852757966042504[0] = 0;
   out_7474852757966042504[1] = 9.8100000000000005*cos(state[1]);
   out_7474852757966042504[2] = 0;
   out_7474852757966042504[3] = 0;
   out_7474852757966042504[4] = -state[8];
   out_7474852757966042504[5] = state[7];
   out_7474852757966042504[6] = 0;
   out_7474852757966042504[7] = state[5];
   out_7474852757966042504[8] = -state[4];
   out_7474852757966042504[9] = 0;
   out_7474852757966042504[10] = 0;
   out_7474852757966042504[11] = 0;
   out_7474852757966042504[12] = 1;
   out_7474852757966042504[13] = 0;
   out_7474852757966042504[14] = 0;
   out_7474852757966042504[15] = 1;
   out_7474852757966042504[16] = 0;
   out_7474852757966042504[17] = 0;
   out_7474852757966042504[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_7474852757966042504[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_7474852757966042504[20] = 0;
   out_7474852757966042504[21] = state[8];
   out_7474852757966042504[22] = 0;
   out_7474852757966042504[23] = -state[6];
   out_7474852757966042504[24] = -state[5];
   out_7474852757966042504[25] = 0;
   out_7474852757966042504[26] = state[3];
   out_7474852757966042504[27] = 0;
   out_7474852757966042504[28] = 0;
   out_7474852757966042504[29] = 0;
   out_7474852757966042504[30] = 0;
   out_7474852757966042504[31] = 1;
   out_7474852757966042504[32] = 0;
   out_7474852757966042504[33] = 0;
   out_7474852757966042504[34] = 1;
   out_7474852757966042504[35] = 0;
   out_7474852757966042504[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_7474852757966042504[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_7474852757966042504[38] = 0;
   out_7474852757966042504[39] = -state[7];
   out_7474852757966042504[40] = state[6];
   out_7474852757966042504[41] = 0;
   out_7474852757966042504[42] = state[4];
   out_7474852757966042504[43] = -state[3];
   out_7474852757966042504[44] = 0;
   out_7474852757966042504[45] = 0;
   out_7474852757966042504[46] = 0;
   out_7474852757966042504[47] = 0;
   out_7474852757966042504[48] = 0;
   out_7474852757966042504[49] = 0;
   out_7474852757966042504[50] = 1;
   out_7474852757966042504[51] = 0;
   out_7474852757966042504[52] = 0;
   out_7474852757966042504[53] = 1;
}
void h_13(double *state, double *unused, double *out_5074029213282930289) {
   out_5074029213282930289[0] = state[3];
   out_5074029213282930289[1] = state[4];
   out_5074029213282930289[2] = state[5];
}
void H_13(double *state, double *unused, double *out_5909642975212635765) {
   out_5909642975212635765[0] = 0;
   out_5909642975212635765[1] = 0;
   out_5909642975212635765[2] = 0;
   out_5909642975212635765[3] = 1;
   out_5909642975212635765[4] = 0;
   out_5909642975212635765[5] = 0;
   out_5909642975212635765[6] = 0;
   out_5909642975212635765[7] = 0;
   out_5909642975212635765[8] = 0;
   out_5909642975212635765[9] = 0;
   out_5909642975212635765[10] = 0;
   out_5909642975212635765[11] = 0;
   out_5909642975212635765[12] = 0;
   out_5909642975212635765[13] = 0;
   out_5909642975212635765[14] = 0;
   out_5909642975212635765[15] = 0;
   out_5909642975212635765[16] = 0;
   out_5909642975212635765[17] = 0;
   out_5909642975212635765[18] = 0;
   out_5909642975212635765[19] = 0;
   out_5909642975212635765[20] = 0;
   out_5909642975212635765[21] = 0;
   out_5909642975212635765[22] = 1;
   out_5909642975212635765[23] = 0;
   out_5909642975212635765[24] = 0;
   out_5909642975212635765[25] = 0;
   out_5909642975212635765[26] = 0;
   out_5909642975212635765[27] = 0;
   out_5909642975212635765[28] = 0;
   out_5909642975212635765[29] = 0;
   out_5909642975212635765[30] = 0;
   out_5909642975212635765[31] = 0;
   out_5909642975212635765[32] = 0;
   out_5909642975212635765[33] = 0;
   out_5909642975212635765[34] = 0;
   out_5909642975212635765[35] = 0;
   out_5909642975212635765[36] = 0;
   out_5909642975212635765[37] = 0;
   out_5909642975212635765[38] = 0;
   out_5909642975212635765[39] = 0;
   out_5909642975212635765[40] = 0;
   out_5909642975212635765[41] = 1;
   out_5909642975212635765[42] = 0;
   out_5909642975212635765[43] = 0;
   out_5909642975212635765[44] = 0;
   out_5909642975212635765[45] = 0;
   out_5909642975212635765[46] = 0;
   out_5909642975212635765[47] = 0;
   out_5909642975212635765[48] = 0;
   out_5909642975212635765[49] = 0;
   out_5909642975212635765[50] = 0;
   out_5909642975212635765[51] = 0;
   out_5909642975212635765[52] = 0;
   out_5909642975212635765[53] = 0;
}
void h_14(double *state, double *unused, double *out_2948074967593073683) {
   out_2948074967593073683[0] = state[6];
   out_2948074967593073683[1] = state[7];
   out_2948074967593073683[2] = state[8];
}
void H_14(double *state, double *unused, double *out_6660610006219787493) {
   out_6660610006219787493[0] = 0;
   out_6660610006219787493[1] = 0;
   out_6660610006219787493[2] = 0;
   out_6660610006219787493[3] = 0;
   out_6660610006219787493[4] = 0;
   out_6660610006219787493[5] = 0;
   out_6660610006219787493[6] = 1;
   out_6660610006219787493[7] = 0;
   out_6660610006219787493[8] = 0;
   out_6660610006219787493[9] = 0;
   out_6660610006219787493[10] = 0;
   out_6660610006219787493[11] = 0;
   out_6660610006219787493[12] = 0;
   out_6660610006219787493[13] = 0;
   out_6660610006219787493[14] = 0;
   out_6660610006219787493[15] = 0;
   out_6660610006219787493[16] = 0;
   out_6660610006219787493[17] = 0;
   out_6660610006219787493[18] = 0;
   out_6660610006219787493[19] = 0;
   out_6660610006219787493[20] = 0;
   out_6660610006219787493[21] = 0;
   out_6660610006219787493[22] = 0;
   out_6660610006219787493[23] = 0;
   out_6660610006219787493[24] = 0;
   out_6660610006219787493[25] = 1;
   out_6660610006219787493[26] = 0;
   out_6660610006219787493[27] = 0;
   out_6660610006219787493[28] = 0;
   out_6660610006219787493[29] = 0;
   out_6660610006219787493[30] = 0;
   out_6660610006219787493[31] = 0;
   out_6660610006219787493[32] = 0;
   out_6660610006219787493[33] = 0;
   out_6660610006219787493[34] = 0;
   out_6660610006219787493[35] = 0;
   out_6660610006219787493[36] = 0;
   out_6660610006219787493[37] = 0;
   out_6660610006219787493[38] = 0;
   out_6660610006219787493[39] = 0;
   out_6660610006219787493[40] = 0;
   out_6660610006219787493[41] = 0;
   out_6660610006219787493[42] = 0;
   out_6660610006219787493[43] = 0;
   out_6660610006219787493[44] = 1;
   out_6660610006219787493[45] = 0;
   out_6660610006219787493[46] = 0;
   out_6660610006219787493[47] = 0;
   out_6660610006219787493[48] = 0;
   out_6660610006219787493[49] = 0;
   out_6660610006219787493[50] = 0;
   out_6660610006219787493[51] = 0;
   out_6660610006219787493[52] = 0;
   out_6660610006219787493[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_7649446209494791890) {
  err_fun(nom_x, delta_x, out_7649446209494791890);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6089283145118711550) {
  inv_err_fun(nom_x, true_x, out_6089283145118711550);
}
void pose_H_mod_fun(double *state, double *out_6735777404239993353) {
  H_mod_fun(state, out_6735777404239993353);
}
void pose_f_fun(double *state, double dt, double *out_4270074648832455255) {
  f_fun(state,  dt, out_4270074648832455255);
}
void pose_F_fun(double *state, double dt, double *out_7572464312966254438) {
  F_fun(state,  dt, out_7572464312966254438);
}
void pose_h_4(double *state, double *unused, double *out_5919563708435201550) {
  h_4(state, unused, out_5919563708435201550);
}
void pose_H_4(double *state, double *unused, double *out_8703345635194391827) {
  H_4(state, unused, out_8703345635194391827);
}
void pose_h_10(double *state, double *unused, double *out_6637153998626932523) {
  h_10(state, unused, out_6637153998626932523);
}
void pose_H_10(double *state, double *unused, double *out_7474852757966042504) {
  H_10(state, unused, out_7474852757966042504);
}
void pose_h_13(double *state, double *unused, double *out_5074029213282930289) {
  h_13(state, unused, out_5074029213282930289);
}
void pose_H_13(double *state, double *unused, double *out_5909642975212635765) {
  H_13(state, unused, out_5909642975212635765);
}
void pose_h_14(double *state, double *unused, double *out_2948074967593073683) {
  h_14(state, unused, out_2948074967593073683);
}
void pose_H_14(double *state, double *unused, double *out_6660610006219787493) {
  H_14(state, unused, out_6660610006219787493);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
