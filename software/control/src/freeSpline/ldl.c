/* Produced by CVXGEN, 2015-03-26 23:25:30 -0400.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: ldl.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
/* Be sure to place ldl_solve first, so storage schemes are defined by it. */
void ldl_solve(double *target, double *var) {
  int i;
  /* Find var = (L*diag(work.d)*L') \ target, then unpermute. */
  /* Answer goes into var. */
  /* Forward substitution. */
  /* Include permutation as we retrieve from target. Use v so we can unpermute */
  /* later. */
  work.v[0] = target[73];
  work.v[1] = target[74];
  work.v[2] = target[75];
  work.v[3] = target[76];
  work.v[4] = target[77];
  work.v[5] = target[78];
  work.v[6] = target[79];
  work.v[7] = target[80];
  work.v[8] = target[81];
  work.v[9] = target[82];
  work.v[10] = target[83];
  work.v[11] = target[84];
  work.v[12] = target[85];
  work.v[13] = target[86];
  work.v[14] = target[87];
  work.v[15] = target[88];
  work.v[16] = target[89];
  work.v[17] = target[90];
  work.v[18] = target[91];
  work.v[19] = target[92];
  work.v[20] = target[93];
  work.v[21] = target[94];
  work.v[22] = target[95];
  work.v[23] = target[96];
  work.v[24] = target[97];
  work.v[25] = target[98];
  work.v[26] = target[99];
  work.v[27] = target[100];
  work.v[28] = target[101];
  work.v[29] = target[102];
  work.v[30] = target[103];
  work.v[31] = target[104];
  work.v[32] = target[105];
  work.v[33] = target[106];
  work.v[34] = target[107];
  work.v[35] = target[108];
  work.v[36] = target[109];
  work.v[37] = target[110];
  work.v[38] = target[111];
  work.v[39] = target[112];
  work.v[40] = target[113];
  work.v[41] = target[114];
  work.v[42] = target[115];
  work.v[43] = target[116];
  work.v[44] = target[117];
  work.v[45] = target[118];
  work.v[46] = target[119];
  work.v[47] = target[120];
  work.v[48] = target[121];
  work.v[49] = target[122];
  work.v[50] = target[123];
  work.v[51] = target[124];
  work.v[52] = target[125];
  work.v[53] = target[126];
  work.v[54] = target[127];
  work.v[55] = target[128];
  work.v[56] = target[129];
  work.v[57] = target[130];
  work.v[58] = target[131];
  work.v[59] = target[132];
  work.v[60] = target[133];
  work.v[61] = target[134];
  work.v[62] = target[135];
  work.v[63] = target[136];
  work.v[64] = target[137];
  work.v[65] = target[138];
  work.v[66] = target[139];
  work.v[67] = target[140];
  work.v[68] = target[141];
  work.v[69] = target[142];
  work.v[70] = target[143];
  work.v[71] = target[144];
  work.v[72] = target[145];
  work.v[73] = target[146];
  work.v[74] = target[147];
  work.v[75] = target[148];
  work.v[76] = target[201]-work.L[0]*work.v[52];
  work.v[77] = target[207]-work.L[1]*work.v[58];
  work.v[78] = target[202]-work.L[2]*work.v[53];
  work.v[79] = target[208]-work.L[3]*work.v[59];
  work.v[80] = target[203]-work.L[4]*work.v[54];
  work.v[81] = target[209]-work.L[5]*work.v[60];
  work.v[82] = target[204]-work.L[6]*work.v[55];
  work.v[83] = target[210]-work.L[7]*work.v[61];
  work.v[84] = target[205]-work.L[8]*work.v[56];
  work.v[85] = target[211]-work.L[9]*work.v[62];
  work.v[86] = target[206]-work.L[10]*work.v[57];
  work.v[87] = target[212]-work.L[11]*work.v[63];
  work.v[88] = target[213]-work.L[12]*work.v[64];
  work.v[89] = target[219]-work.L[13]*work.v[70];
  work.v[90] = target[214]-work.L[14]*work.v[65];
  work.v[91] = target[220]-work.L[15]*work.v[71];
  work.v[92] = target[215]-work.L[16]*work.v[66];
  work.v[93] = target[221]-work.L[17]*work.v[72];
  work.v[94] = target[216]-work.L[18]*work.v[67];
  work.v[95] = target[222]-work.L[19]*work.v[73];
  work.v[96] = target[217]-work.L[20]*work.v[68];
  work.v[97] = target[223]-work.L[21]*work.v[74];
  work.v[98] = target[218]-work.L[22]*work.v[69];
  work.v[99] = target[224]-work.L[23]*work.v[75];
  work.v[100] = target[237];
  work.v[101] = target[0]-work.L[24]*work.v[100];
  work.v[102] = target[238];
  work.v[103] = target[1]-work.L[25]*work.v[102];
  work.v[104] = target[239];
  work.v[105] = target[2]-work.L[26]*work.v[104];
  work.v[106] = target[240];
  work.v[107] = target[3]-work.L[27]*work.v[106];
  work.v[108] = target[241];
  work.v[109] = target[4]-work.L[28]*work.v[108];
  work.v[110] = target[242];
  work.v[111] = target[5]-work.L[29]*work.v[110];
  work.v[112] = target[243];
  work.v[113] = target[244];
  work.v[114] = target[245];
  work.v[115] = target[246];
  work.v[116] = target[247];
  work.v[117] = target[248];
  work.v[118] = target[6]-work.L[30]*work.v[112];
  work.v[119] = target[7]-work.L[31]*work.v[113];
  work.v[120] = target[8]-work.L[32]*work.v[114];
  work.v[121] = target[9]-work.L[33]*work.v[115];
  work.v[122] = target[10]-work.L[34]*work.v[116];
  work.v[123] = target[11]-work.L[35]*work.v[117];
  work.v[124] = target[24]-work.L[36]*work.v[76]-work.L[37]*work.v[77];
  work.v[125] = target[25]-work.L[38]*work.v[78]-work.L[39]*work.v[79];
  work.v[126] = target[26]-work.L[40]*work.v[80]-work.L[41]*work.v[81];
  work.v[127] = target[27]-work.L[42]*work.v[82]-work.L[43]*work.v[83];
  work.v[128] = target[28]-work.L[44]*work.v[84]-work.L[45]*work.v[85];
  work.v[129] = target[29]-work.L[46]*work.v[86]-work.L[47]*work.v[87];
  work.v[130] = target[49];
  work.v[131] = target[150]-work.L[48]*work.v[1]-work.L[49]*work.v[130];
  work.v[132] = target[156]-work.L[50]*work.v[7]-work.L[51]*work.v[130]-work.L[52]*work.v[131];
  work.v[133] = target[50];
  work.v[134] = target[151]-work.L[53]*work.v[2]-work.L[54]*work.v[133];
  work.v[135] = target[157]-work.L[55]*work.v[8]-work.L[56]*work.v[133]-work.L[57]*work.v[134];
  work.v[136] = target[51];
  work.v[137] = target[152]-work.L[58]*work.v[3]-work.L[59]*work.v[136];
  work.v[138] = target[158]-work.L[60]*work.v[9]-work.L[61]*work.v[136]-work.L[62]*work.v[137];
  work.v[139] = target[52];
  work.v[140] = target[153]-work.L[63]*work.v[4]-work.L[64]*work.v[139];
  work.v[141] = target[159]-work.L[65]*work.v[10]-work.L[66]*work.v[139]-work.L[67]*work.v[140];
  work.v[142] = target[53];
  work.v[143] = target[154]-work.L[68]*work.v[5]-work.L[69]*work.v[142];
  work.v[144] = target[160]-work.L[70]*work.v[11]-work.L[71]*work.v[142]-work.L[72]*work.v[143];
  work.v[145] = target[54];
  work.v[146] = target[155]-work.L[73]*work.v[6]-work.L[74]*work.v[145];
  work.v[147] = target[161]-work.L[75]*work.v[12]-work.L[76]*work.v[145]-work.L[77]*work.v[146];
  work.v[148] = target[55];
  work.v[149] = target[56];
  work.v[150] = target[57];
  work.v[151] = target[58];
  work.v[152] = target[59];
  work.v[153] = target[60];
  work.v[154] = target[61];
  work.v[155] = target[176]-work.L[78]*work.v[27]-work.L[79]*work.v[154];
  work.v[156] = target[182]-work.L[80]*work.v[33]-work.L[81]*work.v[154]-work.L[82]*work.v[155];
  work.v[157] = target[62];
  work.v[158] = target[177]-work.L[83]*work.v[28]-work.L[84]*work.v[157];
  work.v[159] = target[183]-work.L[85]*work.v[34]-work.L[86]*work.v[157]-work.L[87]*work.v[158];
  work.v[160] = target[63];
  work.v[161] = target[178]-work.L[88]*work.v[29]-work.L[89]*work.v[160];
  work.v[162] = target[184]-work.L[90]*work.v[35]-work.L[91]*work.v[160]-work.L[92]*work.v[161];
  work.v[163] = target[64];
  work.v[164] = target[179]-work.L[93]*work.v[30]-work.L[94]*work.v[163];
  work.v[165] = target[185]-work.L[95]*work.v[36]-work.L[96]*work.v[163]-work.L[97]*work.v[164];
  work.v[166] = target[65];
  work.v[167] = target[180]-work.L[98]*work.v[31]-work.L[99]*work.v[166];
  work.v[168] = target[186]-work.L[100]*work.v[37]-work.L[101]*work.v[166]-work.L[102]*work.v[167];
  work.v[169] = target[66];
  work.v[170] = target[181]-work.L[103]*work.v[32]-work.L[104]*work.v[169];
  work.v[171] = target[187]-work.L[105]*work.v[38]-work.L[106]*work.v[169]-work.L[107]*work.v[170];
  work.v[172] = target[67];
  work.v[173] = target[68];
  work.v[174] = target[69];
  work.v[175] = target[70];
  work.v[176] = target[71];
  work.v[177] = target[72];
  work.v[178] = target[18];
  work.v[179] = target[163]-work.L[108]*work.v[14]-work.L[109]*work.v[148]-work.L[110]*work.v[178];
  work.v[180] = target[169]-work.L[111]*work.v[20]-work.L[112]*work.v[148]-work.L[113]*work.v[178]-work.L[114]*work.v[179];
  work.v[181] = target[12]-work.L[115]*work.v[131]-work.L[116]*work.v[132]-work.L[117]*work.v[179]-work.L[118]*work.v[180];
  work.v[182] = target[225]-work.L[119]*work.v[101]-work.L[120]*work.v[118]-work.L[121]*work.v[124]-work.L[122]*work.v[178]-work.L[123]*work.v[179]-work.L[124]*work.v[180]-work.L[125]*work.v[181];
  work.v[183] = target[231]-work.L[126]*work.v[118]-work.L[127]*work.v[178]-work.L[128]*work.v[179]-work.L[129]*work.v[180]-work.L[130]*work.v[181]-work.L[131]*work.v[182];
  work.v[184] = target[19];
  work.v[185] = target[164]-work.L[132]*work.v[15]-work.L[133]*work.v[149]-work.L[134]*work.v[184];
  work.v[186] = target[170]-work.L[135]*work.v[21]-work.L[136]*work.v[149]-work.L[137]*work.v[184]-work.L[138]*work.v[185];
  work.v[187] = target[13]-work.L[139]*work.v[134]-work.L[140]*work.v[135]-work.L[141]*work.v[185]-work.L[142]*work.v[186];
  work.v[188] = target[226]-work.L[143]*work.v[103]-work.L[144]*work.v[119]-work.L[145]*work.v[125]-work.L[146]*work.v[184]-work.L[147]*work.v[185]-work.L[148]*work.v[186]-work.L[149]*work.v[187];
  work.v[189] = target[232]-work.L[150]*work.v[119]-work.L[151]*work.v[184]-work.L[152]*work.v[185]-work.L[153]*work.v[186]-work.L[154]*work.v[187]-work.L[155]*work.v[188];
  work.v[190] = target[20];
  work.v[191] = target[165]-work.L[156]*work.v[16]-work.L[157]*work.v[150]-work.L[158]*work.v[190];
  work.v[192] = target[171]-work.L[159]*work.v[22]-work.L[160]*work.v[150]-work.L[161]*work.v[190]-work.L[162]*work.v[191];
  work.v[193] = target[14]-work.L[163]*work.v[137]-work.L[164]*work.v[138]-work.L[165]*work.v[191]-work.L[166]*work.v[192];
  work.v[194] = target[227]-work.L[167]*work.v[105]-work.L[168]*work.v[120]-work.L[169]*work.v[126]-work.L[170]*work.v[190]-work.L[171]*work.v[191]-work.L[172]*work.v[192]-work.L[173]*work.v[193];
  work.v[195] = target[233]-work.L[174]*work.v[120]-work.L[175]*work.v[190]-work.L[176]*work.v[191]-work.L[177]*work.v[192]-work.L[178]*work.v[193]-work.L[179]*work.v[194];
  work.v[196] = target[21];
  work.v[197] = target[166]-work.L[180]*work.v[17]-work.L[181]*work.v[151]-work.L[182]*work.v[196];
  work.v[198] = target[172]-work.L[183]*work.v[23]-work.L[184]*work.v[151]-work.L[185]*work.v[196]-work.L[186]*work.v[197];
  work.v[199] = target[15]-work.L[187]*work.v[140]-work.L[188]*work.v[141]-work.L[189]*work.v[197]-work.L[190]*work.v[198];
  work.v[200] = target[228]-work.L[191]*work.v[107]-work.L[192]*work.v[121]-work.L[193]*work.v[127]-work.L[194]*work.v[196]-work.L[195]*work.v[197]-work.L[196]*work.v[198]-work.L[197]*work.v[199];
  work.v[201] = target[234]-work.L[198]*work.v[121]-work.L[199]*work.v[196]-work.L[200]*work.v[197]-work.L[201]*work.v[198]-work.L[202]*work.v[199]-work.L[203]*work.v[200];
  work.v[202] = target[22];
  work.v[203] = target[167]-work.L[204]*work.v[18]-work.L[205]*work.v[152]-work.L[206]*work.v[202];
  work.v[204] = target[173]-work.L[207]*work.v[24]-work.L[208]*work.v[152]-work.L[209]*work.v[202]-work.L[210]*work.v[203];
  work.v[205] = target[16]-work.L[211]*work.v[143]-work.L[212]*work.v[144]-work.L[213]*work.v[203]-work.L[214]*work.v[204];
  work.v[206] = target[229]-work.L[215]*work.v[109]-work.L[216]*work.v[122]-work.L[217]*work.v[128]-work.L[218]*work.v[202]-work.L[219]*work.v[203]-work.L[220]*work.v[204]-work.L[221]*work.v[205];
  work.v[207] = target[235]-work.L[222]*work.v[122]-work.L[223]*work.v[202]-work.L[224]*work.v[203]-work.L[225]*work.v[204]-work.L[226]*work.v[205]-work.L[227]*work.v[206];
  work.v[208] = target[23];
  work.v[209] = target[168]-work.L[228]*work.v[19]-work.L[229]*work.v[153]-work.L[230]*work.v[208];
  work.v[210] = target[174]-work.L[231]*work.v[25]-work.L[232]*work.v[153]-work.L[233]*work.v[208]-work.L[234]*work.v[209];
  work.v[211] = target[17]-work.L[235]*work.v[146]-work.L[236]*work.v[147]-work.L[237]*work.v[209]-work.L[238]*work.v[210];
  work.v[212] = target[230]-work.L[239]*work.v[111]-work.L[240]*work.v[123]-work.L[241]*work.v[129]-work.L[242]*work.v[208]-work.L[243]*work.v[209]-work.L[244]*work.v[210]-work.L[245]*work.v[211];
  work.v[213] = target[236]-work.L[246]*work.v[123]-work.L[247]*work.v[208]-work.L[248]*work.v[209]-work.L[249]*work.v[210]-work.L[250]*work.v[211]-work.L[251]*work.v[212];
  work.v[214] = target[30]-work.L[252]*work.v[88]-work.L[253]*work.v[89]-work.L[254]*work.v[183];
  work.v[215] = target[31]-work.L[255]*work.v[90]-work.L[256]*work.v[91]-work.L[257]*work.v[189];
  work.v[216] = target[32]-work.L[258]*work.v[92]-work.L[259]*work.v[93]-work.L[260]*work.v[195];
  work.v[217] = target[33]-work.L[261]*work.v[94]-work.L[262]*work.v[95]-work.L[263]*work.v[201];
  work.v[218] = target[34]-work.L[264]*work.v[96]-work.L[265]*work.v[97]-work.L[266]*work.v[207];
  work.v[219] = target[35]-work.L[267]*work.v[98]-work.L[268]*work.v[99]-work.L[269]*work.v[213];
  work.v[220] = target[42];
  work.v[221] = target[189]-work.L[270]*work.v[40]-work.L[271]*work.v[172]-work.L[272]*work.v[220];
  work.v[222] = target[195]-work.L[273]*work.v[46]-work.L[274]*work.v[172]-work.L[275]*work.v[220]-work.L[276]*work.v[221];
  work.v[223] = target[36]-work.L[277]*work.v[155]-work.L[278]*work.v[156]-work.L[279]*work.v[221]-work.L[280]*work.v[222];
  work.v[224] = target[43];
  work.v[225] = target[190]-work.L[281]*work.v[41]-work.L[282]*work.v[173]-work.L[283]*work.v[224];
  work.v[226] = target[196]-work.L[284]*work.v[47]-work.L[285]*work.v[173]-work.L[286]*work.v[224]-work.L[287]*work.v[225];
  work.v[227] = target[37]-work.L[288]*work.v[158]-work.L[289]*work.v[159]-work.L[290]*work.v[225]-work.L[291]*work.v[226];
  work.v[228] = target[44];
  work.v[229] = target[191]-work.L[292]*work.v[42]-work.L[293]*work.v[174]-work.L[294]*work.v[228];
  work.v[230] = target[197]-work.L[295]*work.v[48]-work.L[296]*work.v[174]-work.L[297]*work.v[228]-work.L[298]*work.v[229];
  work.v[231] = target[38]-work.L[299]*work.v[161]-work.L[300]*work.v[162]-work.L[301]*work.v[229]-work.L[302]*work.v[230];
  work.v[232] = target[45];
  work.v[233] = target[192]-work.L[303]*work.v[43]-work.L[304]*work.v[175]-work.L[305]*work.v[232];
  work.v[234] = target[198]-work.L[306]*work.v[49]-work.L[307]*work.v[175]-work.L[308]*work.v[232]-work.L[309]*work.v[233];
  work.v[235] = target[39]-work.L[310]*work.v[164]-work.L[311]*work.v[165]-work.L[312]*work.v[233]-work.L[313]*work.v[234];
  work.v[236] = target[46];
  work.v[237] = target[193]-work.L[314]*work.v[44]-work.L[315]*work.v[176]-work.L[316]*work.v[236];
  work.v[238] = target[199]-work.L[317]*work.v[50]-work.L[318]*work.v[176]-work.L[319]*work.v[236]-work.L[320]*work.v[237];
  work.v[239] = target[40]-work.L[321]*work.v[167]-work.L[322]*work.v[168]-work.L[323]*work.v[237]-work.L[324]*work.v[238];
  work.v[240] = target[47];
  work.v[241] = target[194]-work.L[325]*work.v[45]-work.L[326]*work.v[177]-work.L[327]*work.v[240];
  work.v[242] = target[200]-work.L[328]*work.v[51]-work.L[329]*work.v[177]-work.L[330]*work.v[240]-work.L[331]*work.v[241];
  work.v[243] = target[41]-work.L[332]*work.v[170]-work.L[333]*work.v[171]-work.L[334]*work.v[241]-work.L[335]*work.v[242];
  work.v[244] = target[48];
  work.v[245] = target[249]-work.L[336]*work.v[124]-work.L[337]*work.v[182]-work.L[338]*work.v[183]-work.L[339]*work.v[214]-work.L[340]*work.v[220]-work.L[341]*work.v[221]-work.L[342]*work.v[222]-work.L[343]*work.v[223];
  work.v[246] = target[250]-work.L[344]*work.v[125]-work.L[345]*work.v[188]-work.L[346]*work.v[189]-work.L[347]*work.v[215]-work.L[348]*work.v[224]-work.L[349]*work.v[225]-work.L[350]*work.v[226]-work.L[351]*work.v[227];
  work.v[247] = target[251]-work.L[352]*work.v[126]-work.L[353]*work.v[194]-work.L[354]*work.v[195]-work.L[355]*work.v[216]-work.L[356]*work.v[228]-work.L[357]*work.v[229]-work.L[358]*work.v[230]-work.L[359]*work.v[231];
  work.v[248] = target[252]-work.L[360]*work.v[127]-work.L[361]*work.v[200]-work.L[362]*work.v[201]-work.L[363]*work.v[217]-work.L[364]*work.v[232]-work.L[365]*work.v[233]-work.L[366]*work.v[234]-work.L[367]*work.v[235];
  work.v[249] = target[253]-work.L[368]*work.v[128]-work.L[369]*work.v[206]-work.L[370]*work.v[207]-work.L[371]*work.v[218]-work.L[372]*work.v[236]-work.L[373]*work.v[237]-work.L[374]*work.v[238]-work.L[375]*work.v[239];
  work.v[250] = target[254]-work.L[376]*work.v[129]-work.L[377]*work.v[212]-work.L[378]*work.v[213]-work.L[379]*work.v[219]-work.L[380]*work.v[240]-work.L[381]*work.v[241]-work.L[382]*work.v[242]-work.L[383]*work.v[243];
  work.v[251] = target[255]-work.L[384]*work.v[214]-work.L[385]*work.v[220]-work.L[386]*work.v[221]-work.L[387]*work.v[222]-work.L[388]*work.v[223]-work.L[389]*work.v[245];
  work.v[252] = target[256]-work.L[390]*work.v[215]-work.L[391]*work.v[224]-work.L[392]*work.v[225]-work.L[393]*work.v[226]-work.L[394]*work.v[227]-work.L[395]*work.v[246];
  work.v[253] = target[257]-work.L[396]*work.v[216]-work.L[397]*work.v[228]-work.L[398]*work.v[229]-work.L[399]*work.v[230]-work.L[400]*work.v[231]-work.L[401]*work.v[247];
  work.v[254] = target[258]-work.L[402]*work.v[217]-work.L[403]*work.v[232]-work.L[404]*work.v[233]-work.L[405]*work.v[234]-work.L[406]*work.v[235]-work.L[407]*work.v[248];
  work.v[255] = target[259]-work.L[408]*work.v[218]-work.L[409]*work.v[236]-work.L[410]*work.v[237]-work.L[411]*work.v[238]-work.L[412]*work.v[239]-work.L[413]*work.v[249];
  work.v[256] = target[149]-work.L[414]*work.v[0]-work.L[415]*work.v[130]-work.L[416]*work.v[131]-work.L[417]*work.v[132]-work.L[418]*work.v[133]-work.L[419]*work.v[134]-work.L[420]*work.v[135]-work.L[421]*work.v[136]-work.L[422]*work.v[137]-work.L[423]*work.v[138]-work.L[424]*work.v[139]-work.L[425]*work.v[140]-work.L[426]*work.v[141]-work.L[427]*work.v[142]-work.L[428]*work.v[143]-work.L[429]*work.v[144]-work.L[430]*work.v[145]-work.L[431]*work.v[146]-work.L[432]*work.v[147]-work.L[433]*work.v[181]-work.L[434]*work.v[182]-work.L[435]*work.v[183]-work.L[436]*work.v[187]-work.L[437]*work.v[188]-work.L[438]*work.v[189]-work.L[439]*work.v[193]-work.L[440]*work.v[194]-work.L[441]*work.v[195]-work.L[442]*work.v[199]-work.L[443]*work.v[200]-work.L[444]*work.v[201]-work.L[445]*work.v[205]-work.L[446]*work.v[206]-work.L[447]*work.v[207]-work.L[448]*work.v[211]-work.L[449]*work.v[212]-work.L[450]*work.v[213]-work.L[451]*work.v[214]-work.L[452]*work.v[215]-work.L[453]*work.v[216]-work.L[454]*work.v[217]-work.L[455]*work.v[218]-work.L[456]*work.v[219]-work.L[457]*work.v[244]-work.L[458]*work.v[245]-work.L[459]*work.v[246]-work.L[460]*work.v[247]-work.L[461]*work.v[248]-work.L[462]*work.v[249]-work.L[463]*work.v[250]-work.L[464]*work.v[251]-work.L[465]*work.v[252]-work.L[466]*work.v[253]-work.L[467]*work.v[254]-work.L[468]*work.v[255];
  work.v[257] = target[162]-work.L[469]*work.v[13]-work.L[470]*work.v[148]-work.L[471]*work.v[149]-work.L[472]*work.v[150]-work.L[473]*work.v[151]-work.L[474]*work.v[152]-work.L[475]*work.v[153]-work.L[476]*work.v[179]-work.L[477]*work.v[180]-work.L[478]*work.v[181]-work.L[479]*work.v[182]-work.L[480]*work.v[183]-work.L[481]*work.v[185]-work.L[482]*work.v[186]-work.L[483]*work.v[187]-work.L[484]*work.v[188]-work.L[485]*work.v[189]-work.L[486]*work.v[191]-work.L[487]*work.v[192]-work.L[488]*work.v[193]-work.L[489]*work.v[194]-work.L[490]*work.v[195]-work.L[491]*work.v[197]-work.L[492]*work.v[198]-work.L[493]*work.v[199]-work.L[494]*work.v[200]-work.L[495]*work.v[201]-work.L[496]*work.v[203]-work.L[497]*work.v[204]-work.L[498]*work.v[205]-work.L[499]*work.v[206]-work.L[500]*work.v[207]-work.L[501]*work.v[209]-work.L[502]*work.v[210]-work.L[503]*work.v[211]-work.L[504]*work.v[212]-work.L[505]*work.v[213]-work.L[506]*work.v[214]-work.L[507]*work.v[215]-work.L[508]*work.v[216]-work.L[509]*work.v[217]-work.L[510]*work.v[218]-work.L[511]*work.v[219]-work.L[512]*work.v[244]-work.L[513]*work.v[245]-work.L[514]*work.v[246]-work.L[515]*work.v[247]-work.L[516]*work.v[248]-work.L[517]*work.v[249]-work.L[518]*work.v[250]-work.L[519]*work.v[251]-work.L[520]*work.v[252]-work.L[521]*work.v[253]-work.L[522]*work.v[254]-work.L[523]*work.v[255]-work.L[524]*work.v[256];
  work.v[258] = target[175]-work.L[525]*work.v[26]-work.L[526]*work.v[154]-work.L[527]*work.v[155]-work.L[528]*work.v[156]-work.L[529]*work.v[157]-work.L[530]*work.v[158]-work.L[531]*work.v[159]-work.L[532]*work.v[160]-work.L[533]*work.v[161]-work.L[534]*work.v[162]-work.L[535]*work.v[163]-work.L[536]*work.v[164]-work.L[537]*work.v[165]-work.L[538]*work.v[166]-work.L[539]*work.v[167]-work.L[540]*work.v[168]-work.L[541]*work.v[169]-work.L[542]*work.v[170]-work.L[543]*work.v[171]-work.L[544]*work.v[223]-work.L[545]*work.v[227]-work.L[546]*work.v[231]-work.L[547]*work.v[235]-work.L[548]*work.v[239]-work.L[549]*work.v[243]-work.L[550]*work.v[244]-work.L[551]*work.v[245]-work.L[552]*work.v[246]-work.L[553]*work.v[247]-work.L[554]*work.v[248]-work.L[555]*work.v[249]-work.L[556]*work.v[250]-work.L[557]*work.v[251]-work.L[558]*work.v[252]-work.L[559]*work.v[253]-work.L[560]*work.v[254]-work.L[561]*work.v[255]-work.L[562]*work.v[256]-work.L[563]*work.v[257];
  work.v[259] = target[188]-work.L[564]*work.v[39]-work.L[565]*work.v[172]-work.L[566]*work.v[173]-work.L[567]*work.v[174]-work.L[568]*work.v[175]-work.L[569]*work.v[176]-work.L[570]*work.v[177]-work.L[571]*work.v[221]-work.L[572]*work.v[222]-work.L[573]*work.v[223]-work.L[574]*work.v[225]-work.L[575]*work.v[226]-work.L[576]*work.v[227]-work.L[577]*work.v[229]-work.L[578]*work.v[230]-work.L[579]*work.v[231]-work.L[580]*work.v[233]-work.L[581]*work.v[234]-work.L[582]*work.v[235]-work.L[583]*work.v[237]-work.L[584]*work.v[238]-work.L[585]*work.v[239]-work.L[586]*work.v[241]-work.L[587]*work.v[242]-work.L[588]*work.v[243]-work.L[589]*work.v[244]-work.L[590]*work.v[245]-work.L[591]*work.v[246]-work.L[592]*work.v[247]-work.L[593]*work.v[248]-work.L[594]*work.v[249]-work.L[595]*work.v[250]-work.L[596]*work.v[251]-work.L[597]*work.v[252]-work.L[598]*work.v[253]-work.L[599]*work.v[254]-work.L[600]*work.v[255]-work.L[601]*work.v[256]-work.L[602]*work.v[257]-work.L[603]*work.v[258];
  work.v[260] = target[260]-work.L[604]*work.v[219]-work.L[605]*work.v[240]-work.L[606]*work.v[241]-work.L[607]*work.v[242]-work.L[608]*work.v[243]-work.L[609]*work.v[250]-work.L[610]*work.v[256]-work.L[611]*work.v[257]-work.L[612]*work.v[258]-work.L[613]*work.v[259];
  /* Diagonal scaling. Assume correctness of work.d_inv. */
  for (i = 0; i < 261; i++)
    work.v[i] *= work.d_inv[i];
  /* Back substitution */
  work.v[259] -= work.L[613]*work.v[260];
  work.v[258] -= work.L[603]*work.v[259]+work.L[612]*work.v[260];
  work.v[257] -= work.L[563]*work.v[258]+work.L[602]*work.v[259]+work.L[611]*work.v[260];
  work.v[256] -= work.L[524]*work.v[257]+work.L[562]*work.v[258]+work.L[601]*work.v[259]+work.L[610]*work.v[260];
  work.v[255] -= work.L[468]*work.v[256]+work.L[523]*work.v[257]+work.L[561]*work.v[258]+work.L[600]*work.v[259];
  work.v[254] -= work.L[467]*work.v[256]+work.L[522]*work.v[257]+work.L[560]*work.v[258]+work.L[599]*work.v[259];
  work.v[253] -= work.L[466]*work.v[256]+work.L[521]*work.v[257]+work.L[559]*work.v[258]+work.L[598]*work.v[259];
  work.v[252] -= work.L[465]*work.v[256]+work.L[520]*work.v[257]+work.L[558]*work.v[258]+work.L[597]*work.v[259];
  work.v[251] -= work.L[464]*work.v[256]+work.L[519]*work.v[257]+work.L[557]*work.v[258]+work.L[596]*work.v[259];
  work.v[250] -= work.L[463]*work.v[256]+work.L[518]*work.v[257]+work.L[556]*work.v[258]+work.L[595]*work.v[259]+work.L[609]*work.v[260];
  work.v[249] -= work.L[413]*work.v[255]+work.L[462]*work.v[256]+work.L[517]*work.v[257]+work.L[555]*work.v[258]+work.L[594]*work.v[259];
  work.v[248] -= work.L[407]*work.v[254]+work.L[461]*work.v[256]+work.L[516]*work.v[257]+work.L[554]*work.v[258]+work.L[593]*work.v[259];
  work.v[247] -= work.L[401]*work.v[253]+work.L[460]*work.v[256]+work.L[515]*work.v[257]+work.L[553]*work.v[258]+work.L[592]*work.v[259];
  work.v[246] -= work.L[395]*work.v[252]+work.L[459]*work.v[256]+work.L[514]*work.v[257]+work.L[552]*work.v[258]+work.L[591]*work.v[259];
  work.v[245] -= work.L[389]*work.v[251]+work.L[458]*work.v[256]+work.L[513]*work.v[257]+work.L[551]*work.v[258]+work.L[590]*work.v[259];
  work.v[244] -= work.L[457]*work.v[256]+work.L[512]*work.v[257]+work.L[550]*work.v[258]+work.L[589]*work.v[259];
  work.v[243] -= work.L[383]*work.v[250]+work.L[549]*work.v[258]+work.L[588]*work.v[259]+work.L[608]*work.v[260];
  work.v[242] -= work.L[335]*work.v[243]+work.L[382]*work.v[250]+work.L[587]*work.v[259]+work.L[607]*work.v[260];
  work.v[241] -= work.L[331]*work.v[242]+work.L[334]*work.v[243]+work.L[381]*work.v[250]+work.L[586]*work.v[259]+work.L[606]*work.v[260];
  work.v[240] -= work.L[327]*work.v[241]+work.L[330]*work.v[242]+work.L[380]*work.v[250]+work.L[605]*work.v[260];
  work.v[239] -= work.L[375]*work.v[249]+work.L[412]*work.v[255]+work.L[548]*work.v[258]+work.L[585]*work.v[259];
  work.v[238] -= work.L[324]*work.v[239]+work.L[374]*work.v[249]+work.L[411]*work.v[255]+work.L[584]*work.v[259];
  work.v[237] -= work.L[320]*work.v[238]+work.L[323]*work.v[239]+work.L[373]*work.v[249]+work.L[410]*work.v[255]+work.L[583]*work.v[259];
  work.v[236] -= work.L[316]*work.v[237]+work.L[319]*work.v[238]+work.L[372]*work.v[249]+work.L[409]*work.v[255];
  work.v[235] -= work.L[367]*work.v[248]+work.L[406]*work.v[254]+work.L[547]*work.v[258]+work.L[582]*work.v[259];
  work.v[234] -= work.L[313]*work.v[235]+work.L[366]*work.v[248]+work.L[405]*work.v[254]+work.L[581]*work.v[259];
  work.v[233] -= work.L[309]*work.v[234]+work.L[312]*work.v[235]+work.L[365]*work.v[248]+work.L[404]*work.v[254]+work.L[580]*work.v[259];
  work.v[232] -= work.L[305]*work.v[233]+work.L[308]*work.v[234]+work.L[364]*work.v[248]+work.L[403]*work.v[254];
  work.v[231] -= work.L[359]*work.v[247]+work.L[400]*work.v[253]+work.L[546]*work.v[258]+work.L[579]*work.v[259];
  work.v[230] -= work.L[302]*work.v[231]+work.L[358]*work.v[247]+work.L[399]*work.v[253]+work.L[578]*work.v[259];
  work.v[229] -= work.L[298]*work.v[230]+work.L[301]*work.v[231]+work.L[357]*work.v[247]+work.L[398]*work.v[253]+work.L[577]*work.v[259];
  work.v[228] -= work.L[294]*work.v[229]+work.L[297]*work.v[230]+work.L[356]*work.v[247]+work.L[397]*work.v[253];
  work.v[227] -= work.L[351]*work.v[246]+work.L[394]*work.v[252]+work.L[545]*work.v[258]+work.L[576]*work.v[259];
  work.v[226] -= work.L[291]*work.v[227]+work.L[350]*work.v[246]+work.L[393]*work.v[252]+work.L[575]*work.v[259];
  work.v[225] -= work.L[287]*work.v[226]+work.L[290]*work.v[227]+work.L[349]*work.v[246]+work.L[392]*work.v[252]+work.L[574]*work.v[259];
  work.v[224] -= work.L[283]*work.v[225]+work.L[286]*work.v[226]+work.L[348]*work.v[246]+work.L[391]*work.v[252];
  work.v[223] -= work.L[343]*work.v[245]+work.L[388]*work.v[251]+work.L[544]*work.v[258]+work.L[573]*work.v[259];
  work.v[222] -= work.L[280]*work.v[223]+work.L[342]*work.v[245]+work.L[387]*work.v[251]+work.L[572]*work.v[259];
  work.v[221] -= work.L[276]*work.v[222]+work.L[279]*work.v[223]+work.L[341]*work.v[245]+work.L[386]*work.v[251]+work.L[571]*work.v[259];
  work.v[220] -= work.L[272]*work.v[221]+work.L[275]*work.v[222]+work.L[340]*work.v[245]+work.L[385]*work.v[251];
  work.v[219] -= work.L[379]*work.v[250]+work.L[456]*work.v[256]+work.L[511]*work.v[257]+work.L[604]*work.v[260];
  work.v[218] -= work.L[371]*work.v[249]+work.L[408]*work.v[255]+work.L[455]*work.v[256]+work.L[510]*work.v[257];
  work.v[217] -= work.L[363]*work.v[248]+work.L[402]*work.v[254]+work.L[454]*work.v[256]+work.L[509]*work.v[257];
  work.v[216] -= work.L[355]*work.v[247]+work.L[396]*work.v[253]+work.L[453]*work.v[256]+work.L[508]*work.v[257];
  work.v[215] -= work.L[347]*work.v[246]+work.L[390]*work.v[252]+work.L[452]*work.v[256]+work.L[507]*work.v[257];
  work.v[214] -= work.L[339]*work.v[245]+work.L[384]*work.v[251]+work.L[451]*work.v[256]+work.L[506]*work.v[257];
  work.v[213] -= work.L[269]*work.v[219]+work.L[378]*work.v[250]+work.L[450]*work.v[256]+work.L[505]*work.v[257];
  work.v[212] -= work.L[251]*work.v[213]+work.L[377]*work.v[250]+work.L[449]*work.v[256]+work.L[504]*work.v[257];
  work.v[211] -= work.L[245]*work.v[212]+work.L[250]*work.v[213]+work.L[448]*work.v[256]+work.L[503]*work.v[257];
  work.v[210] -= work.L[238]*work.v[211]+work.L[244]*work.v[212]+work.L[249]*work.v[213]+work.L[502]*work.v[257];
  work.v[209] -= work.L[234]*work.v[210]+work.L[237]*work.v[211]+work.L[243]*work.v[212]+work.L[248]*work.v[213]+work.L[501]*work.v[257];
  work.v[208] -= work.L[230]*work.v[209]+work.L[233]*work.v[210]+work.L[242]*work.v[212]+work.L[247]*work.v[213];
  work.v[207] -= work.L[266]*work.v[218]+work.L[370]*work.v[249]+work.L[447]*work.v[256]+work.L[500]*work.v[257];
  work.v[206] -= work.L[227]*work.v[207]+work.L[369]*work.v[249]+work.L[446]*work.v[256]+work.L[499]*work.v[257];
  work.v[205] -= work.L[221]*work.v[206]+work.L[226]*work.v[207]+work.L[445]*work.v[256]+work.L[498]*work.v[257];
  work.v[204] -= work.L[214]*work.v[205]+work.L[220]*work.v[206]+work.L[225]*work.v[207]+work.L[497]*work.v[257];
  work.v[203] -= work.L[210]*work.v[204]+work.L[213]*work.v[205]+work.L[219]*work.v[206]+work.L[224]*work.v[207]+work.L[496]*work.v[257];
  work.v[202] -= work.L[206]*work.v[203]+work.L[209]*work.v[204]+work.L[218]*work.v[206]+work.L[223]*work.v[207];
  work.v[201] -= work.L[263]*work.v[217]+work.L[362]*work.v[248]+work.L[444]*work.v[256]+work.L[495]*work.v[257];
  work.v[200] -= work.L[203]*work.v[201]+work.L[361]*work.v[248]+work.L[443]*work.v[256]+work.L[494]*work.v[257];
  work.v[199] -= work.L[197]*work.v[200]+work.L[202]*work.v[201]+work.L[442]*work.v[256]+work.L[493]*work.v[257];
  work.v[198] -= work.L[190]*work.v[199]+work.L[196]*work.v[200]+work.L[201]*work.v[201]+work.L[492]*work.v[257];
  work.v[197] -= work.L[186]*work.v[198]+work.L[189]*work.v[199]+work.L[195]*work.v[200]+work.L[200]*work.v[201]+work.L[491]*work.v[257];
  work.v[196] -= work.L[182]*work.v[197]+work.L[185]*work.v[198]+work.L[194]*work.v[200]+work.L[199]*work.v[201];
  work.v[195] -= work.L[260]*work.v[216]+work.L[354]*work.v[247]+work.L[441]*work.v[256]+work.L[490]*work.v[257];
  work.v[194] -= work.L[179]*work.v[195]+work.L[353]*work.v[247]+work.L[440]*work.v[256]+work.L[489]*work.v[257];
  work.v[193] -= work.L[173]*work.v[194]+work.L[178]*work.v[195]+work.L[439]*work.v[256]+work.L[488]*work.v[257];
  work.v[192] -= work.L[166]*work.v[193]+work.L[172]*work.v[194]+work.L[177]*work.v[195]+work.L[487]*work.v[257];
  work.v[191] -= work.L[162]*work.v[192]+work.L[165]*work.v[193]+work.L[171]*work.v[194]+work.L[176]*work.v[195]+work.L[486]*work.v[257];
  work.v[190] -= work.L[158]*work.v[191]+work.L[161]*work.v[192]+work.L[170]*work.v[194]+work.L[175]*work.v[195];
  work.v[189] -= work.L[257]*work.v[215]+work.L[346]*work.v[246]+work.L[438]*work.v[256]+work.L[485]*work.v[257];
  work.v[188] -= work.L[155]*work.v[189]+work.L[345]*work.v[246]+work.L[437]*work.v[256]+work.L[484]*work.v[257];
  work.v[187] -= work.L[149]*work.v[188]+work.L[154]*work.v[189]+work.L[436]*work.v[256]+work.L[483]*work.v[257];
  work.v[186] -= work.L[142]*work.v[187]+work.L[148]*work.v[188]+work.L[153]*work.v[189]+work.L[482]*work.v[257];
  work.v[185] -= work.L[138]*work.v[186]+work.L[141]*work.v[187]+work.L[147]*work.v[188]+work.L[152]*work.v[189]+work.L[481]*work.v[257];
  work.v[184] -= work.L[134]*work.v[185]+work.L[137]*work.v[186]+work.L[146]*work.v[188]+work.L[151]*work.v[189];
  work.v[183] -= work.L[254]*work.v[214]+work.L[338]*work.v[245]+work.L[435]*work.v[256]+work.L[480]*work.v[257];
  work.v[182] -= work.L[131]*work.v[183]+work.L[337]*work.v[245]+work.L[434]*work.v[256]+work.L[479]*work.v[257];
  work.v[181] -= work.L[125]*work.v[182]+work.L[130]*work.v[183]+work.L[433]*work.v[256]+work.L[478]*work.v[257];
  work.v[180] -= work.L[118]*work.v[181]+work.L[124]*work.v[182]+work.L[129]*work.v[183]+work.L[477]*work.v[257];
  work.v[179] -= work.L[114]*work.v[180]+work.L[117]*work.v[181]+work.L[123]*work.v[182]+work.L[128]*work.v[183]+work.L[476]*work.v[257];
  work.v[178] -= work.L[110]*work.v[179]+work.L[113]*work.v[180]+work.L[122]*work.v[182]+work.L[127]*work.v[183];
  work.v[177] -= work.L[326]*work.v[241]+work.L[329]*work.v[242]+work.L[570]*work.v[259];
  work.v[176] -= work.L[315]*work.v[237]+work.L[318]*work.v[238]+work.L[569]*work.v[259];
  work.v[175] -= work.L[304]*work.v[233]+work.L[307]*work.v[234]+work.L[568]*work.v[259];
  work.v[174] -= work.L[293]*work.v[229]+work.L[296]*work.v[230]+work.L[567]*work.v[259];
  work.v[173] -= work.L[282]*work.v[225]+work.L[285]*work.v[226]+work.L[566]*work.v[259];
  work.v[172] -= work.L[271]*work.v[221]+work.L[274]*work.v[222]+work.L[565]*work.v[259];
  work.v[171] -= work.L[333]*work.v[243]+work.L[543]*work.v[258];
  work.v[170] -= work.L[107]*work.v[171]+work.L[332]*work.v[243]+work.L[542]*work.v[258];
  work.v[169] -= work.L[104]*work.v[170]+work.L[106]*work.v[171]+work.L[541]*work.v[258];
  work.v[168] -= work.L[322]*work.v[239]+work.L[540]*work.v[258];
  work.v[167] -= work.L[102]*work.v[168]+work.L[321]*work.v[239]+work.L[539]*work.v[258];
  work.v[166] -= work.L[99]*work.v[167]+work.L[101]*work.v[168]+work.L[538]*work.v[258];
  work.v[165] -= work.L[311]*work.v[235]+work.L[537]*work.v[258];
  work.v[164] -= work.L[97]*work.v[165]+work.L[310]*work.v[235]+work.L[536]*work.v[258];
  work.v[163] -= work.L[94]*work.v[164]+work.L[96]*work.v[165]+work.L[535]*work.v[258];
  work.v[162] -= work.L[300]*work.v[231]+work.L[534]*work.v[258];
  work.v[161] -= work.L[92]*work.v[162]+work.L[299]*work.v[231]+work.L[533]*work.v[258];
  work.v[160] -= work.L[89]*work.v[161]+work.L[91]*work.v[162]+work.L[532]*work.v[258];
  work.v[159] -= work.L[289]*work.v[227]+work.L[531]*work.v[258];
  work.v[158] -= work.L[87]*work.v[159]+work.L[288]*work.v[227]+work.L[530]*work.v[258];
  work.v[157] -= work.L[84]*work.v[158]+work.L[86]*work.v[159]+work.L[529]*work.v[258];
  work.v[156] -= work.L[278]*work.v[223]+work.L[528]*work.v[258];
  work.v[155] -= work.L[82]*work.v[156]+work.L[277]*work.v[223]+work.L[527]*work.v[258];
  work.v[154] -= work.L[79]*work.v[155]+work.L[81]*work.v[156]+work.L[526]*work.v[258];
  work.v[153] -= work.L[229]*work.v[209]+work.L[232]*work.v[210]+work.L[475]*work.v[257];
  work.v[152] -= work.L[205]*work.v[203]+work.L[208]*work.v[204]+work.L[474]*work.v[257];
  work.v[151] -= work.L[181]*work.v[197]+work.L[184]*work.v[198]+work.L[473]*work.v[257];
  work.v[150] -= work.L[157]*work.v[191]+work.L[160]*work.v[192]+work.L[472]*work.v[257];
  work.v[149] -= work.L[133]*work.v[185]+work.L[136]*work.v[186]+work.L[471]*work.v[257];
  work.v[148] -= work.L[109]*work.v[179]+work.L[112]*work.v[180]+work.L[470]*work.v[257];
  work.v[147] -= work.L[236]*work.v[211]+work.L[432]*work.v[256];
  work.v[146] -= work.L[77]*work.v[147]+work.L[235]*work.v[211]+work.L[431]*work.v[256];
  work.v[145] -= work.L[74]*work.v[146]+work.L[76]*work.v[147]+work.L[430]*work.v[256];
  work.v[144] -= work.L[212]*work.v[205]+work.L[429]*work.v[256];
  work.v[143] -= work.L[72]*work.v[144]+work.L[211]*work.v[205]+work.L[428]*work.v[256];
  work.v[142] -= work.L[69]*work.v[143]+work.L[71]*work.v[144]+work.L[427]*work.v[256];
  work.v[141] -= work.L[188]*work.v[199]+work.L[426]*work.v[256];
  work.v[140] -= work.L[67]*work.v[141]+work.L[187]*work.v[199]+work.L[425]*work.v[256];
  work.v[139] -= work.L[64]*work.v[140]+work.L[66]*work.v[141]+work.L[424]*work.v[256];
  work.v[138] -= work.L[164]*work.v[193]+work.L[423]*work.v[256];
  work.v[137] -= work.L[62]*work.v[138]+work.L[163]*work.v[193]+work.L[422]*work.v[256];
  work.v[136] -= work.L[59]*work.v[137]+work.L[61]*work.v[138]+work.L[421]*work.v[256];
  work.v[135] -= work.L[140]*work.v[187]+work.L[420]*work.v[256];
  work.v[134] -= work.L[57]*work.v[135]+work.L[139]*work.v[187]+work.L[419]*work.v[256];
  work.v[133] -= work.L[54]*work.v[134]+work.L[56]*work.v[135]+work.L[418]*work.v[256];
  work.v[132] -= work.L[116]*work.v[181]+work.L[417]*work.v[256];
  work.v[131] -= work.L[52]*work.v[132]+work.L[115]*work.v[181]+work.L[416]*work.v[256];
  work.v[130] -= work.L[49]*work.v[131]+work.L[51]*work.v[132]+work.L[415]*work.v[256];
  work.v[129] -= work.L[241]*work.v[212]+work.L[376]*work.v[250];
  work.v[128] -= work.L[217]*work.v[206]+work.L[368]*work.v[249];
  work.v[127] -= work.L[193]*work.v[200]+work.L[360]*work.v[248];
  work.v[126] -= work.L[169]*work.v[194]+work.L[352]*work.v[247];
  work.v[125] -= work.L[145]*work.v[188]+work.L[344]*work.v[246];
  work.v[124] -= work.L[121]*work.v[182]+work.L[336]*work.v[245];
  work.v[123] -= work.L[240]*work.v[212]+work.L[246]*work.v[213];
  work.v[122] -= work.L[216]*work.v[206]+work.L[222]*work.v[207];
  work.v[121] -= work.L[192]*work.v[200]+work.L[198]*work.v[201];
  work.v[120] -= work.L[168]*work.v[194]+work.L[174]*work.v[195];
  work.v[119] -= work.L[144]*work.v[188]+work.L[150]*work.v[189];
  work.v[118] -= work.L[120]*work.v[182]+work.L[126]*work.v[183];
  work.v[117] -= work.L[35]*work.v[123];
  work.v[116] -= work.L[34]*work.v[122];
  work.v[115] -= work.L[33]*work.v[121];
  work.v[114] -= work.L[32]*work.v[120];
  work.v[113] -= work.L[31]*work.v[119];
  work.v[112] -= work.L[30]*work.v[118];
  work.v[111] -= work.L[239]*work.v[212];
  work.v[110] -= work.L[29]*work.v[111];
  work.v[109] -= work.L[215]*work.v[206];
  work.v[108] -= work.L[28]*work.v[109];
  work.v[107] -= work.L[191]*work.v[200];
  work.v[106] -= work.L[27]*work.v[107];
  work.v[105] -= work.L[167]*work.v[194];
  work.v[104] -= work.L[26]*work.v[105];
  work.v[103] -= work.L[143]*work.v[188];
  work.v[102] -= work.L[25]*work.v[103];
  work.v[101] -= work.L[119]*work.v[182];
  work.v[100] -= work.L[24]*work.v[101];
  work.v[99] -= work.L[268]*work.v[219];
  work.v[98] -= work.L[267]*work.v[219];
  work.v[97] -= work.L[265]*work.v[218];
  work.v[96] -= work.L[264]*work.v[218];
  work.v[95] -= work.L[262]*work.v[217];
  work.v[94] -= work.L[261]*work.v[217];
  work.v[93] -= work.L[259]*work.v[216];
  work.v[92] -= work.L[258]*work.v[216];
  work.v[91] -= work.L[256]*work.v[215];
  work.v[90] -= work.L[255]*work.v[215];
  work.v[89] -= work.L[253]*work.v[214];
  work.v[88] -= work.L[252]*work.v[214];
  work.v[87] -= work.L[47]*work.v[129];
  work.v[86] -= work.L[46]*work.v[129];
  work.v[85] -= work.L[45]*work.v[128];
  work.v[84] -= work.L[44]*work.v[128];
  work.v[83] -= work.L[43]*work.v[127];
  work.v[82] -= work.L[42]*work.v[127];
  work.v[81] -= work.L[41]*work.v[126];
  work.v[80] -= work.L[40]*work.v[126];
  work.v[79] -= work.L[39]*work.v[125];
  work.v[78] -= work.L[38]*work.v[125];
  work.v[77] -= work.L[37]*work.v[124];
  work.v[76] -= work.L[36]*work.v[124];
  work.v[75] -= work.L[23]*work.v[99];
  work.v[74] -= work.L[21]*work.v[97];
  work.v[73] -= work.L[19]*work.v[95];
  work.v[72] -= work.L[17]*work.v[93];
  work.v[71] -= work.L[15]*work.v[91];
  work.v[70] -= work.L[13]*work.v[89];
  work.v[69] -= work.L[22]*work.v[98];
  work.v[68] -= work.L[20]*work.v[96];
  work.v[67] -= work.L[18]*work.v[94];
  work.v[66] -= work.L[16]*work.v[92];
  work.v[65] -= work.L[14]*work.v[90];
  work.v[64] -= work.L[12]*work.v[88];
  work.v[63] -= work.L[11]*work.v[87];
  work.v[62] -= work.L[9]*work.v[85];
  work.v[61] -= work.L[7]*work.v[83];
  work.v[60] -= work.L[5]*work.v[81];
  work.v[59] -= work.L[3]*work.v[79];
  work.v[58] -= work.L[1]*work.v[77];
  work.v[57] -= work.L[10]*work.v[86];
  work.v[56] -= work.L[8]*work.v[84];
  work.v[55] -= work.L[6]*work.v[82];
  work.v[54] -= work.L[4]*work.v[80];
  work.v[53] -= work.L[2]*work.v[78];
  work.v[52] -= work.L[0]*work.v[76];
  work.v[51] -= work.L[328]*work.v[242];
  work.v[50] -= work.L[317]*work.v[238];
  work.v[49] -= work.L[306]*work.v[234];
  work.v[48] -= work.L[295]*work.v[230];
  work.v[47] -= work.L[284]*work.v[226];
  work.v[46] -= work.L[273]*work.v[222];
  work.v[45] -= work.L[325]*work.v[241];
  work.v[44] -= work.L[314]*work.v[237];
  work.v[43] -= work.L[303]*work.v[233];
  work.v[42] -= work.L[292]*work.v[229];
  work.v[41] -= work.L[281]*work.v[225];
  work.v[40] -= work.L[270]*work.v[221];
  work.v[39] -= work.L[564]*work.v[259];
  work.v[38] -= work.L[105]*work.v[171];
  work.v[37] -= work.L[100]*work.v[168];
  work.v[36] -= work.L[95]*work.v[165];
  work.v[35] -= work.L[90]*work.v[162];
  work.v[34] -= work.L[85]*work.v[159];
  work.v[33] -= work.L[80]*work.v[156];
  work.v[32] -= work.L[103]*work.v[170];
  work.v[31] -= work.L[98]*work.v[167];
  work.v[30] -= work.L[93]*work.v[164];
  work.v[29] -= work.L[88]*work.v[161];
  work.v[28] -= work.L[83]*work.v[158];
  work.v[27] -= work.L[78]*work.v[155];
  work.v[26] -= work.L[525]*work.v[258];
  work.v[25] -= work.L[231]*work.v[210];
  work.v[24] -= work.L[207]*work.v[204];
  work.v[23] -= work.L[183]*work.v[198];
  work.v[22] -= work.L[159]*work.v[192];
  work.v[21] -= work.L[135]*work.v[186];
  work.v[20] -= work.L[111]*work.v[180];
  work.v[19] -= work.L[228]*work.v[209];
  work.v[18] -= work.L[204]*work.v[203];
  work.v[17] -= work.L[180]*work.v[197];
  work.v[16] -= work.L[156]*work.v[191];
  work.v[15] -= work.L[132]*work.v[185];
  work.v[14] -= work.L[108]*work.v[179];
  work.v[13] -= work.L[469]*work.v[257];
  work.v[12] -= work.L[75]*work.v[147];
  work.v[11] -= work.L[70]*work.v[144];
  work.v[10] -= work.L[65]*work.v[141];
  work.v[9] -= work.L[60]*work.v[138];
  work.v[8] -= work.L[55]*work.v[135];
  work.v[7] -= work.L[50]*work.v[132];
  work.v[6] -= work.L[73]*work.v[146];
  work.v[5] -= work.L[68]*work.v[143];
  work.v[4] -= work.L[63]*work.v[140];
  work.v[3] -= work.L[58]*work.v[137];
  work.v[2] -= work.L[53]*work.v[134];
  work.v[1] -= work.L[48]*work.v[131];
  work.v[0] -= work.L[414]*work.v[256];
  /* Unpermute the result, from v to var. */
  var[0] = work.v[101];
  var[1] = work.v[103];
  var[2] = work.v[105];
  var[3] = work.v[107];
  var[4] = work.v[109];
  var[5] = work.v[111];
  var[6] = work.v[118];
  var[7] = work.v[119];
  var[8] = work.v[120];
  var[9] = work.v[121];
  var[10] = work.v[122];
  var[11] = work.v[123];
  var[12] = work.v[181];
  var[13] = work.v[187];
  var[14] = work.v[193];
  var[15] = work.v[199];
  var[16] = work.v[205];
  var[17] = work.v[211];
  var[18] = work.v[178];
  var[19] = work.v[184];
  var[20] = work.v[190];
  var[21] = work.v[196];
  var[22] = work.v[202];
  var[23] = work.v[208];
  var[24] = work.v[124];
  var[25] = work.v[125];
  var[26] = work.v[126];
  var[27] = work.v[127];
  var[28] = work.v[128];
  var[29] = work.v[129];
  var[30] = work.v[214];
  var[31] = work.v[215];
  var[32] = work.v[216];
  var[33] = work.v[217];
  var[34] = work.v[218];
  var[35] = work.v[219];
  var[36] = work.v[223];
  var[37] = work.v[227];
  var[38] = work.v[231];
  var[39] = work.v[235];
  var[40] = work.v[239];
  var[41] = work.v[243];
  var[42] = work.v[220];
  var[43] = work.v[224];
  var[44] = work.v[228];
  var[45] = work.v[232];
  var[46] = work.v[236];
  var[47] = work.v[240];
  var[48] = work.v[244];
  var[49] = work.v[130];
  var[50] = work.v[133];
  var[51] = work.v[136];
  var[52] = work.v[139];
  var[53] = work.v[142];
  var[54] = work.v[145];
  var[55] = work.v[148];
  var[56] = work.v[149];
  var[57] = work.v[150];
  var[58] = work.v[151];
  var[59] = work.v[152];
  var[60] = work.v[153];
  var[61] = work.v[154];
  var[62] = work.v[157];
  var[63] = work.v[160];
  var[64] = work.v[163];
  var[65] = work.v[166];
  var[66] = work.v[169];
  var[67] = work.v[172];
  var[68] = work.v[173];
  var[69] = work.v[174];
  var[70] = work.v[175];
  var[71] = work.v[176];
  var[72] = work.v[177];
  var[73] = work.v[0];
  var[74] = work.v[1];
  var[75] = work.v[2];
  var[76] = work.v[3];
  var[77] = work.v[4];
  var[78] = work.v[5];
  var[79] = work.v[6];
  var[80] = work.v[7];
  var[81] = work.v[8];
  var[82] = work.v[9];
  var[83] = work.v[10];
  var[84] = work.v[11];
  var[85] = work.v[12];
  var[86] = work.v[13];
  var[87] = work.v[14];
  var[88] = work.v[15];
  var[89] = work.v[16];
  var[90] = work.v[17];
  var[91] = work.v[18];
  var[92] = work.v[19];
  var[93] = work.v[20];
  var[94] = work.v[21];
  var[95] = work.v[22];
  var[96] = work.v[23];
  var[97] = work.v[24];
  var[98] = work.v[25];
  var[99] = work.v[26];
  var[100] = work.v[27];
  var[101] = work.v[28];
  var[102] = work.v[29];
  var[103] = work.v[30];
  var[104] = work.v[31];
  var[105] = work.v[32];
  var[106] = work.v[33];
  var[107] = work.v[34];
  var[108] = work.v[35];
  var[109] = work.v[36];
  var[110] = work.v[37];
  var[111] = work.v[38];
  var[112] = work.v[39];
  var[113] = work.v[40];
  var[114] = work.v[41];
  var[115] = work.v[42];
  var[116] = work.v[43];
  var[117] = work.v[44];
  var[118] = work.v[45];
  var[119] = work.v[46];
  var[120] = work.v[47];
  var[121] = work.v[48];
  var[122] = work.v[49];
  var[123] = work.v[50];
  var[124] = work.v[51];
  var[125] = work.v[52];
  var[126] = work.v[53];
  var[127] = work.v[54];
  var[128] = work.v[55];
  var[129] = work.v[56];
  var[130] = work.v[57];
  var[131] = work.v[58];
  var[132] = work.v[59];
  var[133] = work.v[60];
  var[134] = work.v[61];
  var[135] = work.v[62];
  var[136] = work.v[63];
  var[137] = work.v[64];
  var[138] = work.v[65];
  var[139] = work.v[66];
  var[140] = work.v[67];
  var[141] = work.v[68];
  var[142] = work.v[69];
  var[143] = work.v[70];
  var[144] = work.v[71];
  var[145] = work.v[72];
  var[146] = work.v[73];
  var[147] = work.v[74];
  var[148] = work.v[75];
  var[149] = work.v[256];
  var[150] = work.v[131];
  var[151] = work.v[134];
  var[152] = work.v[137];
  var[153] = work.v[140];
  var[154] = work.v[143];
  var[155] = work.v[146];
  var[156] = work.v[132];
  var[157] = work.v[135];
  var[158] = work.v[138];
  var[159] = work.v[141];
  var[160] = work.v[144];
  var[161] = work.v[147];
  var[162] = work.v[257];
  var[163] = work.v[179];
  var[164] = work.v[185];
  var[165] = work.v[191];
  var[166] = work.v[197];
  var[167] = work.v[203];
  var[168] = work.v[209];
  var[169] = work.v[180];
  var[170] = work.v[186];
  var[171] = work.v[192];
  var[172] = work.v[198];
  var[173] = work.v[204];
  var[174] = work.v[210];
  var[175] = work.v[258];
  var[176] = work.v[155];
  var[177] = work.v[158];
  var[178] = work.v[161];
  var[179] = work.v[164];
  var[180] = work.v[167];
  var[181] = work.v[170];
  var[182] = work.v[156];
  var[183] = work.v[159];
  var[184] = work.v[162];
  var[185] = work.v[165];
  var[186] = work.v[168];
  var[187] = work.v[171];
  var[188] = work.v[259];
  var[189] = work.v[221];
  var[190] = work.v[225];
  var[191] = work.v[229];
  var[192] = work.v[233];
  var[193] = work.v[237];
  var[194] = work.v[241];
  var[195] = work.v[222];
  var[196] = work.v[226];
  var[197] = work.v[230];
  var[198] = work.v[234];
  var[199] = work.v[238];
  var[200] = work.v[242];
  var[201] = work.v[76];
  var[202] = work.v[78];
  var[203] = work.v[80];
  var[204] = work.v[82];
  var[205] = work.v[84];
  var[206] = work.v[86];
  var[207] = work.v[77];
  var[208] = work.v[79];
  var[209] = work.v[81];
  var[210] = work.v[83];
  var[211] = work.v[85];
  var[212] = work.v[87];
  var[213] = work.v[88];
  var[214] = work.v[90];
  var[215] = work.v[92];
  var[216] = work.v[94];
  var[217] = work.v[96];
  var[218] = work.v[98];
  var[219] = work.v[89];
  var[220] = work.v[91];
  var[221] = work.v[93];
  var[222] = work.v[95];
  var[223] = work.v[97];
  var[224] = work.v[99];
  var[225] = work.v[182];
  var[226] = work.v[188];
  var[227] = work.v[194];
  var[228] = work.v[200];
  var[229] = work.v[206];
  var[230] = work.v[212];
  var[231] = work.v[183];
  var[232] = work.v[189];
  var[233] = work.v[195];
  var[234] = work.v[201];
  var[235] = work.v[207];
  var[236] = work.v[213];
  var[237] = work.v[100];
  var[238] = work.v[102];
  var[239] = work.v[104];
  var[240] = work.v[106];
  var[241] = work.v[108];
  var[242] = work.v[110];
  var[243] = work.v[112];
  var[244] = work.v[113];
  var[245] = work.v[114];
  var[246] = work.v[115];
  var[247] = work.v[116];
  var[248] = work.v[117];
  var[249] = work.v[245];
  var[250] = work.v[246];
  var[251] = work.v[247];
  var[252] = work.v[248];
  var[253] = work.v[249];
  var[254] = work.v[250];
  var[255] = work.v[251];
  var[256] = work.v[252];
  var[257] = work.v[253];
  var[258] = work.v[254];
  var[259] = work.v[255];
  var[260] = work.v[260];
#ifndef ZERO_LIBRARY_MODE
  if (settings.debug) {
    printf("Squared norm for solution is %.8g.\n", check_residual(target, var));
  }
#endif
}
void ldl_factor(void) {
  work.d[0] = work.KKT[0];
  if (work.d[0] < 0)
    work.d[0] = settings.kkt_reg;
  else
    work.d[0] += settings.kkt_reg;
  work.d_inv[0] = 1/work.d[0];
  work.L[414] = work.KKT[1]*work.d_inv[0];
  work.v[1] = work.KKT[2];
  work.d[1] = work.v[1];
  if (work.d[1] < 0)
    work.d[1] = settings.kkt_reg;
  else
    work.d[1] += settings.kkt_reg;
  work.d_inv[1] = 1/work.d[1];
  work.L[48] = (work.KKT[3])*work.d_inv[1];
  work.v[2] = work.KKT[4];
  work.d[2] = work.v[2];
  if (work.d[2] < 0)
    work.d[2] = settings.kkt_reg;
  else
    work.d[2] += settings.kkt_reg;
  work.d_inv[2] = 1/work.d[2];
  work.L[53] = (work.KKT[5])*work.d_inv[2];
  work.v[3] = work.KKT[6];
  work.d[3] = work.v[3];
  if (work.d[3] < 0)
    work.d[3] = settings.kkt_reg;
  else
    work.d[3] += settings.kkt_reg;
  work.d_inv[3] = 1/work.d[3];
  work.L[58] = (work.KKT[7])*work.d_inv[3];
  work.v[4] = work.KKT[8];
  work.d[4] = work.v[4];
  if (work.d[4] < 0)
    work.d[4] = settings.kkt_reg;
  else
    work.d[4] += settings.kkt_reg;
  work.d_inv[4] = 1/work.d[4];
  work.L[63] = (work.KKT[9])*work.d_inv[4];
  work.v[5] = work.KKT[10];
  work.d[5] = work.v[5];
  if (work.d[5] < 0)
    work.d[5] = settings.kkt_reg;
  else
    work.d[5] += settings.kkt_reg;
  work.d_inv[5] = 1/work.d[5];
  work.L[68] = (work.KKT[11])*work.d_inv[5];
  work.v[6] = work.KKT[12];
  work.d[6] = work.v[6];
  if (work.d[6] < 0)
    work.d[6] = settings.kkt_reg;
  else
    work.d[6] += settings.kkt_reg;
  work.d_inv[6] = 1/work.d[6];
  work.L[73] = (work.KKT[13])*work.d_inv[6];
  work.v[7] = work.KKT[14];
  work.d[7] = work.v[7];
  if (work.d[7] < 0)
    work.d[7] = settings.kkt_reg;
  else
    work.d[7] += settings.kkt_reg;
  work.d_inv[7] = 1/work.d[7];
  work.L[50] = (work.KKT[15])*work.d_inv[7];
  work.v[8] = work.KKT[16];
  work.d[8] = work.v[8];
  if (work.d[8] < 0)
    work.d[8] = settings.kkt_reg;
  else
    work.d[8] += settings.kkt_reg;
  work.d_inv[8] = 1/work.d[8];
  work.L[55] = (work.KKT[17])*work.d_inv[8];
  work.v[9] = work.KKT[18];
  work.d[9] = work.v[9];
  if (work.d[9] < 0)
    work.d[9] = settings.kkt_reg;
  else
    work.d[9] += settings.kkt_reg;
  work.d_inv[9] = 1/work.d[9];
  work.L[60] = (work.KKT[19])*work.d_inv[9];
  work.v[10] = work.KKT[20];
  work.d[10] = work.v[10];
  if (work.d[10] < 0)
    work.d[10] = settings.kkt_reg;
  else
    work.d[10] += settings.kkt_reg;
  work.d_inv[10] = 1/work.d[10];
  work.L[65] = (work.KKT[21])*work.d_inv[10];
  work.v[11] = work.KKT[22];
  work.d[11] = work.v[11];
  if (work.d[11] < 0)
    work.d[11] = settings.kkt_reg;
  else
    work.d[11] += settings.kkt_reg;
  work.d_inv[11] = 1/work.d[11];
  work.L[70] = (work.KKT[23])*work.d_inv[11];
  work.v[12] = work.KKT[24];
  work.d[12] = work.v[12];
  if (work.d[12] < 0)
    work.d[12] = settings.kkt_reg;
  else
    work.d[12] += settings.kkt_reg;
  work.d_inv[12] = 1/work.d[12];
  work.L[75] = (work.KKT[25])*work.d_inv[12];
  work.v[13] = work.KKT[26];
  work.d[13] = work.v[13];
  if (work.d[13] < 0)
    work.d[13] = settings.kkt_reg;
  else
    work.d[13] += settings.kkt_reg;
  work.d_inv[13] = 1/work.d[13];
  work.L[469] = (work.KKT[27])*work.d_inv[13];
  work.v[14] = work.KKT[28];
  work.d[14] = work.v[14];
  if (work.d[14] < 0)
    work.d[14] = settings.kkt_reg;
  else
    work.d[14] += settings.kkt_reg;
  work.d_inv[14] = 1/work.d[14];
  work.L[108] = (work.KKT[29])*work.d_inv[14];
  work.v[15] = work.KKT[30];
  work.d[15] = work.v[15];
  if (work.d[15] < 0)
    work.d[15] = settings.kkt_reg;
  else
    work.d[15] += settings.kkt_reg;
  work.d_inv[15] = 1/work.d[15];
  work.L[132] = (work.KKT[31])*work.d_inv[15];
  work.v[16] = work.KKT[32];
  work.d[16] = work.v[16];
  if (work.d[16] < 0)
    work.d[16] = settings.kkt_reg;
  else
    work.d[16] += settings.kkt_reg;
  work.d_inv[16] = 1/work.d[16];
  work.L[156] = (work.KKT[33])*work.d_inv[16];
  work.v[17] = work.KKT[34];
  work.d[17] = work.v[17];
  if (work.d[17] < 0)
    work.d[17] = settings.kkt_reg;
  else
    work.d[17] += settings.kkt_reg;
  work.d_inv[17] = 1/work.d[17];
  work.L[180] = (work.KKT[35])*work.d_inv[17];
  work.v[18] = work.KKT[36];
  work.d[18] = work.v[18];
  if (work.d[18] < 0)
    work.d[18] = settings.kkt_reg;
  else
    work.d[18] += settings.kkt_reg;
  work.d_inv[18] = 1/work.d[18];
  work.L[204] = (work.KKT[37])*work.d_inv[18];
  work.v[19] = work.KKT[38];
  work.d[19] = work.v[19];
  if (work.d[19] < 0)
    work.d[19] = settings.kkt_reg;
  else
    work.d[19] += settings.kkt_reg;
  work.d_inv[19] = 1/work.d[19];
  work.L[228] = (work.KKT[39])*work.d_inv[19];
  work.v[20] = work.KKT[40];
  work.d[20] = work.v[20];
  if (work.d[20] < 0)
    work.d[20] = settings.kkt_reg;
  else
    work.d[20] += settings.kkt_reg;
  work.d_inv[20] = 1/work.d[20];
  work.L[111] = (work.KKT[41])*work.d_inv[20];
  work.v[21] = work.KKT[42];
  work.d[21] = work.v[21];
  if (work.d[21] < 0)
    work.d[21] = settings.kkt_reg;
  else
    work.d[21] += settings.kkt_reg;
  work.d_inv[21] = 1/work.d[21];
  work.L[135] = (work.KKT[43])*work.d_inv[21];
  work.v[22] = work.KKT[44];
  work.d[22] = work.v[22];
  if (work.d[22] < 0)
    work.d[22] = settings.kkt_reg;
  else
    work.d[22] += settings.kkt_reg;
  work.d_inv[22] = 1/work.d[22];
  work.L[159] = (work.KKT[45])*work.d_inv[22];
  work.v[23] = work.KKT[46];
  work.d[23] = work.v[23];
  if (work.d[23] < 0)
    work.d[23] = settings.kkt_reg;
  else
    work.d[23] += settings.kkt_reg;
  work.d_inv[23] = 1/work.d[23];
  work.L[183] = (work.KKT[47])*work.d_inv[23];
  work.v[24] = work.KKT[48];
  work.d[24] = work.v[24];
  if (work.d[24] < 0)
    work.d[24] = settings.kkt_reg;
  else
    work.d[24] += settings.kkt_reg;
  work.d_inv[24] = 1/work.d[24];
  work.L[207] = (work.KKT[49])*work.d_inv[24];
  work.v[25] = work.KKT[50];
  work.d[25] = work.v[25];
  if (work.d[25] < 0)
    work.d[25] = settings.kkt_reg;
  else
    work.d[25] += settings.kkt_reg;
  work.d_inv[25] = 1/work.d[25];
  work.L[231] = (work.KKT[51])*work.d_inv[25];
  work.v[26] = work.KKT[52];
  work.d[26] = work.v[26];
  if (work.d[26] < 0)
    work.d[26] = settings.kkt_reg;
  else
    work.d[26] += settings.kkt_reg;
  work.d_inv[26] = 1/work.d[26];
  work.L[525] = (work.KKT[53])*work.d_inv[26];
  work.v[27] = work.KKT[54];
  work.d[27] = work.v[27];
  if (work.d[27] < 0)
    work.d[27] = settings.kkt_reg;
  else
    work.d[27] += settings.kkt_reg;
  work.d_inv[27] = 1/work.d[27];
  work.L[78] = (work.KKT[55])*work.d_inv[27];
  work.v[28] = work.KKT[56];
  work.d[28] = work.v[28];
  if (work.d[28] < 0)
    work.d[28] = settings.kkt_reg;
  else
    work.d[28] += settings.kkt_reg;
  work.d_inv[28] = 1/work.d[28];
  work.L[83] = (work.KKT[57])*work.d_inv[28];
  work.v[29] = work.KKT[58];
  work.d[29] = work.v[29];
  if (work.d[29] < 0)
    work.d[29] = settings.kkt_reg;
  else
    work.d[29] += settings.kkt_reg;
  work.d_inv[29] = 1/work.d[29];
  work.L[88] = (work.KKT[59])*work.d_inv[29];
  work.v[30] = work.KKT[60];
  work.d[30] = work.v[30];
  if (work.d[30] < 0)
    work.d[30] = settings.kkt_reg;
  else
    work.d[30] += settings.kkt_reg;
  work.d_inv[30] = 1/work.d[30];
  work.L[93] = (work.KKT[61])*work.d_inv[30];
  work.v[31] = work.KKT[62];
  work.d[31] = work.v[31];
  if (work.d[31] < 0)
    work.d[31] = settings.kkt_reg;
  else
    work.d[31] += settings.kkt_reg;
  work.d_inv[31] = 1/work.d[31];
  work.L[98] = (work.KKT[63])*work.d_inv[31];
  work.v[32] = work.KKT[64];
  work.d[32] = work.v[32];
  if (work.d[32] < 0)
    work.d[32] = settings.kkt_reg;
  else
    work.d[32] += settings.kkt_reg;
  work.d_inv[32] = 1/work.d[32];
  work.L[103] = (work.KKT[65])*work.d_inv[32];
  work.v[33] = work.KKT[66];
  work.d[33] = work.v[33];
  if (work.d[33] < 0)
    work.d[33] = settings.kkt_reg;
  else
    work.d[33] += settings.kkt_reg;
  work.d_inv[33] = 1/work.d[33];
  work.L[80] = (work.KKT[67])*work.d_inv[33];
  work.v[34] = work.KKT[68];
  work.d[34] = work.v[34];
  if (work.d[34] < 0)
    work.d[34] = settings.kkt_reg;
  else
    work.d[34] += settings.kkt_reg;
  work.d_inv[34] = 1/work.d[34];
  work.L[85] = (work.KKT[69])*work.d_inv[34];
  work.v[35] = work.KKT[70];
  work.d[35] = work.v[35];
  if (work.d[35] < 0)
    work.d[35] = settings.kkt_reg;
  else
    work.d[35] += settings.kkt_reg;
  work.d_inv[35] = 1/work.d[35];
  work.L[90] = (work.KKT[71])*work.d_inv[35];
  work.v[36] = work.KKT[72];
  work.d[36] = work.v[36];
  if (work.d[36] < 0)
    work.d[36] = settings.kkt_reg;
  else
    work.d[36] += settings.kkt_reg;
  work.d_inv[36] = 1/work.d[36];
  work.L[95] = (work.KKT[73])*work.d_inv[36];
  work.v[37] = work.KKT[74];
  work.d[37] = work.v[37];
  if (work.d[37] < 0)
    work.d[37] = settings.kkt_reg;
  else
    work.d[37] += settings.kkt_reg;
  work.d_inv[37] = 1/work.d[37];
  work.L[100] = (work.KKT[75])*work.d_inv[37];
  work.v[38] = work.KKT[76];
  work.d[38] = work.v[38];
  if (work.d[38] < 0)
    work.d[38] = settings.kkt_reg;
  else
    work.d[38] += settings.kkt_reg;
  work.d_inv[38] = 1/work.d[38];
  work.L[105] = (work.KKT[77])*work.d_inv[38];
  work.v[39] = work.KKT[78];
  work.d[39] = work.v[39];
  if (work.d[39] < 0)
    work.d[39] = settings.kkt_reg;
  else
    work.d[39] += settings.kkt_reg;
  work.d_inv[39] = 1/work.d[39];
  work.L[564] = (work.KKT[79])*work.d_inv[39];
  work.v[40] = work.KKT[80];
  work.d[40] = work.v[40];
  if (work.d[40] < 0)
    work.d[40] = settings.kkt_reg;
  else
    work.d[40] += settings.kkt_reg;
  work.d_inv[40] = 1/work.d[40];
  work.L[270] = (work.KKT[81])*work.d_inv[40];
  work.v[41] = work.KKT[82];
  work.d[41] = work.v[41];
  if (work.d[41] < 0)
    work.d[41] = settings.kkt_reg;
  else
    work.d[41] += settings.kkt_reg;
  work.d_inv[41] = 1/work.d[41];
  work.L[281] = (work.KKT[83])*work.d_inv[41];
  work.v[42] = work.KKT[84];
  work.d[42] = work.v[42];
  if (work.d[42] < 0)
    work.d[42] = settings.kkt_reg;
  else
    work.d[42] += settings.kkt_reg;
  work.d_inv[42] = 1/work.d[42];
  work.L[292] = (work.KKT[85])*work.d_inv[42];
  work.v[43] = work.KKT[86];
  work.d[43] = work.v[43];
  if (work.d[43] < 0)
    work.d[43] = settings.kkt_reg;
  else
    work.d[43] += settings.kkt_reg;
  work.d_inv[43] = 1/work.d[43];
  work.L[303] = (work.KKT[87])*work.d_inv[43];
  work.v[44] = work.KKT[88];
  work.d[44] = work.v[44];
  if (work.d[44] < 0)
    work.d[44] = settings.kkt_reg;
  else
    work.d[44] += settings.kkt_reg;
  work.d_inv[44] = 1/work.d[44];
  work.L[314] = (work.KKT[89])*work.d_inv[44];
  work.v[45] = work.KKT[90];
  work.d[45] = work.v[45];
  if (work.d[45] < 0)
    work.d[45] = settings.kkt_reg;
  else
    work.d[45] += settings.kkt_reg;
  work.d_inv[45] = 1/work.d[45];
  work.L[325] = (work.KKT[91])*work.d_inv[45];
  work.v[46] = work.KKT[92];
  work.d[46] = work.v[46];
  if (work.d[46] < 0)
    work.d[46] = settings.kkt_reg;
  else
    work.d[46] += settings.kkt_reg;
  work.d_inv[46] = 1/work.d[46];
  work.L[273] = (work.KKT[93])*work.d_inv[46];
  work.v[47] = work.KKT[94];
  work.d[47] = work.v[47];
  if (work.d[47] < 0)
    work.d[47] = settings.kkt_reg;
  else
    work.d[47] += settings.kkt_reg;
  work.d_inv[47] = 1/work.d[47];
  work.L[284] = (work.KKT[95])*work.d_inv[47];
  work.v[48] = work.KKT[96];
  work.d[48] = work.v[48];
  if (work.d[48] < 0)
    work.d[48] = settings.kkt_reg;
  else
    work.d[48] += settings.kkt_reg;
  work.d_inv[48] = 1/work.d[48];
  work.L[295] = (work.KKT[97])*work.d_inv[48];
  work.v[49] = work.KKT[98];
  work.d[49] = work.v[49];
  if (work.d[49] < 0)
    work.d[49] = settings.kkt_reg;
  else
    work.d[49] += settings.kkt_reg;
  work.d_inv[49] = 1/work.d[49];
  work.L[306] = (work.KKT[99])*work.d_inv[49];
  work.v[50] = work.KKT[100];
  work.d[50] = work.v[50];
  if (work.d[50] < 0)
    work.d[50] = settings.kkt_reg;
  else
    work.d[50] += settings.kkt_reg;
  work.d_inv[50] = 1/work.d[50];
  work.L[317] = (work.KKT[101])*work.d_inv[50];
  work.v[51] = work.KKT[102];
  work.d[51] = work.v[51];
  if (work.d[51] < 0)
    work.d[51] = settings.kkt_reg;
  else
    work.d[51] += settings.kkt_reg;
  work.d_inv[51] = 1/work.d[51];
  work.L[328] = (work.KKT[103])*work.d_inv[51];
  work.v[52] = work.KKT[104];
  work.d[52] = work.v[52];
  if (work.d[52] < 0)
    work.d[52] = settings.kkt_reg;
  else
    work.d[52] += settings.kkt_reg;
  work.d_inv[52] = 1/work.d[52];
  work.L[0] = (work.KKT[105])*work.d_inv[52];
  work.v[53] = work.KKT[106];
  work.d[53] = work.v[53];
  if (work.d[53] < 0)
    work.d[53] = settings.kkt_reg;
  else
    work.d[53] += settings.kkt_reg;
  work.d_inv[53] = 1/work.d[53];
  work.L[2] = (work.KKT[107])*work.d_inv[53];
  work.v[54] = work.KKT[108];
  work.d[54] = work.v[54];
  if (work.d[54] < 0)
    work.d[54] = settings.kkt_reg;
  else
    work.d[54] += settings.kkt_reg;
  work.d_inv[54] = 1/work.d[54];
  work.L[4] = (work.KKT[109])*work.d_inv[54];
  work.v[55] = work.KKT[110];
  work.d[55] = work.v[55];
  if (work.d[55] < 0)
    work.d[55] = settings.kkt_reg;
  else
    work.d[55] += settings.kkt_reg;
  work.d_inv[55] = 1/work.d[55];
  work.L[6] = (work.KKT[111])*work.d_inv[55];
  work.v[56] = work.KKT[112];
  work.d[56] = work.v[56];
  if (work.d[56] < 0)
    work.d[56] = settings.kkt_reg;
  else
    work.d[56] += settings.kkt_reg;
  work.d_inv[56] = 1/work.d[56];
  work.L[8] = (work.KKT[113])*work.d_inv[56];
  work.v[57] = work.KKT[114];
  work.d[57] = work.v[57];
  if (work.d[57] < 0)
    work.d[57] = settings.kkt_reg;
  else
    work.d[57] += settings.kkt_reg;
  work.d_inv[57] = 1/work.d[57];
  work.L[10] = (work.KKT[115])*work.d_inv[57];
  work.v[58] = work.KKT[116];
  work.d[58] = work.v[58];
  if (work.d[58] < 0)
    work.d[58] = settings.kkt_reg;
  else
    work.d[58] += settings.kkt_reg;
  work.d_inv[58] = 1/work.d[58];
  work.L[1] = (work.KKT[117])*work.d_inv[58];
  work.v[59] = work.KKT[118];
  work.d[59] = work.v[59];
  if (work.d[59] < 0)
    work.d[59] = settings.kkt_reg;
  else
    work.d[59] += settings.kkt_reg;
  work.d_inv[59] = 1/work.d[59];
  work.L[3] = (work.KKT[119])*work.d_inv[59];
  work.v[60] = work.KKT[120];
  work.d[60] = work.v[60];
  if (work.d[60] < 0)
    work.d[60] = settings.kkt_reg;
  else
    work.d[60] += settings.kkt_reg;
  work.d_inv[60] = 1/work.d[60];
  work.L[5] = (work.KKT[121])*work.d_inv[60];
  work.v[61] = work.KKT[122];
  work.d[61] = work.v[61];
  if (work.d[61] < 0)
    work.d[61] = settings.kkt_reg;
  else
    work.d[61] += settings.kkt_reg;
  work.d_inv[61] = 1/work.d[61];
  work.L[7] = (work.KKT[123])*work.d_inv[61];
  work.v[62] = work.KKT[124];
  work.d[62] = work.v[62];
  if (work.d[62] < 0)
    work.d[62] = settings.kkt_reg;
  else
    work.d[62] += settings.kkt_reg;
  work.d_inv[62] = 1/work.d[62];
  work.L[9] = (work.KKT[125])*work.d_inv[62];
  work.v[63] = work.KKT[126];
  work.d[63] = work.v[63];
  if (work.d[63] < 0)
    work.d[63] = settings.kkt_reg;
  else
    work.d[63] += settings.kkt_reg;
  work.d_inv[63] = 1/work.d[63];
  work.L[11] = (work.KKT[127])*work.d_inv[63];
  work.v[64] = work.KKT[128];
  work.d[64] = work.v[64];
  if (work.d[64] < 0)
    work.d[64] = settings.kkt_reg;
  else
    work.d[64] += settings.kkt_reg;
  work.d_inv[64] = 1/work.d[64];
  work.L[12] = (work.KKT[129])*work.d_inv[64];
  work.v[65] = work.KKT[130];
  work.d[65] = work.v[65];
  if (work.d[65] < 0)
    work.d[65] = settings.kkt_reg;
  else
    work.d[65] += settings.kkt_reg;
  work.d_inv[65] = 1/work.d[65];
  work.L[14] = (work.KKT[131])*work.d_inv[65];
  work.v[66] = work.KKT[132];
  work.d[66] = work.v[66];
  if (work.d[66] < 0)
    work.d[66] = settings.kkt_reg;
  else
    work.d[66] += settings.kkt_reg;
  work.d_inv[66] = 1/work.d[66];
  work.L[16] = (work.KKT[133])*work.d_inv[66];
  work.v[67] = work.KKT[134];
  work.d[67] = work.v[67];
  if (work.d[67] < 0)
    work.d[67] = settings.kkt_reg;
  else
    work.d[67] += settings.kkt_reg;
  work.d_inv[67] = 1/work.d[67];
  work.L[18] = (work.KKT[135])*work.d_inv[67];
  work.v[68] = work.KKT[136];
  work.d[68] = work.v[68];
  if (work.d[68] < 0)
    work.d[68] = settings.kkt_reg;
  else
    work.d[68] += settings.kkt_reg;
  work.d_inv[68] = 1/work.d[68];
  work.L[20] = (work.KKT[137])*work.d_inv[68];
  work.v[69] = work.KKT[138];
  work.d[69] = work.v[69];
  if (work.d[69] < 0)
    work.d[69] = settings.kkt_reg;
  else
    work.d[69] += settings.kkt_reg;
  work.d_inv[69] = 1/work.d[69];
  work.L[22] = (work.KKT[139])*work.d_inv[69];
  work.v[70] = work.KKT[140];
  work.d[70] = work.v[70];
  if (work.d[70] < 0)
    work.d[70] = settings.kkt_reg;
  else
    work.d[70] += settings.kkt_reg;
  work.d_inv[70] = 1/work.d[70];
  work.L[13] = (work.KKT[141])*work.d_inv[70];
  work.v[71] = work.KKT[142];
  work.d[71] = work.v[71];
  if (work.d[71] < 0)
    work.d[71] = settings.kkt_reg;
  else
    work.d[71] += settings.kkt_reg;
  work.d_inv[71] = 1/work.d[71];
  work.L[15] = (work.KKT[143])*work.d_inv[71];
  work.v[72] = work.KKT[144];
  work.d[72] = work.v[72];
  if (work.d[72] < 0)
    work.d[72] = settings.kkt_reg;
  else
    work.d[72] += settings.kkt_reg;
  work.d_inv[72] = 1/work.d[72];
  work.L[17] = (work.KKT[145])*work.d_inv[72];
  work.v[73] = work.KKT[146];
  work.d[73] = work.v[73];
  if (work.d[73] < 0)
    work.d[73] = settings.kkt_reg;
  else
    work.d[73] += settings.kkt_reg;
  work.d_inv[73] = 1/work.d[73];
  work.L[19] = (work.KKT[147])*work.d_inv[73];
  work.v[74] = work.KKT[148];
  work.d[74] = work.v[74];
  if (work.d[74] < 0)
    work.d[74] = settings.kkt_reg;
  else
    work.d[74] += settings.kkt_reg;
  work.d_inv[74] = 1/work.d[74];
  work.L[21] = (work.KKT[149])*work.d_inv[74];
  work.v[75] = work.KKT[150];
  work.d[75] = work.v[75];
  if (work.d[75] < 0)
    work.d[75] = settings.kkt_reg;
  else
    work.d[75] += settings.kkt_reg;
  work.d_inv[75] = 1/work.d[75];
  work.L[23] = (work.KKT[151])*work.d_inv[75];
  work.v[52] = work.L[0]*work.d[52];
  work.v[76] = work.KKT[152]-work.L[0]*work.v[52];
  work.d[76] = work.v[76];
  if (work.d[76] > 0)
    work.d[76] = -settings.kkt_reg;
  else
    work.d[76] -= settings.kkt_reg;
  work.d_inv[76] = 1/work.d[76];
  work.L[36] = (work.KKT[153])*work.d_inv[76];
  work.v[58] = work.L[1]*work.d[58];
  work.v[77] = work.KKT[154]-work.L[1]*work.v[58];
  work.d[77] = work.v[77];
  if (work.d[77] > 0)
    work.d[77] = -settings.kkt_reg;
  else
    work.d[77] -= settings.kkt_reg;
  work.d_inv[77] = 1/work.d[77];
  work.L[37] = (work.KKT[155])*work.d_inv[77];
  work.v[53] = work.L[2]*work.d[53];
  work.v[78] = work.KKT[156]-work.L[2]*work.v[53];
  work.d[78] = work.v[78];
  if (work.d[78] > 0)
    work.d[78] = -settings.kkt_reg;
  else
    work.d[78] -= settings.kkt_reg;
  work.d_inv[78] = 1/work.d[78];
  work.L[38] = (work.KKT[157])*work.d_inv[78];
  work.v[59] = work.L[3]*work.d[59];
  work.v[79] = work.KKT[158]-work.L[3]*work.v[59];
  work.d[79] = work.v[79];
  if (work.d[79] > 0)
    work.d[79] = -settings.kkt_reg;
  else
    work.d[79] -= settings.kkt_reg;
  work.d_inv[79] = 1/work.d[79];
  work.L[39] = (work.KKT[159])*work.d_inv[79];
  work.v[54] = work.L[4]*work.d[54];
  work.v[80] = work.KKT[160]-work.L[4]*work.v[54];
  work.d[80] = work.v[80];
  if (work.d[80] > 0)
    work.d[80] = -settings.kkt_reg;
  else
    work.d[80] -= settings.kkt_reg;
  work.d_inv[80] = 1/work.d[80];
  work.L[40] = (work.KKT[161])*work.d_inv[80];
  work.v[60] = work.L[5]*work.d[60];
  work.v[81] = work.KKT[162]-work.L[5]*work.v[60];
  work.d[81] = work.v[81];
  if (work.d[81] > 0)
    work.d[81] = -settings.kkt_reg;
  else
    work.d[81] -= settings.kkt_reg;
  work.d_inv[81] = 1/work.d[81];
  work.L[41] = (work.KKT[163])*work.d_inv[81];
  work.v[55] = work.L[6]*work.d[55];
  work.v[82] = work.KKT[164]-work.L[6]*work.v[55];
  work.d[82] = work.v[82];
  if (work.d[82] > 0)
    work.d[82] = -settings.kkt_reg;
  else
    work.d[82] -= settings.kkt_reg;
  work.d_inv[82] = 1/work.d[82];
  work.L[42] = (work.KKT[165])*work.d_inv[82];
  work.v[61] = work.L[7]*work.d[61];
  work.v[83] = work.KKT[166]-work.L[7]*work.v[61];
  work.d[83] = work.v[83];
  if (work.d[83] > 0)
    work.d[83] = -settings.kkt_reg;
  else
    work.d[83] -= settings.kkt_reg;
  work.d_inv[83] = 1/work.d[83];
  work.L[43] = (work.KKT[167])*work.d_inv[83];
  work.v[56] = work.L[8]*work.d[56];
  work.v[84] = work.KKT[168]-work.L[8]*work.v[56];
  work.d[84] = work.v[84];
  if (work.d[84] > 0)
    work.d[84] = -settings.kkt_reg;
  else
    work.d[84] -= settings.kkt_reg;
  work.d_inv[84] = 1/work.d[84];
  work.L[44] = (work.KKT[169])*work.d_inv[84];
  work.v[62] = work.L[9]*work.d[62];
  work.v[85] = work.KKT[170]-work.L[9]*work.v[62];
  work.d[85] = work.v[85];
  if (work.d[85] > 0)
    work.d[85] = -settings.kkt_reg;
  else
    work.d[85] -= settings.kkt_reg;
  work.d_inv[85] = 1/work.d[85];
  work.L[45] = (work.KKT[171])*work.d_inv[85];
  work.v[57] = work.L[10]*work.d[57];
  work.v[86] = work.KKT[172]-work.L[10]*work.v[57];
  work.d[86] = work.v[86];
  if (work.d[86] > 0)
    work.d[86] = -settings.kkt_reg;
  else
    work.d[86] -= settings.kkt_reg;
  work.d_inv[86] = 1/work.d[86];
  work.L[46] = (work.KKT[173])*work.d_inv[86];
  work.v[63] = work.L[11]*work.d[63];
  work.v[87] = work.KKT[174]-work.L[11]*work.v[63];
  work.d[87] = work.v[87];
  if (work.d[87] > 0)
    work.d[87] = -settings.kkt_reg;
  else
    work.d[87] -= settings.kkt_reg;
  work.d_inv[87] = 1/work.d[87];
  work.L[47] = (work.KKT[175])*work.d_inv[87];
  work.v[64] = work.L[12]*work.d[64];
  work.v[88] = work.KKT[176]-work.L[12]*work.v[64];
  work.d[88] = work.v[88];
  if (work.d[88] > 0)
    work.d[88] = -settings.kkt_reg;
  else
    work.d[88] -= settings.kkt_reg;
  work.d_inv[88] = 1/work.d[88];
  work.L[252] = (work.KKT[177])*work.d_inv[88];
  work.v[70] = work.L[13]*work.d[70];
  work.v[89] = work.KKT[178]-work.L[13]*work.v[70];
  work.d[89] = work.v[89];
  if (work.d[89] > 0)
    work.d[89] = -settings.kkt_reg;
  else
    work.d[89] -= settings.kkt_reg;
  work.d_inv[89] = 1/work.d[89];
  work.L[253] = (work.KKT[179])*work.d_inv[89];
  work.v[65] = work.L[14]*work.d[65];
  work.v[90] = work.KKT[180]-work.L[14]*work.v[65];
  work.d[90] = work.v[90];
  if (work.d[90] > 0)
    work.d[90] = -settings.kkt_reg;
  else
    work.d[90] -= settings.kkt_reg;
  work.d_inv[90] = 1/work.d[90];
  work.L[255] = (work.KKT[181])*work.d_inv[90];
  work.v[71] = work.L[15]*work.d[71];
  work.v[91] = work.KKT[182]-work.L[15]*work.v[71];
  work.d[91] = work.v[91];
  if (work.d[91] > 0)
    work.d[91] = -settings.kkt_reg;
  else
    work.d[91] -= settings.kkt_reg;
  work.d_inv[91] = 1/work.d[91];
  work.L[256] = (work.KKT[183])*work.d_inv[91];
  work.v[66] = work.L[16]*work.d[66];
  work.v[92] = work.KKT[184]-work.L[16]*work.v[66];
  work.d[92] = work.v[92];
  if (work.d[92] > 0)
    work.d[92] = -settings.kkt_reg;
  else
    work.d[92] -= settings.kkt_reg;
  work.d_inv[92] = 1/work.d[92];
  work.L[258] = (work.KKT[185])*work.d_inv[92];
  work.v[72] = work.L[17]*work.d[72];
  work.v[93] = work.KKT[186]-work.L[17]*work.v[72];
  work.d[93] = work.v[93];
  if (work.d[93] > 0)
    work.d[93] = -settings.kkt_reg;
  else
    work.d[93] -= settings.kkt_reg;
  work.d_inv[93] = 1/work.d[93];
  work.L[259] = (work.KKT[187])*work.d_inv[93];
  work.v[67] = work.L[18]*work.d[67];
  work.v[94] = work.KKT[188]-work.L[18]*work.v[67];
  work.d[94] = work.v[94];
  if (work.d[94] > 0)
    work.d[94] = -settings.kkt_reg;
  else
    work.d[94] -= settings.kkt_reg;
  work.d_inv[94] = 1/work.d[94];
  work.L[261] = (work.KKT[189])*work.d_inv[94];
  work.v[73] = work.L[19]*work.d[73];
  work.v[95] = work.KKT[190]-work.L[19]*work.v[73];
  work.d[95] = work.v[95];
  if (work.d[95] > 0)
    work.d[95] = -settings.kkt_reg;
  else
    work.d[95] -= settings.kkt_reg;
  work.d_inv[95] = 1/work.d[95];
  work.L[262] = (work.KKT[191])*work.d_inv[95];
  work.v[68] = work.L[20]*work.d[68];
  work.v[96] = work.KKT[192]-work.L[20]*work.v[68];
  work.d[96] = work.v[96];
  if (work.d[96] > 0)
    work.d[96] = -settings.kkt_reg;
  else
    work.d[96] -= settings.kkt_reg;
  work.d_inv[96] = 1/work.d[96];
  work.L[264] = (work.KKT[193])*work.d_inv[96];
  work.v[74] = work.L[21]*work.d[74];
  work.v[97] = work.KKT[194]-work.L[21]*work.v[74];
  work.d[97] = work.v[97];
  if (work.d[97] > 0)
    work.d[97] = -settings.kkt_reg;
  else
    work.d[97] -= settings.kkt_reg;
  work.d_inv[97] = 1/work.d[97];
  work.L[265] = (work.KKT[195])*work.d_inv[97];
  work.v[69] = work.L[22]*work.d[69];
  work.v[98] = work.KKT[196]-work.L[22]*work.v[69];
  work.d[98] = work.v[98];
  if (work.d[98] > 0)
    work.d[98] = -settings.kkt_reg;
  else
    work.d[98] -= settings.kkt_reg;
  work.d_inv[98] = 1/work.d[98];
  work.L[267] = (work.KKT[197])*work.d_inv[98];
  work.v[75] = work.L[23]*work.d[75];
  work.v[99] = work.KKT[198]-work.L[23]*work.v[75];
  work.d[99] = work.v[99];
  if (work.d[99] > 0)
    work.d[99] = -settings.kkt_reg;
  else
    work.d[99] -= settings.kkt_reg;
  work.d_inv[99] = 1/work.d[99];
  work.L[268] = (work.KKT[199])*work.d_inv[99];
  work.v[100] = 0;
  work.d[100] = work.v[100];
  if (work.d[100] > 0)
    work.d[100] = -settings.kkt_reg;
  else
    work.d[100] -= settings.kkt_reg;
  work.d_inv[100] = 1/work.d[100];
  work.L[24] = (work.KKT[200])*work.d_inv[100];
  work.v[100] = work.L[24]*work.d[100];
  work.v[101] = 0-work.L[24]*work.v[100];
  work.d[101] = work.v[101];
  if (work.d[101] < 0)
    work.d[101] = settings.kkt_reg;
  else
    work.d[101] += settings.kkt_reg;
  work.d_inv[101] = 1/work.d[101];
  work.L[119] = (work.KKT[201])*work.d_inv[101];
  work.v[102] = 0;
  work.d[102] = work.v[102];
  if (work.d[102] > 0)
    work.d[102] = -settings.kkt_reg;
  else
    work.d[102] -= settings.kkt_reg;
  work.d_inv[102] = 1/work.d[102];
  work.L[25] = (work.KKT[202])*work.d_inv[102];
  work.v[102] = work.L[25]*work.d[102];
  work.v[103] = 0-work.L[25]*work.v[102];
  work.d[103] = work.v[103];
  if (work.d[103] < 0)
    work.d[103] = settings.kkt_reg;
  else
    work.d[103] += settings.kkt_reg;
  work.d_inv[103] = 1/work.d[103];
  work.L[143] = (work.KKT[203])*work.d_inv[103];
  work.v[104] = 0;
  work.d[104] = work.v[104];
  if (work.d[104] > 0)
    work.d[104] = -settings.kkt_reg;
  else
    work.d[104] -= settings.kkt_reg;
  work.d_inv[104] = 1/work.d[104];
  work.L[26] = (work.KKT[204])*work.d_inv[104];
  work.v[104] = work.L[26]*work.d[104];
  work.v[105] = 0-work.L[26]*work.v[104];
  work.d[105] = work.v[105];
  if (work.d[105] < 0)
    work.d[105] = settings.kkt_reg;
  else
    work.d[105] += settings.kkt_reg;
  work.d_inv[105] = 1/work.d[105];
  work.L[167] = (work.KKT[205])*work.d_inv[105];
  work.v[106] = 0;
  work.d[106] = work.v[106];
  if (work.d[106] > 0)
    work.d[106] = -settings.kkt_reg;
  else
    work.d[106] -= settings.kkt_reg;
  work.d_inv[106] = 1/work.d[106];
  work.L[27] = (work.KKT[206])*work.d_inv[106];
  work.v[106] = work.L[27]*work.d[106];
  work.v[107] = 0-work.L[27]*work.v[106];
  work.d[107] = work.v[107];
  if (work.d[107] < 0)
    work.d[107] = settings.kkt_reg;
  else
    work.d[107] += settings.kkt_reg;
  work.d_inv[107] = 1/work.d[107];
  work.L[191] = (work.KKT[207])*work.d_inv[107];
  work.v[108] = 0;
  work.d[108] = work.v[108];
  if (work.d[108] > 0)
    work.d[108] = -settings.kkt_reg;
  else
    work.d[108] -= settings.kkt_reg;
  work.d_inv[108] = 1/work.d[108];
  work.L[28] = (work.KKT[208])*work.d_inv[108];
  work.v[108] = work.L[28]*work.d[108];
  work.v[109] = 0-work.L[28]*work.v[108];
  work.d[109] = work.v[109];
  if (work.d[109] < 0)
    work.d[109] = settings.kkt_reg;
  else
    work.d[109] += settings.kkt_reg;
  work.d_inv[109] = 1/work.d[109];
  work.L[215] = (work.KKT[209])*work.d_inv[109];
  work.v[110] = 0;
  work.d[110] = work.v[110];
  if (work.d[110] > 0)
    work.d[110] = -settings.kkt_reg;
  else
    work.d[110] -= settings.kkt_reg;
  work.d_inv[110] = 1/work.d[110];
  work.L[29] = (work.KKT[210])*work.d_inv[110];
  work.v[110] = work.L[29]*work.d[110];
  work.v[111] = 0-work.L[29]*work.v[110];
  work.d[111] = work.v[111];
  if (work.d[111] < 0)
    work.d[111] = settings.kkt_reg;
  else
    work.d[111] += settings.kkt_reg;
  work.d_inv[111] = 1/work.d[111];
  work.L[239] = (work.KKT[211])*work.d_inv[111];
  work.v[112] = 0;
  work.d[112] = work.v[112];
  if (work.d[112] > 0)
    work.d[112] = -settings.kkt_reg;
  else
    work.d[112] -= settings.kkt_reg;
  work.d_inv[112] = 1/work.d[112];
  work.L[30] = (work.KKT[212])*work.d_inv[112];
  work.v[113] = 0;
  work.d[113] = work.v[113];
  if (work.d[113] > 0)
    work.d[113] = -settings.kkt_reg;
  else
    work.d[113] -= settings.kkt_reg;
  work.d_inv[113] = 1/work.d[113];
  work.L[31] = (work.KKT[213])*work.d_inv[113];
  work.v[114] = 0;
  work.d[114] = work.v[114];
  if (work.d[114] > 0)
    work.d[114] = -settings.kkt_reg;
  else
    work.d[114] -= settings.kkt_reg;
  work.d_inv[114] = 1/work.d[114];
  work.L[32] = (work.KKT[214])*work.d_inv[114];
  work.v[115] = 0;
  work.d[115] = work.v[115];
  if (work.d[115] > 0)
    work.d[115] = -settings.kkt_reg;
  else
    work.d[115] -= settings.kkt_reg;
  work.d_inv[115] = 1/work.d[115];
  work.L[33] = (work.KKT[215])*work.d_inv[115];
  work.v[116] = 0;
  work.d[116] = work.v[116];
  if (work.d[116] > 0)
    work.d[116] = -settings.kkt_reg;
  else
    work.d[116] -= settings.kkt_reg;
  work.d_inv[116] = 1/work.d[116];
  work.L[34] = (work.KKT[216])*work.d_inv[116];
  work.v[117] = 0;
  work.d[117] = work.v[117];
  if (work.d[117] > 0)
    work.d[117] = -settings.kkt_reg;
  else
    work.d[117] -= settings.kkt_reg;
  work.d_inv[117] = 1/work.d[117];
  work.L[35] = (work.KKT[217])*work.d_inv[117];
  work.v[112] = work.L[30]*work.d[112];
  work.v[118] = 0-work.L[30]*work.v[112];
  work.d[118] = work.v[118];
  if (work.d[118] < 0)
    work.d[118] = settings.kkt_reg;
  else
    work.d[118] += settings.kkt_reg;
  work.d_inv[118] = 1/work.d[118];
  work.L[120] = (work.KKT[218])*work.d_inv[118];
  work.L[126] = (work.KKT[219])*work.d_inv[118];
  work.v[113] = work.L[31]*work.d[113];
  work.v[119] = 0-work.L[31]*work.v[113];
  work.d[119] = work.v[119];
  if (work.d[119] < 0)
    work.d[119] = settings.kkt_reg;
  else
    work.d[119] += settings.kkt_reg;
  work.d_inv[119] = 1/work.d[119];
  work.L[144] = (work.KKT[220])*work.d_inv[119];
  work.L[150] = (work.KKT[221])*work.d_inv[119];
  work.v[114] = work.L[32]*work.d[114];
  work.v[120] = 0-work.L[32]*work.v[114];
  work.d[120] = work.v[120];
  if (work.d[120] < 0)
    work.d[120] = settings.kkt_reg;
  else
    work.d[120] += settings.kkt_reg;
  work.d_inv[120] = 1/work.d[120];
  work.L[168] = (work.KKT[222])*work.d_inv[120];
  work.L[174] = (work.KKT[223])*work.d_inv[120];
  work.v[115] = work.L[33]*work.d[115];
  work.v[121] = 0-work.L[33]*work.v[115];
  work.d[121] = work.v[121];
  if (work.d[121] < 0)
    work.d[121] = settings.kkt_reg;
  else
    work.d[121] += settings.kkt_reg;
  work.d_inv[121] = 1/work.d[121];
  work.L[192] = (work.KKT[224])*work.d_inv[121];
  work.L[198] = (work.KKT[225])*work.d_inv[121];
  work.v[116] = work.L[34]*work.d[116];
  work.v[122] = 0-work.L[34]*work.v[116];
  work.d[122] = work.v[122];
  if (work.d[122] < 0)
    work.d[122] = settings.kkt_reg;
  else
    work.d[122] += settings.kkt_reg;
  work.d_inv[122] = 1/work.d[122];
  work.L[216] = (work.KKT[226])*work.d_inv[122];
  work.L[222] = (work.KKT[227])*work.d_inv[122];
  work.v[117] = work.L[35]*work.d[117];
  work.v[123] = 0-work.L[35]*work.v[117];
  work.d[123] = work.v[123];
  if (work.d[123] < 0)
    work.d[123] = settings.kkt_reg;
  else
    work.d[123] += settings.kkt_reg;
  work.d_inv[123] = 1/work.d[123];
  work.L[240] = (work.KKT[228])*work.d_inv[123];
  work.L[246] = (work.KKT[229])*work.d_inv[123];
  work.v[76] = work.L[36]*work.d[76];
  work.v[77] = work.L[37]*work.d[77];
  work.v[124] = 0-work.L[36]*work.v[76]-work.L[37]*work.v[77];
  work.d[124] = work.v[124];
  if (work.d[124] < 0)
    work.d[124] = settings.kkt_reg;
  else
    work.d[124] += settings.kkt_reg;
  work.d_inv[124] = 1/work.d[124];
  work.L[121] = (work.KKT[230])*work.d_inv[124];
  work.L[336] = (work.KKT[231])*work.d_inv[124];
  work.v[78] = work.L[38]*work.d[78];
  work.v[79] = work.L[39]*work.d[79];
  work.v[125] = 0-work.L[38]*work.v[78]-work.L[39]*work.v[79];
  work.d[125] = work.v[125];
  if (work.d[125] < 0)
    work.d[125] = settings.kkt_reg;
  else
    work.d[125] += settings.kkt_reg;
  work.d_inv[125] = 1/work.d[125];
  work.L[145] = (work.KKT[232])*work.d_inv[125];
  work.L[344] = (work.KKT[233])*work.d_inv[125];
  work.v[80] = work.L[40]*work.d[80];
  work.v[81] = work.L[41]*work.d[81];
  work.v[126] = 0-work.L[40]*work.v[80]-work.L[41]*work.v[81];
  work.d[126] = work.v[126];
  if (work.d[126] < 0)
    work.d[126] = settings.kkt_reg;
  else
    work.d[126] += settings.kkt_reg;
  work.d_inv[126] = 1/work.d[126];
  work.L[169] = (work.KKT[234])*work.d_inv[126];
  work.L[352] = (work.KKT[235])*work.d_inv[126];
  work.v[82] = work.L[42]*work.d[82];
  work.v[83] = work.L[43]*work.d[83];
  work.v[127] = 0-work.L[42]*work.v[82]-work.L[43]*work.v[83];
  work.d[127] = work.v[127];
  if (work.d[127] < 0)
    work.d[127] = settings.kkt_reg;
  else
    work.d[127] += settings.kkt_reg;
  work.d_inv[127] = 1/work.d[127];
  work.L[193] = (work.KKT[236])*work.d_inv[127];
  work.L[360] = (work.KKT[237])*work.d_inv[127];
  work.v[84] = work.L[44]*work.d[84];
  work.v[85] = work.L[45]*work.d[85];
  work.v[128] = 0-work.L[44]*work.v[84]-work.L[45]*work.v[85];
  work.d[128] = work.v[128];
  if (work.d[128] < 0)
    work.d[128] = settings.kkt_reg;
  else
    work.d[128] += settings.kkt_reg;
  work.d_inv[128] = 1/work.d[128];
  work.L[217] = (work.KKT[238])*work.d_inv[128];
  work.L[368] = (work.KKT[239])*work.d_inv[128];
  work.v[86] = work.L[46]*work.d[86];
  work.v[87] = work.L[47]*work.d[87];
  work.v[129] = 0-work.L[46]*work.v[86]-work.L[47]*work.v[87];
  work.d[129] = work.v[129];
  if (work.d[129] < 0)
    work.d[129] = settings.kkt_reg;
  else
    work.d[129] += settings.kkt_reg;
  work.d_inv[129] = 1/work.d[129];
  work.L[241] = (work.KKT[240])*work.d_inv[129];
  work.L[376] = (work.KKT[241])*work.d_inv[129];
  work.v[130] = 0;
  work.d[130] = work.v[130];
  if (work.d[130] < 0)
    work.d[130] = settings.kkt_reg;
  else
    work.d[130] += settings.kkt_reg;
  work.d_inv[130] = 1/work.d[130];
  work.L[49] = (work.KKT[242])*work.d_inv[130];
  work.L[51] = (work.KKT[243])*work.d_inv[130];
  work.L[415] = (work.KKT[244])*work.d_inv[130];
  work.v[1] = work.L[48]*work.d[1];
  work.v[130] = work.L[49]*work.d[130];
  work.v[131] = work.KKT[245]-work.L[48]*work.v[1]-work.L[49]*work.v[130];
  work.d[131] = work.v[131];
  if (work.d[131] > 0)
    work.d[131] = -settings.kkt_reg;
  else
    work.d[131] -= settings.kkt_reg;
  work.d_inv[131] = 1/work.d[131];
  work.L[52] = (-work.L[51]*work.v[130])*work.d_inv[131];
  work.L[115] = (work.KKT[246])*work.d_inv[131];
  work.L[416] = (-work.L[415]*work.v[130])*work.d_inv[131];
  work.v[7] = work.L[50]*work.d[7];
  work.v[130] = work.L[51]*work.d[130];
  work.v[131] = work.L[52]*work.d[131];
  work.v[132] = work.KKT[247]-work.L[50]*work.v[7]-work.L[51]*work.v[130]-work.L[52]*work.v[131];
  work.d[132] = work.v[132];
  if (work.d[132] > 0)
    work.d[132] = -settings.kkt_reg;
  else
    work.d[132] -= settings.kkt_reg;
  work.d_inv[132] = 1/work.d[132];
  work.L[116] = (work.KKT[248]-work.L[115]*work.v[131])*work.d_inv[132];
  work.L[417] = (-work.L[415]*work.v[130]-work.L[416]*work.v[131])*work.d_inv[132];
  work.v[133] = 0;
  work.d[133] = work.v[133];
  if (work.d[133] < 0)
    work.d[133] = settings.kkt_reg;
  else
    work.d[133] += settings.kkt_reg;
  work.d_inv[133] = 1/work.d[133];
  work.L[54] = (work.KKT[249])*work.d_inv[133];
  work.L[56] = (work.KKT[250])*work.d_inv[133];
  work.L[418] = (work.KKT[251])*work.d_inv[133];
  work.v[2] = work.L[53]*work.d[2];
  work.v[133] = work.L[54]*work.d[133];
  work.v[134] = work.KKT[252]-work.L[53]*work.v[2]-work.L[54]*work.v[133];
  work.d[134] = work.v[134];
  if (work.d[134] > 0)
    work.d[134] = -settings.kkt_reg;
  else
    work.d[134] -= settings.kkt_reg;
  work.d_inv[134] = 1/work.d[134];
  work.L[57] = (-work.L[56]*work.v[133])*work.d_inv[134];
  work.L[139] = (work.KKT[253])*work.d_inv[134];
  work.L[419] = (-work.L[418]*work.v[133])*work.d_inv[134];
  work.v[8] = work.L[55]*work.d[8];
  work.v[133] = work.L[56]*work.d[133];
  work.v[134] = work.L[57]*work.d[134];
  work.v[135] = work.KKT[254]-work.L[55]*work.v[8]-work.L[56]*work.v[133]-work.L[57]*work.v[134];
  work.d[135] = work.v[135];
  if (work.d[135] > 0)
    work.d[135] = -settings.kkt_reg;
  else
    work.d[135] -= settings.kkt_reg;
  work.d_inv[135] = 1/work.d[135];
  work.L[140] = (work.KKT[255]-work.L[139]*work.v[134])*work.d_inv[135];
  work.L[420] = (-work.L[418]*work.v[133]-work.L[419]*work.v[134])*work.d_inv[135];
  work.v[136] = 0;
  work.d[136] = work.v[136];
  if (work.d[136] < 0)
    work.d[136] = settings.kkt_reg;
  else
    work.d[136] += settings.kkt_reg;
  work.d_inv[136] = 1/work.d[136];
  work.L[59] = (work.KKT[256])*work.d_inv[136];
  work.L[61] = (work.KKT[257])*work.d_inv[136];
  work.L[421] = (work.KKT[258])*work.d_inv[136];
  work.v[3] = work.L[58]*work.d[3];
  work.v[136] = work.L[59]*work.d[136];
  work.v[137] = work.KKT[259]-work.L[58]*work.v[3]-work.L[59]*work.v[136];
  work.d[137] = work.v[137];
  if (work.d[137] > 0)
    work.d[137] = -settings.kkt_reg;
  else
    work.d[137] -= settings.kkt_reg;
  work.d_inv[137] = 1/work.d[137];
  work.L[62] = (-work.L[61]*work.v[136])*work.d_inv[137];
  work.L[163] = (work.KKT[260])*work.d_inv[137];
  work.L[422] = (-work.L[421]*work.v[136])*work.d_inv[137];
  work.v[9] = work.L[60]*work.d[9];
  work.v[136] = work.L[61]*work.d[136];
  work.v[137] = work.L[62]*work.d[137];
  work.v[138] = work.KKT[261]-work.L[60]*work.v[9]-work.L[61]*work.v[136]-work.L[62]*work.v[137];
  work.d[138] = work.v[138];
  if (work.d[138] > 0)
    work.d[138] = -settings.kkt_reg;
  else
    work.d[138] -= settings.kkt_reg;
  work.d_inv[138] = 1/work.d[138];
  work.L[164] = (work.KKT[262]-work.L[163]*work.v[137])*work.d_inv[138];
  work.L[423] = (-work.L[421]*work.v[136]-work.L[422]*work.v[137])*work.d_inv[138];
  work.v[139] = 0;
  work.d[139] = work.v[139];
  if (work.d[139] < 0)
    work.d[139] = settings.kkt_reg;
  else
    work.d[139] += settings.kkt_reg;
  work.d_inv[139] = 1/work.d[139];
  work.L[64] = (work.KKT[263])*work.d_inv[139];
  work.L[66] = (work.KKT[264])*work.d_inv[139];
  work.L[424] = (work.KKT[265])*work.d_inv[139];
  work.v[4] = work.L[63]*work.d[4];
  work.v[139] = work.L[64]*work.d[139];
  work.v[140] = work.KKT[266]-work.L[63]*work.v[4]-work.L[64]*work.v[139];
  work.d[140] = work.v[140];
  if (work.d[140] > 0)
    work.d[140] = -settings.kkt_reg;
  else
    work.d[140] -= settings.kkt_reg;
  work.d_inv[140] = 1/work.d[140];
  work.L[67] = (-work.L[66]*work.v[139])*work.d_inv[140];
  work.L[187] = (work.KKT[267])*work.d_inv[140];
  work.L[425] = (-work.L[424]*work.v[139])*work.d_inv[140];
  work.v[10] = work.L[65]*work.d[10];
  work.v[139] = work.L[66]*work.d[139];
  work.v[140] = work.L[67]*work.d[140];
  work.v[141] = work.KKT[268]-work.L[65]*work.v[10]-work.L[66]*work.v[139]-work.L[67]*work.v[140];
  work.d[141] = work.v[141];
  if (work.d[141] > 0)
    work.d[141] = -settings.kkt_reg;
  else
    work.d[141] -= settings.kkt_reg;
  work.d_inv[141] = 1/work.d[141];
  work.L[188] = (work.KKT[269]-work.L[187]*work.v[140])*work.d_inv[141];
  work.L[426] = (-work.L[424]*work.v[139]-work.L[425]*work.v[140])*work.d_inv[141];
  work.v[142] = 0;
  work.d[142] = work.v[142];
  if (work.d[142] < 0)
    work.d[142] = settings.kkt_reg;
  else
    work.d[142] += settings.kkt_reg;
  work.d_inv[142] = 1/work.d[142];
  work.L[69] = (work.KKT[270])*work.d_inv[142];
  work.L[71] = (work.KKT[271])*work.d_inv[142];
  work.L[427] = (work.KKT[272])*work.d_inv[142];
  work.v[5] = work.L[68]*work.d[5];
  work.v[142] = work.L[69]*work.d[142];
  work.v[143] = work.KKT[273]-work.L[68]*work.v[5]-work.L[69]*work.v[142];
  work.d[143] = work.v[143];
  if (work.d[143] > 0)
    work.d[143] = -settings.kkt_reg;
  else
    work.d[143] -= settings.kkt_reg;
  work.d_inv[143] = 1/work.d[143];
  work.L[72] = (-work.L[71]*work.v[142])*work.d_inv[143];
  work.L[211] = (work.KKT[274])*work.d_inv[143];
  work.L[428] = (-work.L[427]*work.v[142])*work.d_inv[143];
  work.v[11] = work.L[70]*work.d[11];
  work.v[142] = work.L[71]*work.d[142];
  work.v[143] = work.L[72]*work.d[143];
  work.v[144] = work.KKT[275]-work.L[70]*work.v[11]-work.L[71]*work.v[142]-work.L[72]*work.v[143];
  work.d[144] = work.v[144];
  if (work.d[144] > 0)
    work.d[144] = -settings.kkt_reg;
  else
    work.d[144] -= settings.kkt_reg;
  work.d_inv[144] = 1/work.d[144];
  work.L[212] = (work.KKT[276]-work.L[211]*work.v[143])*work.d_inv[144];
  work.L[429] = (-work.L[427]*work.v[142]-work.L[428]*work.v[143])*work.d_inv[144];
  work.v[145] = 0;
  work.d[145] = work.v[145];
  if (work.d[145] < 0)
    work.d[145] = settings.kkt_reg;
  else
    work.d[145] += settings.kkt_reg;
  work.d_inv[145] = 1/work.d[145];
  work.L[74] = (work.KKT[277])*work.d_inv[145];
  work.L[76] = (work.KKT[278])*work.d_inv[145];
  work.L[430] = (work.KKT[279])*work.d_inv[145];
  work.v[6] = work.L[73]*work.d[6];
  work.v[145] = work.L[74]*work.d[145];
  work.v[146] = work.KKT[280]-work.L[73]*work.v[6]-work.L[74]*work.v[145];
  work.d[146] = work.v[146];
  if (work.d[146] > 0)
    work.d[146] = -settings.kkt_reg;
  else
    work.d[146] -= settings.kkt_reg;
  work.d_inv[146] = 1/work.d[146];
  work.L[77] = (-work.L[76]*work.v[145])*work.d_inv[146];
  work.L[235] = (work.KKT[281])*work.d_inv[146];
  work.L[431] = (-work.L[430]*work.v[145])*work.d_inv[146];
  work.v[12] = work.L[75]*work.d[12];
  work.v[145] = work.L[76]*work.d[145];
  work.v[146] = work.L[77]*work.d[146];
  work.v[147] = work.KKT[282]-work.L[75]*work.v[12]-work.L[76]*work.v[145]-work.L[77]*work.v[146];
  work.d[147] = work.v[147];
  if (work.d[147] > 0)
    work.d[147] = -settings.kkt_reg;
  else
    work.d[147] -= settings.kkt_reg;
  work.d_inv[147] = 1/work.d[147];
  work.L[236] = (work.KKT[283]-work.L[235]*work.v[146])*work.d_inv[147];
  work.L[432] = (-work.L[430]*work.v[145]-work.L[431]*work.v[146])*work.d_inv[147];
  work.v[148] = 0;
  work.d[148] = work.v[148];
  if (work.d[148] < 0)
    work.d[148] = settings.kkt_reg;
  else
    work.d[148] += settings.kkt_reg;
  work.d_inv[148] = 1/work.d[148];
  work.L[109] = (work.KKT[284])*work.d_inv[148];
  work.L[112] = (work.KKT[285])*work.d_inv[148];
  work.L[470] = (work.KKT[286])*work.d_inv[148];
  work.v[149] = 0;
  work.d[149] = work.v[149];
  if (work.d[149] < 0)
    work.d[149] = settings.kkt_reg;
  else
    work.d[149] += settings.kkt_reg;
  work.d_inv[149] = 1/work.d[149];
  work.L[133] = (work.KKT[287])*work.d_inv[149];
  work.L[136] = (work.KKT[288])*work.d_inv[149];
  work.L[471] = (work.KKT[289])*work.d_inv[149];
  work.v[150] = 0;
  work.d[150] = work.v[150];
  if (work.d[150] < 0)
    work.d[150] = settings.kkt_reg;
  else
    work.d[150] += settings.kkt_reg;
  work.d_inv[150] = 1/work.d[150];
  work.L[157] = (work.KKT[290])*work.d_inv[150];
  work.L[160] = (work.KKT[291])*work.d_inv[150];
  work.L[472] = (work.KKT[292])*work.d_inv[150];
  work.v[151] = 0;
  work.d[151] = work.v[151];
  if (work.d[151] < 0)
    work.d[151] = settings.kkt_reg;
  else
    work.d[151] += settings.kkt_reg;
  work.d_inv[151] = 1/work.d[151];
  work.L[181] = (work.KKT[293])*work.d_inv[151];
  work.L[184] = (work.KKT[294])*work.d_inv[151];
  work.L[473] = (work.KKT[295])*work.d_inv[151];
  work.v[152] = 0;
  work.d[152] = work.v[152];
  if (work.d[152] < 0)
    work.d[152] = settings.kkt_reg;
  else
    work.d[152] += settings.kkt_reg;
  work.d_inv[152] = 1/work.d[152];
  work.L[205] = (work.KKT[296])*work.d_inv[152];
  work.L[208] = (work.KKT[297])*work.d_inv[152];
  work.L[474] = (work.KKT[298])*work.d_inv[152];
  work.v[153] = 0;
  work.d[153] = work.v[153];
  if (work.d[153] < 0)
    work.d[153] = settings.kkt_reg;
  else
    work.d[153] += settings.kkt_reg;
  work.d_inv[153] = 1/work.d[153];
  work.L[229] = (work.KKT[299])*work.d_inv[153];
  work.L[232] = (work.KKT[300])*work.d_inv[153];
  work.L[475] = (work.KKT[301])*work.d_inv[153];
  work.v[154] = 0;
  work.d[154] = work.v[154];
  if (work.d[154] < 0)
    work.d[154] = settings.kkt_reg;
  else
    work.d[154] += settings.kkt_reg;
  work.d_inv[154] = 1/work.d[154];
  work.L[79] = (work.KKT[302])*work.d_inv[154];
  work.L[81] = (work.KKT[303])*work.d_inv[154];
  work.L[526] = (work.KKT[304])*work.d_inv[154];
  work.v[27] = work.L[78]*work.d[27];
  work.v[154] = work.L[79]*work.d[154];
  work.v[155] = work.KKT[305]-work.L[78]*work.v[27]-work.L[79]*work.v[154];
  work.d[155] = work.v[155];
  if (work.d[155] > 0)
    work.d[155] = -settings.kkt_reg;
  else
    work.d[155] -= settings.kkt_reg;
  work.d_inv[155] = 1/work.d[155];
  work.L[82] = (-work.L[81]*work.v[154])*work.d_inv[155];
  work.L[277] = (work.KKT[306])*work.d_inv[155];
  work.L[527] = (-work.L[526]*work.v[154])*work.d_inv[155];
  work.v[33] = work.L[80]*work.d[33];
  work.v[154] = work.L[81]*work.d[154];
  work.v[155] = work.L[82]*work.d[155];
  work.v[156] = work.KKT[307]-work.L[80]*work.v[33]-work.L[81]*work.v[154]-work.L[82]*work.v[155];
  work.d[156] = work.v[156];
  if (work.d[156] > 0)
    work.d[156] = -settings.kkt_reg;
  else
    work.d[156] -= settings.kkt_reg;
  work.d_inv[156] = 1/work.d[156];
  work.L[278] = (work.KKT[308]-work.L[277]*work.v[155])*work.d_inv[156];
  work.L[528] = (-work.L[526]*work.v[154]-work.L[527]*work.v[155])*work.d_inv[156];
  work.v[157] = 0;
  work.d[157] = work.v[157];
  if (work.d[157] < 0)
    work.d[157] = settings.kkt_reg;
  else
    work.d[157] += settings.kkt_reg;
  work.d_inv[157] = 1/work.d[157];
  work.L[84] = (work.KKT[309])*work.d_inv[157];
  work.L[86] = (work.KKT[310])*work.d_inv[157];
  work.L[529] = (work.KKT[311])*work.d_inv[157];
  work.v[28] = work.L[83]*work.d[28];
  work.v[157] = work.L[84]*work.d[157];
  work.v[158] = work.KKT[312]-work.L[83]*work.v[28]-work.L[84]*work.v[157];
  work.d[158] = work.v[158];
  if (work.d[158] > 0)
    work.d[158] = -settings.kkt_reg;
  else
    work.d[158] -= settings.kkt_reg;
  work.d_inv[158] = 1/work.d[158];
  work.L[87] = (-work.L[86]*work.v[157])*work.d_inv[158];
  work.L[288] = (work.KKT[313])*work.d_inv[158];
  work.L[530] = (-work.L[529]*work.v[157])*work.d_inv[158];
  work.v[34] = work.L[85]*work.d[34];
  work.v[157] = work.L[86]*work.d[157];
  work.v[158] = work.L[87]*work.d[158];
  work.v[159] = work.KKT[314]-work.L[85]*work.v[34]-work.L[86]*work.v[157]-work.L[87]*work.v[158];
  work.d[159] = work.v[159];
  if (work.d[159] > 0)
    work.d[159] = -settings.kkt_reg;
  else
    work.d[159] -= settings.kkt_reg;
  work.d_inv[159] = 1/work.d[159];
  work.L[289] = (work.KKT[315]-work.L[288]*work.v[158])*work.d_inv[159];
  work.L[531] = (-work.L[529]*work.v[157]-work.L[530]*work.v[158])*work.d_inv[159];
  work.v[160] = 0;
  work.d[160] = work.v[160];
  if (work.d[160] < 0)
    work.d[160] = settings.kkt_reg;
  else
    work.d[160] += settings.kkt_reg;
  work.d_inv[160] = 1/work.d[160];
  work.L[89] = (work.KKT[316])*work.d_inv[160];
  work.L[91] = (work.KKT[317])*work.d_inv[160];
  work.L[532] = (work.KKT[318])*work.d_inv[160];
  work.v[29] = work.L[88]*work.d[29];
  work.v[160] = work.L[89]*work.d[160];
  work.v[161] = work.KKT[319]-work.L[88]*work.v[29]-work.L[89]*work.v[160];
  work.d[161] = work.v[161];
  if (work.d[161] > 0)
    work.d[161] = -settings.kkt_reg;
  else
    work.d[161] -= settings.kkt_reg;
  work.d_inv[161] = 1/work.d[161];
  work.L[92] = (-work.L[91]*work.v[160])*work.d_inv[161];
  work.L[299] = (work.KKT[320])*work.d_inv[161];
  work.L[533] = (-work.L[532]*work.v[160])*work.d_inv[161];
  work.v[35] = work.L[90]*work.d[35];
  work.v[160] = work.L[91]*work.d[160];
  work.v[161] = work.L[92]*work.d[161];
  work.v[162] = work.KKT[321]-work.L[90]*work.v[35]-work.L[91]*work.v[160]-work.L[92]*work.v[161];
  work.d[162] = work.v[162];
  if (work.d[162] > 0)
    work.d[162] = -settings.kkt_reg;
  else
    work.d[162] -= settings.kkt_reg;
  work.d_inv[162] = 1/work.d[162];
  work.L[300] = (work.KKT[322]-work.L[299]*work.v[161])*work.d_inv[162];
  work.L[534] = (-work.L[532]*work.v[160]-work.L[533]*work.v[161])*work.d_inv[162];
  work.v[163] = 0;
  work.d[163] = work.v[163];
  if (work.d[163] < 0)
    work.d[163] = settings.kkt_reg;
  else
    work.d[163] += settings.kkt_reg;
  work.d_inv[163] = 1/work.d[163];
  work.L[94] = (work.KKT[323])*work.d_inv[163];
  work.L[96] = (work.KKT[324])*work.d_inv[163];
  work.L[535] = (work.KKT[325])*work.d_inv[163];
  work.v[30] = work.L[93]*work.d[30];
  work.v[163] = work.L[94]*work.d[163];
  work.v[164] = work.KKT[326]-work.L[93]*work.v[30]-work.L[94]*work.v[163];
  work.d[164] = work.v[164];
  if (work.d[164] > 0)
    work.d[164] = -settings.kkt_reg;
  else
    work.d[164] -= settings.kkt_reg;
  work.d_inv[164] = 1/work.d[164];
  work.L[97] = (-work.L[96]*work.v[163])*work.d_inv[164];
  work.L[310] = (work.KKT[327])*work.d_inv[164];
  work.L[536] = (-work.L[535]*work.v[163])*work.d_inv[164];
  work.v[36] = work.L[95]*work.d[36];
  work.v[163] = work.L[96]*work.d[163];
  work.v[164] = work.L[97]*work.d[164];
  work.v[165] = work.KKT[328]-work.L[95]*work.v[36]-work.L[96]*work.v[163]-work.L[97]*work.v[164];
  work.d[165] = work.v[165];
  if (work.d[165] > 0)
    work.d[165] = -settings.kkt_reg;
  else
    work.d[165] -= settings.kkt_reg;
  work.d_inv[165] = 1/work.d[165];
  work.L[311] = (work.KKT[329]-work.L[310]*work.v[164])*work.d_inv[165];
  work.L[537] = (-work.L[535]*work.v[163]-work.L[536]*work.v[164])*work.d_inv[165];
  work.v[166] = 0;
  work.d[166] = work.v[166];
  if (work.d[166] < 0)
    work.d[166] = settings.kkt_reg;
  else
    work.d[166] += settings.kkt_reg;
  work.d_inv[166] = 1/work.d[166];
  work.L[99] = (work.KKT[330])*work.d_inv[166];
  work.L[101] = (work.KKT[331])*work.d_inv[166];
  work.L[538] = (work.KKT[332])*work.d_inv[166];
  work.v[31] = work.L[98]*work.d[31];
  work.v[166] = work.L[99]*work.d[166];
  work.v[167] = work.KKT[333]-work.L[98]*work.v[31]-work.L[99]*work.v[166];
  work.d[167] = work.v[167];
  if (work.d[167] > 0)
    work.d[167] = -settings.kkt_reg;
  else
    work.d[167] -= settings.kkt_reg;
  work.d_inv[167] = 1/work.d[167];
  work.L[102] = (-work.L[101]*work.v[166])*work.d_inv[167];
  work.L[321] = (work.KKT[334])*work.d_inv[167];
  work.L[539] = (-work.L[538]*work.v[166])*work.d_inv[167];
  work.v[37] = work.L[100]*work.d[37];
  work.v[166] = work.L[101]*work.d[166];
  work.v[167] = work.L[102]*work.d[167];
  work.v[168] = work.KKT[335]-work.L[100]*work.v[37]-work.L[101]*work.v[166]-work.L[102]*work.v[167];
  work.d[168] = work.v[168];
  if (work.d[168] > 0)
    work.d[168] = -settings.kkt_reg;
  else
    work.d[168] -= settings.kkt_reg;
  work.d_inv[168] = 1/work.d[168];
  work.L[322] = (work.KKT[336]-work.L[321]*work.v[167])*work.d_inv[168];
  work.L[540] = (-work.L[538]*work.v[166]-work.L[539]*work.v[167])*work.d_inv[168];
  work.v[169] = 0;
  work.d[169] = work.v[169];
  if (work.d[169] < 0)
    work.d[169] = settings.kkt_reg;
  else
    work.d[169] += settings.kkt_reg;
  work.d_inv[169] = 1/work.d[169];
  work.L[104] = (work.KKT[337])*work.d_inv[169];
  work.L[106] = (work.KKT[338])*work.d_inv[169];
  work.L[541] = (work.KKT[339])*work.d_inv[169];
  work.v[32] = work.L[103]*work.d[32];
  work.v[169] = work.L[104]*work.d[169];
  work.v[170] = work.KKT[340]-work.L[103]*work.v[32]-work.L[104]*work.v[169];
  work.d[170] = work.v[170];
  if (work.d[170] > 0)
    work.d[170] = -settings.kkt_reg;
  else
    work.d[170] -= settings.kkt_reg;
  work.d_inv[170] = 1/work.d[170];
  work.L[107] = (-work.L[106]*work.v[169])*work.d_inv[170];
  work.L[332] = (work.KKT[341])*work.d_inv[170];
  work.L[542] = (-work.L[541]*work.v[169])*work.d_inv[170];
  work.v[38] = work.L[105]*work.d[38];
  work.v[169] = work.L[106]*work.d[169];
  work.v[170] = work.L[107]*work.d[170];
  work.v[171] = work.KKT[342]-work.L[105]*work.v[38]-work.L[106]*work.v[169]-work.L[107]*work.v[170];
  work.d[171] = work.v[171];
  if (work.d[171] > 0)
    work.d[171] = -settings.kkt_reg;
  else
    work.d[171] -= settings.kkt_reg;
  work.d_inv[171] = 1/work.d[171];
  work.L[333] = (work.KKT[343]-work.L[332]*work.v[170])*work.d_inv[171];
  work.L[543] = (-work.L[541]*work.v[169]-work.L[542]*work.v[170])*work.d_inv[171];
  work.v[172] = 0;
  work.d[172] = work.v[172];
  if (work.d[172] < 0)
    work.d[172] = settings.kkt_reg;
  else
    work.d[172] += settings.kkt_reg;
  work.d_inv[172] = 1/work.d[172];
  work.L[271] = (work.KKT[344])*work.d_inv[172];
  work.L[274] = (work.KKT[345])*work.d_inv[172];
  work.L[565] = (work.KKT[346])*work.d_inv[172];
  work.v[173] = 0;
  work.d[173] = work.v[173];
  if (work.d[173] < 0)
    work.d[173] = settings.kkt_reg;
  else
    work.d[173] += settings.kkt_reg;
  work.d_inv[173] = 1/work.d[173];
  work.L[282] = (work.KKT[347])*work.d_inv[173];
  work.L[285] = (work.KKT[348])*work.d_inv[173];
  work.L[566] = (work.KKT[349])*work.d_inv[173];
  work.v[174] = 0;
  work.d[174] = work.v[174];
  if (work.d[174] < 0)
    work.d[174] = settings.kkt_reg;
  else
    work.d[174] += settings.kkt_reg;
  work.d_inv[174] = 1/work.d[174];
  work.L[293] = (work.KKT[350])*work.d_inv[174];
  work.L[296] = (work.KKT[351])*work.d_inv[174];
  work.L[567] = (work.KKT[352])*work.d_inv[174];
  work.v[175] = 0;
  work.d[175] = work.v[175];
  if (work.d[175] < 0)
    work.d[175] = settings.kkt_reg;
  else
    work.d[175] += settings.kkt_reg;
  work.d_inv[175] = 1/work.d[175];
  work.L[304] = (work.KKT[353])*work.d_inv[175];
  work.L[307] = (work.KKT[354])*work.d_inv[175];
  work.L[568] = (work.KKT[355])*work.d_inv[175];
  work.v[176] = 0;
  work.d[176] = work.v[176];
  if (work.d[176] < 0)
    work.d[176] = settings.kkt_reg;
  else
    work.d[176] += settings.kkt_reg;
  work.d_inv[176] = 1/work.d[176];
  work.L[315] = (work.KKT[356])*work.d_inv[176];
  work.L[318] = (work.KKT[357])*work.d_inv[176];
  work.L[569] = (work.KKT[358])*work.d_inv[176];
  work.v[177] = 0;
  work.d[177] = work.v[177];
  if (work.d[177] < 0)
    work.d[177] = settings.kkt_reg;
  else
    work.d[177] += settings.kkt_reg;
  work.d_inv[177] = 1/work.d[177];
  work.L[326] = (work.KKT[359])*work.d_inv[177];
  work.L[329] = (work.KKT[360])*work.d_inv[177];
  work.L[570] = (work.KKT[361])*work.d_inv[177];
  work.v[178] = 0;
  work.d[178] = work.v[178];
  if (work.d[178] < 0)
    work.d[178] = settings.kkt_reg;
  else
    work.d[178] += settings.kkt_reg;
  work.d_inv[178] = 1/work.d[178];
  work.L[110] = (work.KKT[362])*work.d_inv[178];
  work.L[113] = (work.KKT[363])*work.d_inv[178];
  work.L[122] = (work.KKT[364])*work.d_inv[178];
  work.L[127] = (work.KKT[365])*work.d_inv[178];
  work.v[14] = work.L[108]*work.d[14];
  work.v[148] = work.L[109]*work.d[148];
  work.v[178] = work.L[110]*work.d[178];
  work.v[179] = work.KKT[366]-work.L[108]*work.v[14]-work.L[109]*work.v[148]-work.L[110]*work.v[178];
  work.d[179] = work.v[179];
  if (work.d[179] > 0)
    work.d[179] = -settings.kkt_reg;
  else
    work.d[179] -= settings.kkt_reg;
  work.d_inv[179] = 1/work.d[179];
  work.L[114] = (-work.L[112]*work.v[148]-work.L[113]*work.v[178])*work.d_inv[179];
  work.L[117] = (work.KKT[367])*work.d_inv[179];
  work.L[123] = (-work.L[122]*work.v[178])*work.d_inv[179];
  work.L[128] = (-work.L[127]*work.v[178])*work.d_inv[179];
  work.L[476] = (-work.L[470]*work.v[148])*work.d_inv[179];
  work.v[20] = work.L[111]*work.d[20];
  work.v[148] = work.L[112]*work.d[148];
  work.v[178] = work.L[113]*work.d[178];
  work.v[179] = work.L[114]*work.d[179];
  work.v[180] = work.KKT[368]-work.L[111]*work.v[20]-work.L[112]*work.v[148]-work.L[113]*work.v[178]-work.L[114]*work.v[179];
  work.d[180] = work.v[180];
  if (work.d[180] > 0)
    work.d[180] = -settings.kkt_reg;
  else
    work.d[180] -= settings.kkt_reg;
  work.d_inv[180] = 1/work.d[180];
  work.L[118] = (work.KKT[369]-work.L[117]*work.v[179])*work.d_inv[180];
  work.L[124] = (-work.L[122]*work.v[178]-work.L[123]*work.v[179])*work.d_inv[180];
  work.L[129] = (-work.L[127]*work.v[178]-work.L[128]*work.v[179])*work.d_inv[180];
  work.L[477] = (-work.L[470]*work.v[148]-work.L[476]*work.v[179])*work.d_inv[180];
  work.v[131] = work.L[115]*work.d[131];
  work.v[132] = work.L[116]*work.d[132];
  work.v[179] = work.L[117]*work.d[179];
  work.v[180] = work.L[118]*work.d[180];
  work.v[181] = 0-work.L[115]*work.v[131]-work.L[116]*work.v[132]-work.L[117]*work.v[179]-work.L[118]*work.v[180];
  work.d[181] = work.v[181];
  if (work.d[181] < 0)
    work.d[181] = settings.kkt_reg;
  else
    work.d[181] += settings.kkt_reg;
  work.d_inv[181] = 1/work.d[181];
  work.L[125] = (work.KKT[370]-work.L[123]*work.v[179]-work.L[124]*work.v[180])*work.d_inv[181];
  work.L[130] = (work.KKT[371]-work.L[128]*work.v[179]-work.L[129]*work.v[180])*work.d_inv[181];
  work.L[433] = (-work.L[416]*work.v[131]-work.L[417]*work.v[132])*work.d_inv[181];
  work.L[478] = (-work.L[476]*work.v[179]-work.L[477]*work.v[180])*work.d_inv[181];
  work.v[101] = work.L[119]*work.d[101];
  work.v[118] = work.L[120]*work.d[118];
  work.v[124] = work.L[121]*work.d[124];
  work.v[178] = work.L[122]*work.d[178];
  work.v[179] = work.L[123]*work.d[179];
  work.v[180] = work.L[124]*work.d[180];
  work.v[181] = work.L[125]*work.d[181];
  work.v[182] = 0-work.L[119]*work.v[101]-work.L[120]*work.v[118]-work.L[121]*work.v[124]-work.L[122]*work.v[178]-work.L[123]*work.v[179]-work.L[124]*work.v[180]-work.L[125]*work.v[181];
  work.d[182] = work.v[182];
  if (work.d[182] > 0)
    work.d[182] = -settings.kkt_reg;
  else
    work.d[182] -= settings.kkt_reg;
  work.d_inv[182] = 1/work.d[182];
  work.L[131] = (-work.L[126]*work.v[118]-work.L[127]*work.v[178]-work.L[128]*work.v[179]-work.L[129]*work.v[180]-work.L[130]*work.v[181])*work.d_inv[182];
  work.L[337] = (-work.L[336]*work.v[124])*work.d_inv[182];
  work.L[434] = (-work.L[433]*work.v[181])*work.d_inv[182];
  work.L[479] = (-work.L[476]*work.v[179]-work.L[477]*work.v[180]-work.L[478]*work.v[181])*work.d_inv[182];
  work.v[118] = work.L[126]*work.d[118];
  work.v[178] = work.L[127]*work.d[178];
  work.v[179] = work.L[128]*work.d[179];
  work.v[180] = work.L[129]*work.d[180];
  work.v[181] = work.L[130]*work.d[181];
  work.v[182] = work.L[131]*work.d[182];
  work.v[183] = 0-work.L[126]*work.v[118]-work.L[127]*work.v[178]-work.L[128]*work.v[179]-work.L[129]*work.v[180]-work.L[130]*work.v[181]-work.L[131]*work.v[182];
  work.d[183] = work.v[183];
  if (work.d[183] > 0)
    work.d[183] = -settings.kkt_reg;
  else
    work.d[183] -= settings.kkt_reg;
  work.d_inv[183] = 1/work.d[183];
  work.L[254] = (work.KKT[372])*work.d_inv[183];
  work.L[338] = (-work.L[337]*work.v[182])*work.d_inv[183];
  work.L[435] = (-work.L[433]*work.v[181]-work.L[434]*work.v[182])*work.d_inv[183];
  work.L[480] = (-work.L[476]*work.v[179]-work.L[477]*work.v[180]-work.L[478]*work.v[181]-work.L[479]*work.v[182])*work.d_inv[183];
  work.v[184] = 0;
  work.d[184] = work.v[184];
  if (work.d[184] < 0)
    work.d[184] = settings.kkt_reg;
  else
    work.d[184] += settings.kkt_reg;
  work.d_inv[184] = 1/work.d[184];
  work.L[134] = (work.KKT[373])*work.d_inv[184];
  work.L[137] = (work.KKT[374])*work.d_inv[184];
  work.L[146] = (work.KKT[375])*work.d_inv[184];
  work.L[151] = (work.KKT[376])*work.d_inv[184];
  work.v[15] = work.L[132]*work.d[15];
  work.v[149] = work.L[133]*work.d[149];
  work.v[184] = work.L[134]*work.d[184];
  work.v[185] = work.KKT[377]-work.L[132]*work.v[15]-work.L[133]*work.v[149]-work.L[134]*work.v[184];
  work.d[185] = work.v[185];
  if (work.d[185] > 0)
    work.d[185] = -settings.kkt_reg;
  else
    work.d[185] -= settings.kkt_reg;
  work.d_inv[185] = 1/work.d[185];
  work.L[138] = (-work.L[136]*work.v[149]-work.L[137]*work.v[184])*work.d_inv[185];
  work.L[141] = (work.KKT[378])*work.d_inv[185];
  work.L[147] = (-work.L[146]*work.v[184])*work.d_inv[185];
  work.L[152] = (-work.L[151]*work.v[184])*work.d_inv[185];
  work.L[481] = (-work.L[471]*work.v[149])*work.d_inv[185];
  work.v[21] = work.L[135]*work.d[21];
  work.v[149] = work.L[136]*work.d[149];
  work.v[184] = work.L[137]*work.d[184];
  work.v[185] = work.L[138]*work.d[185];
  work.v[186] = work.KKT[379]-work.L[135]*work.v[21]-work.L[136]*work.v[149]-work.L[137]*work.v[184]-work.L[138]*work.v[185];
  work.d[186] = work.v[186];
  if (work.d[186] > 0)
    work.d[186] = -settings.kkt_reg;
  else
    work.d[186] -= settings.kkt_reg;
  work.d_inv[186] = 1/work.d[186];
  work.L[142] = (work.KKT[380]-work.L[141]*work.v[185])*work.d_inv[186];
  work.L[148] = (-work.L[146]*work.v[184]-work.L[147]*work.v[185])*work.d_inv[186];
  work.L[153] = (-work.L[151]*work.v[184]-work.L[152]*work.v[185])*work.d_inv[186];
  work.L[482] = (-work.L[471]*work.v[149]-work.L[481]*work.v[185])*work.d_inv[186];
  work.v[134] = work.L[139]*work.d[134];
  work.v[135] = work.L[140]*work.d[135];
  work.v[185] = work.L[141]*work.d[185];
  work.v[186] = work.L[142]*work.d[186];
  work.v[187] = 0-work.L[139]*work.v[134]-work.L[140]*work.v[135]-work.L[141]*work.v[185]-work.L[142]*work.v[186];
  work.d[187] = work.v[187];
  if (work.d[187] < 0)
    work.d[187] = settings.kkt_reg;
  else
    work.d[187] += settings.kkt_reg;
  work.d_inv[187] = 1/work.d[187];
  work.L[149] = (work.KKT[381]-work.L[147]*work.v[185]-work.L[148]*work.v[186])*work.d_inv[187];
  work.L[154] = (work.KKT[382]-work.L[152]*work.v[185]-work.L[153]*work.v[186])*work.d_inv[187];
  work.L[436] = (-work.L[419]*work.v[134]-work.L[420]*work.v[135])*work.d_inv[187];
  work.L[483] = (-work.L[481]*work.v[185]-work.L[482]*work.v[186])*work.d_inv[187];
  work.v[103] = work.L[143]*work.d[103];
  work.v[119] = work.L[144]*work.d[119];
  work.v[125] = work.L[145]*work.d[125];
  work.v[184] = work.L[146]*work.d[184];
  work.v[185] = work.L[147]*work.d[185];
  work.v[186] = work.L[148]*work.d[186];
  work.v[187] = work.L[149]*work.d[187];
  work.v[188] = 0-work.L[143]*work.v[103]-work.L[144]*work.v[119]-work.L[145]*work.v[125]-work.L[146]*work.v[184]-work.L[147]*work.v[185]-work.L[148]*work.v[186]-work.L[149]*work.v[187];
  work.d[188] = work.v[188];
  if (work.d[188] > 0)
    work.d[188] = -settings.kkt_reg;
  else
    work.d[188] -= settings.kkt_reg;
  work.d_inv[188] = 1/work.d[188];
  work.L[155] = (-work.L[150]*work.v[119]-work.L[151]*work.v[184]-work.L[152]*work.v[185]-work.L[153]*work.v[186]-work.L[154]*work.v[187])*work.d_inv[188];
  work.L[345] = (-work.L[344]*work.v[125])*work.d_inv[188];
  work.L[437] = (-work.L[436]*work.v[187])*work.d_inv[188];
  work.L[484] = (-work.L[481]*work.v[185]-work.L[482]*work.v[186]-work.L[483]*work.v[187])*work.d_inv[188];
  work.v[119] = work.L[150]*work.d[119];
  work.v[184] = work.L[151]*work.d[184];
  work.v[185] = work.L[152]*work.d[185];
  work.v[186] = work.L[153]*work.d[186];
  work.v[187] = work.L[154]*work.d[187];
  work.v[188] = work.L[155]*work.d[188];
  work.v[189] = 0-work.L[150]*work.v[119]-work.L[151]*work.v[184]-work.L[152]*work.v[185]-work.L[153]*work.v[186]-work.L[154]*work.v[187]-work.L[155]*work.v[188];
  work.d[189] = work.v[189];
  if (work.d[189] > 0)
    work.d[189] = -settings.kkt_reg;
  else
    work.d[189] -= settings.kkt_reg;
  work.d_inv[189] = 1/work.d[189];
  work.L[257] = (work.KKT[383])*work.d_inv[189];
  work.L[346] = (-work.L[345]*work.v[188])*work.d_inv[189];
  work.L[438] = (-work.L[436]*work.v[187]-work.L[437]*work.v[188])*work.d_inv[189];
  work.L[485] = (-work.L[481]*work.v[185]-work.L[482]*work.v[186]-work.L[483]*work.v[187]-work.L[484]*work.v[188])*work.d_inv[189];
  work.v[190] = 0;
  work.d[190] = work.v[190];
  if (work.d[190] < 0)
    work.d[190] = settings.kkt_reg;
  else
    work.d[190] += settings.kkt_reg;
  work.d_inv[190] = 1/work.d[190];
  work.L[158] = (work.KKT[384])*work.d_inv[190];
  work.L[161] = (work.KKT[385])*work.d_inv[190];
  work.L[170] = (work.KKT[386])*work.d_inv[190];
  work.L[175] = (work.KKT[387])*work.d_inv[190];
  work.v[16] = work.L[156]*work.d[16];
  work.v[150] = work.L[157]*work.d[150];
  work.v[190] = work.L[158]*work.d[190];
  work.v[191] = work.KKT[388]-work.L[156]*work.v[16]-work.L[157]*work.v[150]-work.L[158]*work.v[190];
  work.d[191] = work.v[191];
  if (work.d[191] > 0)
    work.d[191] = -settings.kkt_reg;
  else
    work.d[191] -= settings.kkt_reg;
  work.d_inv[191] = 1/work.d[191];
  work.L[162] = (-work.L[160]*work.v[150]-work.L[161]*work.v[190])*work.d_inv[191];
  work.L[165] = (work.KKT[389])*work.d_inv[191];
  work.L[171] = (-work.L[170]*work.v[190])*work.d_inv[191];
  work.L[176] = (-work.L[175]*work.v[190])*work.d_inv[191];
  work.L[486] = (-work.L[472]*work.v[150])*work.d_inv[191];
  work.v[22] = work.L[159]*work.d[22];
  work.v[150] = work.L[160]*work.d[150];
  work.v[190] = work.L[161]*work.d[190];
  work.v[191] = work.L[162]*work.d[191];
  work.v[192] = work.KKT[390]-work.L[159]*work.v[22]-work.L[160]*work.v[150]-work.L[161]*work.v[190]-work.L[162]*work.v[191];
  work.d[192] = work.v[192];
  if (work.d[192] > 0)
    work.d[192] = -settings.kkt_reg;
  else
    work.d[192] -= settings.kkt_reg;
  work.d_inv[192] = 1/work.d[192];
  work.L[166] = (work.KKT[391]-work.L[165]*work.v[191])*work.d_inv[192];
  work.L[172] = (-work.L[170]*work.v[190]-work.L[171]*work.v[191])*work.d_inv[192];
  work.L[177] = (-work.L[175]*work.v[190]-work.L[176]*work.v[191])*work.d_inv[192];
  work.L[487] = (-work.L[472]*work.v[150]-work.L[486]*work.v[191])*work.d_inv[192];
  work.v[137] = work.L[163]*work.d[137];
  work.v[138] = work.L[164]*work.d[138];
  work.v[191] = work.L[165]*work.d[191];
  work.v[192] = work.L[166]*work.d[192];
  work.v[193] = 0-work.L[163]*work.v[137]-work.L[164]*work.v[138]-work.L[165]*work.v[191]-work.L[166]*work.v[192];
  work.d[193] = work.v[193];
  if (work.d[193] < 0)
    work.d[193] = settings.kkt_reg;
  else
    work.d[193] += settings.kkt_reg;
  work.d_inv[193] = 1/work.d[193];
  work.L[173] = (work.KKT[392]-work.L[171]*work.v[191]-work.L[172]*work.v[192])*work.d_inv[193];
  work.L[178] = (work.KKT[393]-work.L[176]*work.v[191]-work.L[177]*work.v[192])*work.d_inv[193];
  work.L[439] = (-work.L[422]*work.v[137]-work.L[423]*work.v[138])*work.d_inv[193];
  work.L[488] = (-work.L[486]*work.v[191]-work.L[487]*work.v[192])*work.d_inv[193];
  work.v[105] = work.L[167]*work.d[105];
  work.v[120] = work.L[168]*work.d[120];
  work.v[126] = work.L[169]*work.d[126];
  work.v[190] = work.L[170]*work.d[190];
  work.v[191] = work.L[171]*work.d[191];
  work.v[192] = work.L[172]*work.d[192];
  work.v[193] = work.L[173]*work.d[193];
  work.v[194] = 0-work.L[167]*work.v[105]-work.L[168]*work.v[120]-work.L[169]*work.v[126]-work.L[170]*work.v[190]-work.L[171]*work.v[191]-work.L[172]*work.v[192]-work.L[173]*work.v[193];
  work.d[194] = work.v[194];
  if (work.d[194] > 0)
    work.d[194] = -settings.kkt_reg;
  else
    work.d[194] -= settings.kkt_reg;
  work.d_inv[194] = 1/work.d[194];
  work.L[179] = (-work.L[174]*work.v[120]-work.L[175]*work.v[190]-work.L[176]*work.v[191]-work.L[177]*work.v[192]-work.L[178]*work.v[193])*work.d_inv[194];
  work.L[353] = (-work.L[352]*work.v[126])*work.d_inv[194];
  work.L[440] = (-work.L[439]*work.v[193])*work.d_inv[194];
  work.L[489] = (-work.L[486]*work.v[191]-work.L[487]*work.v[192]-work.L[488]*work.v[193])*work.d_inv[194];
  work.v[120] = work.L[174]*work.d[120];
  work.v[190] = work.L[175]*work.d[190];
  work.v[191] = work.L[176]*work.d[191];
  work.v[192] = work.L[177]*work.d[192];
  work.v[193] = work.L[178]*work.d[193];
  work.v[194] = work.L[179]*work.d[194];
  work.v[195] = 0-work.L[174]*work.v[120]-work.L[175]*work.v[190]-work.L[176]*work.v[191]-work.L[177]*work.v[192]-work.L[178]*work.v[193]-work.L[179]*work.v[194];
  work.d[195] = work.v[195];
  if (work.d[195] > 0)
    work.d[195] = -settings.kkt_reg;
  else
    work.d[195] -= settings.kkt_reg;
  work.d_inv[195] = 1/work.d[195];
  work.L[260] = (work.KKT[394])*work.d_inv[195];
  work.L[354] = (-work.L[353]*work.v[194])*work.d_inv[195];
  work.L[441] = (-work.L[439]*work.v[193]-work.L[440]*work.v[194])*work.d_inv[195];
  work.L[490] = (-work.L[486]*work.v[191]-work.L[487]*work.v[192]-work.L[488]*work.v[193]-work.L[489]*work.v[194])*work.d_inv[195];
  work.v[196] = 0;
  work.d[196] = work.v[196];
  if (work.d[196] < 0)
    work.d[196] = settings.kkt_reg;
  else
    work.d[196] += settings.kkt_reg;
  work.d_inv[196] = 1/work.d[196];
  work.L[182] = (work.KKT[395])*work.d_inv[196];
  work.L[185] = (work.KKT[396])*work.d_inv[196];
  work.L[194] = (work.KKT[397])*work.d_inv[196];
  work.L[199] = (work.KKT[398])*work.d_inv[196];
  work.v[17] = work.L[180]*work.d[17];
  work.v[151] = work.L[181]*work.d[151];
  work.v[196] = work.L[182]*work.d[196];
  work.v[197] = work.KKT[399]-work.L[180]*work.v[17]-work.L[181]*work.v[151]-work.L[182]*work.v[196];
  work.d[197] = work.v[197];
  if (work.d[197] > 0)
    work.d[197] = -settings.kkt_reg;
  else
    work.d[197] -= settings.kkt_reg;
  work.d_inv[197] = 1/work.d[197];
  work.L[186] = (-work.L[184]*work.v[151]-work.L[185]*work.v[196])*work.d_inv[197];
  work.L[189] = (work.KKT[400])*work.d_inv[197];
  work.L[195] = (-work.L[194]*work.v[196])*work.d_inv[197];
  work.L[200] = (-work.L[199]*work.v[196])*work.d_inv[197];
  work.L[491] = (-work.L[473]*work.v[151])*work.d_inv[197];
  work.v[23] = work.L[183]*work.d[23];
  work.v[151] = work.L[184]*work.d[151];
  work.v[196] = work.L[185]*work.d[196];
  work.v[197] = work.L[186]*work.d[197];
  work.v[198] = work.KKT[401]-work.L[183]*work.v[23]-work.L[184]*work.v[151]-work.L[185]*work.v[196]-work.L[186]*work.v[197];
  work.d[198] = work.v[198];
  if (work.d[198] > 0)
    work.d[198] = -settings.kkt_reg;
  else
    work.d[198] -= settings.kkt_reg;
  work.d_inv[198] = 1/work.d[198];
  work.L[190] = (work.KKT[402]-work.L[189]*work.v[197])*work.d_inv[198];
  work.L[196] = (-work.L[194]*work.v[196]-work.L[195]*work.v[197])*work.d_inv[198];
  work.L[201] = (-work.L[199]*work.v[196]-work.L[200]*work.v[197])*work.d_inv[198];
  work.L[492] = (-work.L[473]*work.v[151]-work.L[491]*work.v[197])*work.d_inv[198];
  work.v[140] = work.L[187]*work.d[140];
  work.v[141] = work.L[188]*work.d[141];
  work.v[197] = work.L[189]*work.d[197];
  work.v[198] = work.L[190]*work.d[198];
  work.v[199] = 0-work.L[187]*work.v[140]-work.L[188]*work.v[141]-work.L[189]*work.v[197]-work.L[190]*work.v[198];
  work.d[199] = work.v[199];
  if (work.d[199] < 0)
    work.d[199] = settings.kkt_reg;
  else
    work.d[199] += settings.kkt_reg;
  work.d_inv[199] = 1/work.d[199];
  work.L[197] = (work.KKT[403]-work.L[195]*work.v[197]-work.L[196]*work.v[198])*work.d_inv[199];
  work.L[202] = (work.KKT[404]-work.L[200]*work.v[197]-work.L[201]*work.v[198])*work.d_inv[199];
  work.L[442] = (-work.L[425]*work.v[140]-work.L[426]*work.v[141])*work.d_inv[199];
  work.L[493] = (-work.L[491]*work.v[197]-work.L[492]*work.v[198])*work.d_inv[199];
  work.v[107] = work.L[191]*work.d[107];
  work.v[121] = work.L[192]*work.d[121];
  work.v[127] = work.L[193]*work.d[127];
  work.v[196] = work.L[194]*work.d[196];
  work.v[197] = work.L[195]*work.d[197];
  work.v[198] = work.L[196]*work.d[198];
  work.v[199] = work.L[197]*work.d[199];
  work.v[200] = 0-work.L[191]*work.v[107]-work.L[192]*work.v[121]-work.L[193]*work.v[127]-work.L[194]*work.v[196]-work.L[195]*work.v[197]-work.L[196]*work.v[198]-work.L[197]*work.v[199];
  work.d[200] = work.v[200];
  if (work.d[200] > 0)
    work.d[200] = -settings.kkt_reg;
  else
    work.d[200] -= settings.kkt_reg;
  work.d_inv[200] = 1/work.d[200];
  work.L[203] = (-work.L[198]*work.v[121]-work.L[199]*work.v[196]-work.L[200]*work.v[197]-work.L[201]*work.v[198]-work.L[202]*work.v[199])*work.d_inv[200];
  work.L[361] = (-work.L[360]*work.v[127])*work.d_inv[200];
  work.L[443] = (-work.L[442]*work.v[199])*work.d_inv[200];
  work.L[494] = (-work.L[491]*work.v[197]-work.L[492]*work.v[198]-work.L[493]*work.v[199])*work.d_inv[200];
  work.v[121] = work.L[198]*work.d[121];
  work.v[196] = work.L[199]*work.d[196];
  work.v[197] = work.L[200]*work.d[197];
  work.v[198] = work.L[201]*work.d[198];
  work.v[199] = work.L[202]*work.d[199];
  work.v[200] = work.L[203]*work.d[200];
  work.v[201] = 0-work.L[198]*work.v[121]-work.L[199]*work.v[196]-work.L[200]*work.v[197]-work.L[201]*work.v[198]-work.L[202]*work.v[199]-work.L[203]*work.v[200];
  work.d[201] = work.v[201];
  if (work.d[201] > 0)
    work.d[201] = -settings.kkt_reg;
  else
    work.d[201] -= settings.kkt_reg;
  work.d_inv[201] = 1/work.d[201];
  work.L[263] = (work.KKT[405])*work.d_inv[201];
  work.L[362] = (-work.L[361]*work.v[200])*work.d_inv[201];
  work.L[444] = (-work.L[442]*work.v[199]-work.L[443]*work.v[200])*work.d_inv[201];
  work.L[495] = (-work.L[491]*work.v[197]-work.L[492]*work.v[198]-work.L[493]*work.v[199]-work.L[494]*work.v[200])*work.d_inv[201];
  work.v[202] = 0;
  work.d[202] = work.v[202];
  if (work.d[202] < 0)
    work.d[202] = settings.kkt_reg;
  else
    work.d[202] += settings.kkt_reg;
  work.d_inv[202] = 1/work.d[202];
  work.L[206] = (work.KKT[406])*work.d_inv[202];
  work.L[209] = (work.KKT[407])*work.d_inv[202];
  work.L[218] = (work.KKT[408])*work.d_inv[202];
  work.L[223] = (work.KKT[409])*work.d_inv[202];
  work.v[18] = work.L[204]*work.d[18];
  work.v[152] = work.L[205]*work.d[152];
  work.v[202] = work.L[206]*work.d[202];
  work.v[203] = work.KKT[410]-work.L[204]*work.v[18]-work.L[205]*work.v[152]-work.L[206]*work.v[202];
  work.d[203] = work.v[203];
  if (work.d[203] > 0)
    work.d[203] = -settings.kkt_reg;
  else
    work.d[203] -= settings.kkt_reg;
  work.d_inv[203] = 1/work.d[203];
  work.L[210] = (-work.L[208]*work.v[152]-work.L[209]*work.v[202])*work.d_inv[203];
  work.L[213] = (work.KKT[411])*work.d_inv[203];
  work.L[219] = (-work.L[218]*work.v[202])*work.d_inv[203];
  work.L[224] = (-work.L[223]*work.v[202])*work.d_inv[203];
  work.L[496] = (-work.L[474]*work.v[152])*work.d_inv[203];
  work.v[24] = work.L[207]*work.d[24];
  work.v[152] = work.L[208]*work.d[152];
  work.v[202] = work.L[209]*work.d[202];
  work.v[203] = work.L[210]*work.d[203];
  work.v[204] = work.KKT[412]-work.L[207]*work.v[24]-work.L[208]*work.v[152]-work.L[209]*work.v[202]-work.L[210]*work.v[203];
  work.d[204] = work.v[204];
  if (work.d[204] > 0)
    work.d[204] = -settings.kkt_reg;
  else
    work.d[204] -= settings.kkt_reg;
  work.d_inv[204] = 1/work.d[204];
  work.L[214] = (work.KKT[413]-work.L[213]*work.v[203])*work.d_inv[204];
  work.L[220] = (-work.L[218]*work.v[202]-work.L[219]*work.v[203])*work.d_inv[204];
  work.L[225] = (-work.L[223]*work.v[202]-work.L[224]*work.v[203])*work.d_inv[204];
  work.L[497] = (-work.L[474]*work.v[152]-work.L[496]*work.v[203])*work.d_inv[204];
  work.v[143] = work.L[211]*work.d[143];
  work.v[144] = work.L[212]*work.d[144];
  work.v[203] = work.L[213]*work.d[203];
  work.v[204] = work.L[214]*work.d[204];
  work.v[205] = 0-work.L[211]*work.v[143]-work.L[212]*work.v[144]-work.L[213]*work.v[203]-work.L[214]*work.v[204];
  work.d[205] = work.v[205];
  if (work.d[205] < 0)
    work.d[205] = settings.kkt_reg;
  else
    work.d[205] += settings.kkt_reg;
  work.d_inv[205] = 1/work.d[205];
  work.L[221] = (work.KKT[414]-work.L[219]*work.v[203]-work.L[220]*work.v[204])*work.d_inv[205];
  work.L[226] = (work.KKT[415]-work.L[224]*work.v[203]-work.L[225]*work.v[204])*work.d_inv[205];
  work.L[445] = (-work.L[428]*work.v[143]-work.L[429]*work.v[144])*work.d_inv[205];
  work.L[498] = (-work.L[496]*work.v[203]-work.L[497]*work.v[204])*work.d_inv[205];
  work.v[109] = work.L[215]*work.d[109];
  work.v[122] = work.L[216]*work.d[122];
  work.v[128] = work.L[217]*work.d[128];
  work.v[202] = work.L[218]*work.d[202];
  work.v[203] = work.L[219]*work.d[203];
  work.v[204] = work.L[220]*work.d[204];
  work.v[205] = work.L[221]*work.d[205];
  work.v[206] = 0-work.L[215]*work.v[109]-work.L[216]*work.v[122]-work.L[217]*work.v[128]-work.L[218]*work.v[202]-work.L[219]*work.v[203]-work.L[220]*work.v[204]-work.L[221]*work.v[205];
  work.d[206] = work.v[206];
  if (work.d[206] > 0)
    work.d[206] = -settings.kkt_reg;
  else
    work.d[206] -= settings.kkt_reg;
  work.d_inv[206] = 1/work.d[206];
  work.L[227] = (-work.L[222]*work.v[122]-work.L[223]*work.v[202]-work.L[224]*work.v[203]-work.L[225]*work.v[204]-work.L[226]*work.v[205])*work.d_inv[206];
  work.L[369] = (-work.L[368]*work.v[128])*work.d_inv[206];
  work.L[446] = (-work.L[445]*work.v[205])*work.d_inv[206];
  work.L[499] = (-work.L[496]*work.v[203]-work.L[497]*work.v[204]-work.L[498]*work.v[205])*work.d_inv[206];
  work.v[122] = work.L[222]*work.d[122];
  work.v[202] = work.L[223]*work.d[202];
  work.v[203] = work.L[224]*work.d[203];
  work.v[204] = work.L[225]*work.d[204];
  work.v[205] = work.L[226]*work.d[205];
  work.v[206] = work.L[227]*work.d[206];
  work.v[207] = 0-work.L[222]*work.v[122]-work.L[223]*work.v[202]-work.L[224]*work.v[203]-work.L[225]*work.v[204]-work.L[226]*work.v[205]-work.L[227]*work.v[206];
  work.d[207] = work.v[207];
  if (work.d[207] > 0)
    work.d[207] = -settings.kkt_reg;
  else
    work.d[207] -= settings.kkt_reg;
  work.d_inv[207] = 1/work.d[207];
  work.L[266] = (work.KKT[416])*work.d_inv[207];
  work.L[370] = (-work.L[369]*work.v[206])*work.d_inv[207];
  work.L[447] = (-work.L[445]*work.v[205]-work.L[446]*work.v[206])*work.d_inv[207];
  work.L[500] = (-work.L[496]*work.v[203]-work.L[497]*work.v[204]-work.L[498]*work.v[205]-work.L[499]*work.v[206])*work.d_inv[207];
  work.v[208] = 0;
  work.d[208] = work.v[208];
  if (work.d[208] < 0)
    work.d[208] = settings.kkt_reg;
  else
    work.d[208] += settings.kkt_reg;
  work.d_inv[208] = 1/work.d[208];
  work.L[230] = (work.KKT[417])*work.d_inv[208];
  work.L[233] = (work.KKT[418])*work.d_inv[208];
  work.L[242] = (work.KKT[419])*work.d_inv[208];
  work.L[247] = (work.KKT[420])*work.d_inv[208];
  work.v[19] = work.L[228]*work.d[19];
  work.v[153] = work.L[229]*work.d[153];
  work.v[208] = work.L[230]*work.d[208];
  work.v[209] = work.KKT[421]-work.L[228]*work.v[19]-work.L[229]*work.v[153]-work.L[230]*work.v[208];
  work.d[209] = work.v[209];
  if (work.d[209] > 0)
    work.d[209] = -settings.kkt_reg;
  else
    work.d[209] -= settings.kkt_reg;
  work.d_inv[209] = 1/work.d[209];
  work.L[234] = (-work.L[232]*work.v[153]-work.L[233]*work.v[208])*work.d_inv[209];
  work.L[237] = (work.KKT[422])*work.d_inv[209];
  work.L[243] = (-work.L[242]*work.v[208])*work.d_inv[209];
  work.L[248] = (-work.L[247]*work.v[208])*work.d_inv[209];
  work.L[501] = (-work.L[475]*work.v[153])*work.d_inv[209];
  work.v[25] = work.L[231]*work.d[25];
  work.v[153] = work.L[232]*work.d[153];
  work.v[208] = work.L[233]*work.d[208];
  work.v[209] = work.L[234]*work.d[209];
  work.v[210] = work.KKT[423]-work.L[231]*work.v[25]-work.L[232]*work.v[153]-work.L[233]*work.v[208]-work.L[234]*work.v[209];
  work.d[210] = work.v[210];
  if (work.d[210] > 0)
    work.d[210] = -settings.kkt_reg;
  else
    work.d[210] -= settings.kkt_reg;
  work.d_inv[210] = 1/work.d[210];
  work.L[238] = (work.KKT[424]-work.L[237]*work.v[209])*work.d_inv[210];
  work.L[244] = (-work.L[242]*work.v[208]-work.L[243]*work.v[209])*work.d_inv[210];
  work.L[249] = (-work.L[247]*work.v[208]-work.L[248]*work.v[209])*work.d_inv[210];
  work.L[502] = (-work.L[475]*work.v[153]-work.L[501]*work.v[209])*work.d_inv[210];
  work.v[146] = work.L[235]*work.d[146];
  work.v[147] = work.L[236]*work.d[147];
  work.v[209] = work.L[237]*work.d[209];
  work.v[210] = work.L[238]*work.d[210];
  work.v[211] = 0-work.L[235]*work.v[146]-work.L[236]*work.v[147]-work.L[237]*work.v[209]-work.L[238]*work.v[210];
  work.d[211] = work.v[211];
  if (work.d[211] < 0)
    work.d[211] = settings.kkt_reg;
  else
    work.d[211] += settings.kkt_reg;
  work.d_inv[211] = 1/work.d[211];
  work.L[245] = (work.KKT[425]-work.L[243]*work.v[209]-work.L[244]*work.v[210])*work.d_inv[211];
  work.L[250] = (work.KKT[426]-work.L[248]*work.v[209]-work.L[249]*work.v[210])*work.d_inv[211];
  work.L[448] = (-work.L[431]*work.v[146]-work.L[432]*work.v[147])*work.d_inv[211];
  work.L[503] = (-work.L[501]*work.v[209]-work.L[502]*work.v[210])*work.d_inv[211];
  work.v[111] = work.L[239]*work.d[111];
  work.v[123] = work.L[240]*work.d[123];
  work.v[129] = work.L[241]*work.d[129];
  work.v[208] = work.L[242]*work.d[208];
  work.v[209] = work.L[243]*work.d[209];
  work.v[210] = work.L[244]*work.d[210];
  work.v[211] = work.L[245]*work.d[211];
  work.v[212] = 0-work.L[239]*work.v[111]-work.L[240]*work.v[123]-work.L[241]*work.v[129]-work.L[242]*work.v[208]-work.L[243]*work.v[209]-work.L[244]*work.v[210]-work.L[245]*work.v[211];
  work.d[212] = work.v[212];
  if (work.d[212] > 0)
    work.d[212] = -settings.kkt_reg;
  else
    work.d[212] -= settings.kkt_reg;
  work.d_inv[212] = 1/work.d[212];
  work.L[251] = (-work.L[246]*work.v[123]-work.L[247]*work.v[208]-work.L[248]*work.v[209]-work.L[249]*work.v[210]-work.L[250]*work.v[211])*work.d_inv[212];
  work.L[377] = (-work.L[376]*work.v[129])*work.d_inv[212];
  work.L[449] = (-work.L[448]*work.v[211])*work.d_inv[212];
  work.L[504] = (-work.L[501]*work.v[209]-work.L[502]*work.v[210]-work.L[503]*work.v[211])*work.d_inv[212];
  work.v[123] = work.L[246]*work.d[123];
  work.v[208] = work.L[247]*work.d[208];
  work.v[209] = work.L[248]*work.d[209];
  work.v[210] = work.L[249]*work.d[210];
  work.v[211] = work.L[250]*work.d[211];
  work.v[212] = work.L[251]*work.d[212];
  work.v[213] = 0-work.L[246]*work.v[123]-work.L[247]*work.v[208]-work.L[248]*work.v[209]-work.L[249]*work.v[210]-work.L[250]*work.v[211]-work.L[251]*work.v[212];
  work.d[213] = work.v[213];
  if (work.d[213] > 0)
    work.d[213] = -settings.kkt_reg;
  else
    work.d[213] -= settings.kkt_reg;
  work.d_inv[213] = 1/work.d[213];
  work.L[269] = (work.KKT[427])*work.d_inv[213];
  work.L[378] = (-work.L[377]*work.v[212])*work.d_inv[213];
  work.L[450] = (-work.L[448]*work.v[211]-work.L[449]*work.v[212])*work.d_inv[213];
  work.L[505] = (-work.L[501]*work.v[209]-work.L[502]*work.v[210]-work.L[503]*work.v[211]-work.L[504]*work.v[212])*work.d_inv[213];
  work.v[88] = work.L[252]*work.d[88];
  work.v[89] = work.L[253]*work.d[89];
  work.v[183] = work.L[254]*work.d[183];
  work.v[214] = 0-work.L[252]*work.v[88]-work.L[253]*work.v[89]-work.L[254]*work.v[183];
  work.d[214] = work.v[214];
  if (work.d[214] < 0)
    work.d[214] = settings.kkt_reg;
  else
    work.d[214] += settings.kkt_reg;
  work.d_inv[214] = 1/work.d[214];
  work.L[339] = (work.KKT[428]-work.L[338]*work.v[183])*work.d_inv[214];
  work.L[384] = (work.KKT[429])*work.d_inv[214];
  work.L[451] = (-work.L[435]*work.v[183])*work.d_inv[214];
  work.L[506] = (-work.L[480]*work.v[183])*work.d_inv[214];
  work.v[90] = work.L[255]*work.d[90];
  work.v[91] = work.L[256]*work.d[91];
  work.v[189] = work.L[257]*work.d[189];
  work.v[215] = 0-work.L[255]*work.v[90]-work.L[256]*work.v[91]-work.L[257]*work.v[189];
  work.d[215] = work.v[215];
  if (work.d[215] < 0)
    work.d[215] = settings.kkt_reg;
  else
    work.d[215] += settings.kkt_reg;
  work.d_inv[215] = 1/work.d[215];
  work.L[347] = (work.KKT[430]-work.L[346]*work.v[189])*work.d_inv[215];
  work.L[390] = (work.KKT[431])*work.d_inv[215];
  work.L[452] = (-work.L[438]*work.v[189])*work.d_inv[215];
  work.L[507] = (-work.L[485]*work.v[189])*work.d_inv[215];
  work.v[92] = work.L[258]*work.d[92];
  work.v[93] = work.L[259]*work.d[93];
  work.v[195] = work.L[260]*work.d[195];
  work.v[216] = 0-work.L[258]*work.v[92]-work.L[259]*work.v[93]-work.L[260]*work.v[195];
  work.d[216] = work.v[216];
  if (work.d[216] < 0)
    work.d[216] = settings.kkt_reg;
  else
    work.d[216] += settings.kkt_reg;
  work.d_inv[216] = 1/work.d[216];
  work.L[355] = (work.KKT[432]-work.L[354]*work.v[195])*work.d_inv[216];
  work.L[396] = (work.KKT[433])*work.d_inv[216];
  work.L[453] = (-work.L[441]*work.v[195])*work.d_inv[216];
  work.L[508] = (-work.L[490]*work.v[195])*work.d_inv[216];
  work.v[94] = work.L[261]*work.d[94];
  work.v[95] = work.L[262]*work.d[95];
  work.v[201] = work.L[263]*work.d[201];
  work.v[217] = 0-work.L[261]*work.v[94]-work.L[262]*work.v[95]-work.L[263]*work.v[201];
  work.d[217] = work.v[217];
  if (work.d[217] < 0)
    work.d[217] = settings.kkt_reg;
  else
    work.d[217] += settings.kkt_reg;
  work.d_inv[217] = 1/work.d[217];
  work.L[363] = (work.KKT[434]-work.L[362]*work.v[201])*work.d_inv[217];
  work.L[402] = (work.KKT[435])*work.d_inv[217];
  work.L[454] = (-work.L[444]*work.v[201])*work.d_inv[217];
  work.L[509] = (-work.L[495]*work.v[201])*work.d_inv[217];
  work.v[96] = work.L[264]*work.d[96];
  work.v[97] = work.L[265]*work.d[97];
  work.v[207] = work.L[266]*work.d[207];
  work.v[218] = 0-work.L[264]*work.v[96]-work.L[265]*work.v[97]-work.L[266]*work.v[207];
  work.d[218] = work.v[218];
  if (work.d[218] < 0)
    work.d[218] = settings.kkt_reg;
  else
    work.d[218] += settings.kkt_reg;
  work.d_inv[218] = 1/work.d[218];
  work.L[371] = (work.KKT[436]-work.L[370]*work.v[207])*work.d_inv[218];
  work.L[408] = (work.KKT[437])*work.d_inv[218];
  work.L[455] = (-work.L[447]*work.v[207])*work.d_inv[218];
  work.L[510] = (-work.L[500]*work.v[207])*work.d_inv[218];
  work.v[98] = work.L[267]*work.d[98];
  work.v[99] = work.L[268]*work.d[99];
  work.v[213] = work.L[269]*work.d[213];
  work.v[219] = 0-work.L[267]*work.v[98]-work.L[268]*work.v[99]-work.L[269]*work.v[213];
  work.d[219] = work.v[219];
  if (work.d[219] < 0)
    work.d[219] = settings.kkt_reg;
  else
    work.d[219] += settings.kkt_reg;
  work.d_inv[219] = 1/work.d[219];
  work.L[379] = (work.KKT[438]-work.L[378]*work.v[213])*work.d_inv[219];
  work.L[456] = (-work.L[450]*work.v[213])*work.d_inv[219];
  work.L[511] = (-work.L[505]*work.v[213])*work.d_inv[219];
  work.L[604] = (work.KKT[439])*work.d_inv[219];
  work.v[220] = 0;
  work.d[220] = work.v[220];
  if (work.d[220] < 0)
    work.d[220] = settings.kkt_reg;
  else
    work.d[220] += settings.kkt_reg;
  work.d_inv[220] = 1/work.d[220];
  work.L[272] = (work.KKT[440])*work.d_inv[220];
  work.L[275] = (work.KKT[441])*work.d_inv[220];
  work.L[340] = (work.KKT[442])*work.d_inv[220];
  work.L[385] = (work.KKT[443])*work.d_inv[220];
  work.v[40] = work.L[270]*work.d[40];
  work.v[172] = work.L[271]*work.d[172];
  work.v[220] = work.L[272]*work.d[220];
  work.v[221] = work.KKT[444]-work.L[270]*work.v[40]-work.L[271]*work.v[172]-work.L[272]*work.v[220];
  work.d[221] = work.v[221];
  if (work.d[221] > 0)
    work.d[221] = -settings.kkt_reg;
  else
    work.d[221] -= settings.kkt_reg;
  work.d_inv[221] = 1/work.d[221];
  work.L[276] = (-work.L[274]*work.v[172]-work.L[275]*work.v[220])*work.d_inv[221];
  work.L[279] = (work.KKT[445])*work.d_inv[221];
  work.L[341] = (-work.L[340]*work.v[220])*work.d_inv[221];
  work.L[386] = (-work.L[385]*work.v[220])*work.d_inv[221];
  work.L[571] = (-work.L[565]*work.v[172])*work.d_inv[221];
  work.v[46] = work.L[273]*work.d[46];
  work.v[172] = work.L[274]*work.d[172];
  work.v[220] = work.L[275]*work.d[220];
  work.v[221] = work.L[276]*work.d[221];
  work.v[222] = work.KKT[446]-work.L[273]*work.v[46]-work.L[274]*work.v[172]-work.L[275]*work.v[220]-work.L[276]*work.v[221];
  work.d[222] = work.v[222];
  if (work.d[222] > 0)
    work.d[222] = -settings.kkt_reg;
  else
    work.d[222] -= settings.kkt_reg;
  work.d_inv[222] = 1/work.d[222];
  work.L[280] = (work.KKT[447]-work.L[279]*work.v[221])*work.d_inv[222];
  work.L[342] = (-work.L[340]*work.v[220]-work.L[341]*work.v[221])*work.d_inv[222];
  work.L[387] = (-work.L[385]*work.v[220]-work.L[386]*work.v[221])*work.d_inv[222];
  work.L[572] = (-work.L[565]*work.v[172]-work.L[571]*work.v[221])*work.d_inv[222];
  work.v[155] = work.L[277]*work.d[155];
  work.v[156] = work.L[278]*work.d[156];
  work.v[221] = work.L[279]*work.d[221];
  work.v[222] = work.L[280]*work.d[222];
  work.v[223] = 0-work.L[277]*work.v[155]-work.L[278]*work.v[156]-work.L[279]*work.v[221]-work.L[280]*work.v[222];
  work.d[223] = work.v[223];
  if (work.d[223] < 0)
    work.d[223] = settings.kkt_reg;
  else
    work.d[223] += settings.kkt_reg;
  work.d_inv[223] = 1/work.d[223];
  work.L[343] = (work.KKT[448]-work.L[341]*work.v[221]-work.L[342]*work.v[222])*work.d_inv[223];
  work.L[388] = (work.KKT[449]-work.L[386]*work.v[221]-work.L[387]*work.v[222])*work.d_inv[223];
  work.L[544] = (-work.L[527]*work.v[155]-work.L[528]*work.v[156])*work.d_inv[223];
  work.L[573] = (-work.L[571]*work.v[221]-work.L[572]*work.v[222])*work.d_inv[223];
  work.v[224] = 0;
  work.d[224] = work.v[224];
  if (work.d[224] < 0)
    work.d[224] = settings.kkt_reg;
  else
    work.d[224] += settings.kkt_reg;
  work.d_inv[224] = 1/work.d[224];
  work.L[283] = (work.KKT[450])*work.d_inv[224];
  work.L[286] = (work.KKT[451])*work.d_inv[224];
  work.L[348] = (work.KKT[452])*work.d_inv[224];
  work.L[391] = (work.KKT[453])*work.d_inv[224];
  work.v[41] = work.L[281]*work.d[41];
  work.v[173] = work.L[282]*work.d[173];
  work.v[224] = work.L[283]*work.d[224];
  work.v[225] = work.KKT[454]-work.L[281]*work.v[41]-work.L[282]*work.v[173]-work.L[283]*work.v[224];
  work.d[225] = work.v[225];
  if (work.d[225] > 0)
    work.d[225] = -settings.kkt_reg;
  else
    work.d[225] -= settings.kkt_reg;
  work.d_inv[225] = 1/work.d[225];
  work.L[287] = (-work.L[285]*work.v[173]-work.L[286]*work.v[224])*work.d_inv[225];
  work.L[290] = (work.KKT[455])*work.d_inv[225];
  work.L[349] = (-work.L[348]*work.v[224])*work.d_inv[225];
  work.L[392] = (-work.L[391]*work.v[224])*work.d_inv[225];
  work.L[574] = (-work.L[566]*work.v[173])*work.d_inv[225];
  work.v[47] = work.L[284]*work.d[47];
  work.v[173] = work.L[285]*work.d[173];
  work.v[224] = work.L[286]*work.d[224];
  work.v[225] = work.L[287]*work.d[225];
  work.v[226] = work.KKT[456]-work.L[284]*work.v[47]-work.L[285]*work.v[173]-work.L[286]*work.v[224]-work.L[287]*work.v[225];
  work.d[226] = work.v[226];
  if (work.d[226] > 0)
    work.d[226] = -settings.kkt_reg;
  else
    work.d[226] -= settings.kkt_reg;
  work.d_inv[226] = 1/work.d[226];
  work.L[291] = (work.KKT[457]-work.L[290]*work.v[225])*work.d_inv[226];
  work.L[350] = (-work.L[348]*work.v[224]-work.L[349]*work.v[225])*work.d_inv[226];
  work.L[393] = (-work.L[391]*work.v[224]-work.L[392]*work.v[225])*work.d_inv[226];
  work.L[575] = (-work.L[566]*work.v[173]-work.L[574]*work.v[225])*work.d_inv[226];
  work.v[158] = work.L[288]*work.d[158];
  work.v[159] = work.L[289]*work.d[159];
  work.v[225] = work.L[290]*work.d[225];
  work.v[226] = work.L[291]*work.d[226];
  work.v[227] = 0-work.L[288]*work.v[158]-work.L[289]*work.v[159]-work.L[290]*work.v[225]-work.L[291]*work.v[226];
  work.d[227] = work.v[227];
  if (work.d[227] < 0)
    work.d[227] = settings.kkt_reg;
  else
    work.d[227] += settings.kkt_reg;
  work.d_inv[227] = 1/work.d[227];
  work.L[351] = (work.KKT[458]-work.L[349]*work.v[225]-work.L[350]*work.v[226])*work.d_inv[227];
  work.L[394] = (work.KKT[459]-work.L[392]*work.v[225]-work.L[393]*work.v[226])*work.d_inv[227];
  work.L[545] = (-work.L[530]*work.v[158]-work.L[531]*work.v[159])*work.d_inv[227];
  work.L[576] = (-work.L[574]*work.v[225]-work.L[575]*work.v[226])*work.d_inv[227];
  work.v[228] = 0;
  work.d[228] = work.v[228];
  if (work.d[228] < 0)
    work.d[228] = settings.kkt_reg;
  else
    work.d[228] += settings.kkt_reg;
  work.d_inv[228] = 1/work.d[228];
  work.L[294] = (work.KKT[460])*work.d_inv[228];
  work.L[297] = (work.KKT[461])*work.d_inv[228];
  work.L[356] = (work.KKT[462])*work.d_inv[228];
  work.L[397] = (work.KKT[463])*work.d_inv[228];
  work.v[42] = work.L[292]*work.d[42];
  work.v[174] = work.L[293]*work.d[174];
  work.v[228] = work.L[294]*work.d[228];
  work.v[229] = work.KKT[464]-work.L[292]*work.v[42]-work.L[293]*work.v[174]-work.L[294]*work.v[228];
  work.d[229] = work.v[229];
  if (work.d[229] > 0)
    work.d[229] = -settings.kkt_reg;
  else
    work.d[229] -= settings.kkt_reg;
  work.d_inv[229] = 1/work.d[229];
  work.L[298] = (-work.L[296]*work.v[174]-work.L[297]*work.v[228])*work.d_inv[229];
  work.L[301] = (work.KKT[465])*work.d_inv[229];
  work.L[357] = (-work.L[356]*work.v[228])*work.d_inv[229];
  work.L[398] = (-work.L[397]*work.v[228])*work.d_inv[229];
  work.L[577] = (-work.L[567]*work.v[174])*work.d_inv[229];
  work.v[48] = work.L[295]*work.d[48];
  work.v[174] = work.L[296]*work.d[174];
  work.v[228] = work.L[297]*work.d[228];
  work.v[229] = work.L[298]*work.d[229];
  work.v[230] = work.KKT[466]-work.L[295]*work.v[48]-work.L[296]*work.v[174]-work.L[297]*work.v[228]-work.L[298]*work.v[229];
  work.d[230] = work.v[230];
  if (work.d[230] > 0)
    work.d[230] = -settings.kkt_reg;
  else
    work.d[230] -= settings.kkt_reg;
  work.d_inv[230] = 1/work.d[230];
  work.L[302] = (work.KKT[467]-work.L[301]*work.v[229])*work.d_inv[230];
  work.L[358] = (-work.L[356]*work.v[228]-work.L[357]*work.v[229])*work.d_inv[230];
  work.L[399] = (-work.L[397]*work.v[228]-work.L[398]*work.v[229])*work.d_inv[230];
  work.L[578] = (-work.L[567]*work.v[174]-work.L[577]*work.v[229])*work.d_inv[230];
  work.v[161] = work.L[299]*work.d[161];
  work.v[162] = work.L[300]*work.d[162];
  work.v[229] = work.L[301]*work.d[229];
  work.v[230] = work.L[302]*work.d[230];
  work.v[231] = 0-work.L[299]*work.v[161]-work.L[300]*work.v[162]-work.L[301]*work.v[229]-work.L[302]*work.v[230];
  work.d[231] = work.v[231];
  if (work.d[231] < 0)
    work.d[231] = settings.kkt_reg;
  else
    work.d[231] += settings.kkt_reg;
  work.d_inv[231] = 1/work.d[231];
  work.L[359] = (work.KKT[468]-work.L[357]*work.v[229]-work.L[358]*work.v[230])*work.d_inv[231];
  work.L[400] = (work.KKT[469]-work.L[398]*work.v[229]-work.L[399]*work.v[230])*work.d_inv[231];
  work.L[546] = (-work.L[533]*work.v[161]-work.L[534]*work.v[162])*work.d_inv[231];
  work.L[579] = (-work.L[577]*work.v[229]-work.L[578]*work.v[230])*work.d_inv[231];
  work.v[232] = 0;
  work.d[232] = work.v[232];
  if (work.d[232] < 0)
    work.d[232] = settings.kkt_reg;
  else
    work.d[232] += settings.kkt_reg;
  work.d_inv[232] = 1/work.d[232];
  work.L[305] = (work.KKT[470])*work.d_inv[232];
  work.L[308] = (work.KKT[471])*work.d_inv[232];
  work.L[364] = (work.KKT[472])*work.d_inv[232];
  work.L[403] = (work.KKT[473])*work.d_inv[232];
  work.v[43] = work.L[303]*work.d[43];
  work.v[175] = work.L[304]*work.d[175];
  work.v[232] = work.L[305]*work.d[232];
  work.v[233] = work.KKT[474]-work.L[303]*work.v[43]-work.L[304]*work.v[175]-work.L[305]*work.v[232];
  work.d[233] = work.v[233];
  if (work.d[233] > 0)
    work.d[233] = -settings.kkt_reg;
  else
    work.d[233] -= settings.kkt_reg;
  work.d_inv[233] = 1/work.d[233];
  work.L[309] = (-work.L[307]*work.v[175]-work.L[308]*work.v[232])*work.d_inv[233];
  work.L[312] = (work.KKT[475])*work.d_inv[233];
  work.L[365] = (-work.L[364]*work.v[232])*work.d_inv[233];
  work.L[404] = (-work.L[403]*work.v[232])*work.d_inv[233];
  work.L[580] = (-work.L[568]*work.v[175])*work.d_inv[233];
  work.v[49] = work.L[306]*work.d[49];
  work.v[175] = work.L[307]*work.d[175];
  work.v[232] = work.L[308]*work.d[232];
  work.v[233] = work.L[309]*work.d[233];
  work.v[234] = work.KKT[476]-work.L[306]*work.v[49]-work.L[307]*work.v[175]-work.L[308]*work.v[232]-work.L[309]*work.v[233];
  work.d[234] = work.v[234];
  if (work.d[234] > 0)
    work.d[234] = -settings.kkt_reg;
  else
    work.d[234] -= settings.kkt_reg;
  work.d_inv[234] = 1/work.d[234];
  work.L[313] = (work.KKT[477]-work.L[312]*work.v[233])*work.d_inv[234];
  work.L[366] = (-work.L[364]*work.v[232]-work.L[365]*work.v[233])*work.d_inv[234];
  work.L[405] = (-work.L[403]*work.v[232]-work.L[404]*work.v[233])*work.d_inv[234];
  work.L[581] = (-work.L[568]*work.v[175]-work.L[580]*work.v[233])*work.d_inv[234];
  work.v[164] = work.L[310]*work.d[164];
  work.v[165] = work.L[311]*work.d[165];
  work.v[233] = work.L[312]*work.d[233];
  work.v[234] = work.L[313]*work.d[234];
  work.v[235] = 0-work.L[310]*work.v[164]-work.L[311]*work.v[165]-work.L[312]*work.v[233]-work.L[313]*work.v[234];
  work.d[235] = work.v[235];
  if (work.d[235] < 0)
    work.d[235] = settings.kkt_reg;
  else
    work.d[235] += settings.kkt_reg;
  work.d_inv[235] = 1/work.d[235];
  work.L[367] = (work.KKT[478]-work.L[365]*work.v[233]-work.L[366]*work.v[234])*work.d_inv[235];
  work.L[406] = (work.KKT[479]-work.L[404]*work.v[233]-work.L[405]*work.v[234])*work.d_inv[235];
  work.L[547] = (-work.L[536]*work.v[164]-work.L[537]*work.v[165])*work.d_inv[235];
  work.L[582] = (-work.L[580]*work.v[233]-work.L[581]*work.v[234])*work.d_inv[235];
  work.v[236] = 0;
  work.d[236] = work.v[236];
  if (work.d[236] < 0)
    work.d[236] = settings.kkt_reg;
  else
    work.d[236] += settings.kkt_reg;
  work.d_inv[236] = 1/work.d[236];
  work.L[316] = (work.KKT[480])*work.d_inv[236];
  work.L[319] = (work.KKT[481])*work.d_inv[236];
  work.L[372] = (work.KKT[482])*work.d_inv[236];
  work.L[409] = (work.KKT[483])*work.d_inv[236];
  work.v[44] = work.L[314]*work.d[44];
  work.v[176] = work.L[315]*work.d[176];
  work.v[236] = work.L[316]*work.d[236];
  work.v[237] = work.KKT[484]-work.L[314]*work.v[44]-work.L[315]*work.v[176]-work.L[316]*work.v[236];
  work.d[237] = work.v[237];
  if (work.d[237] > 0)
    work.d[237] = -settings.kkt_reg;
  else
    work.d[237] -= settings.kkt_reg;
  work.d_inv[237] = 1/work.d[237];
  work.L[320] = (-work.L[318]*work.v[176]-work.L[319]*work.v[236])*work.d_inv[237];
  work.L[323] = (work.KKT[485])*work.d_inv[237];
  work.L[373] = (-work.L[372]*work.v[236])*work.d_inv[237];
  work.L[410] = (-work.L[409]*work.v[236])*work.d_inv[237];
  work.L[583] = (-work.L[569]*work.v[176])*work.d_inv[237];
  work.v[50] = work.L[317]*work.d[50];
  work.v[176] = work.L[318]*work.d[176];
  work.v[236] = work.L[319]*work.d[236];
  work.v[237] = work.L[320]*work.d[237];
  work.v[238] = work.KKT[486]-work.L[317]*work.v[50]-work.L[318]*work.v[176]-work.L[319]*work.v[236]-work.L[320]*work.v[237];
  work.d[238] = work.v[238];
  if (work.d[238] > 0)
    work.d[238] = -settings.kkt_reg;
  else
    work.d[238] -= settings.kkt_reg;
  work.d_inv[238] = 1/work.d[238];
  work.L[324] = (work.KKT[487]-work.L[323]*work.v[237])*work.d_inv[238];
  work.L[374] = (-work.L[372]*work.v[236]-work.L[373]*work.v[237])*work.d_inv[238];
  work.L[411] = (-work.L[409]*work.v[236]-work.L[410]*work.v[237])*work.d_inv[238];
  work.L[584] = (-work.L[569]*work.v[176]-work.L[583]*work.v[237])*work.d_inv[238];
  work.v[167] = work.L[321]*work.d[167];
  work.v[168] = work.L[322]*work.d[168];
  work.v[237] = work.L[323]*work.d[237];
  work.v[238] = work.L[324]*work.d[238];
  work.v[239] = 0-work.L[321]*work.v[167]-work.L[322]*work.v[168]-work.L[323]*work.v[237]-work.L[324]*work.v[238];
  work.d[239] = work.v[239];
  if (work.d[239] < 0)
    work.d[239] = settings.kkt_reg;
  else
    work.d[239] += settings.kkt_reg;
  work.d_inv[239] = 1/work.d[239];
  work.L[375] = (work.KKT[488]-work.L[373]*work.v[237]-work.L[374]*work.v[238])*work.d_inv[239];
  work.L[412] = (work.KKT[489]-work.L[410]*work.v[237]-work.L[411]*work.v[238])*work.d_inv[239];
  work.L[548] = (-work.L[539]*work.v[167]-work.L[540]*work.v[168])*work.d_inv[239];
  work.L[585] = (-work.L[583]*work.v[237]-work.L[584]*work.v[238])*work.d_inv[239];
  work.v[240] = 0;
  work.d[240] = work.v[240];
  if (work.d[240] < 0)
    work.d[240] = settings.kkt_reg;
  else
    work.d[240] += settings.kkt_reg;
  work.d_inv[240] = 1/work.d[240];
  work.L[327] = (work.KKT[490])*work.d_inv[240];
  work.L[330] = (work.KKT[491])*work.d_inv[240];
  work.L[380] = (work.KKT[492])*work.d_inv[240];
  work.L[605] = (work.KKT[493])*work.d_inv[240];
  work.v[45] = work.L[325]*work.d[45];
  work.v[177] = work.L[326]*work.d[177];
  work.v[240] = work.L[327]*work.d[240];
  work.v[241] = work.KKT[494]-work.L[325]*work.v[45]-work.L[326]*work.v[177]-work.L[327]*work.v[240];
  work.d[241] = work.v[241];
  if (work.d[241] > 0)
    work.d[241] = -settings.kkt_reg;
  else
    work.d[241] -= settings.kkt_reg;
  work.d_inv[241] = 1/work.d[241];
  work.L[331] = (-work.L[329]*work.v[177]-work.L[330]*work.v[240])*work.d_inv[241];
  work.L[334] = (work.KKT[495])*work.d_inv[241];
  work.L[381] = (-work.L[380]*work.v[240])*work.d_inv[241];
  work.L[586] = (-work.L[570]*work.v[177])*work.d_inv[241];
  work.L[606] = (-work.L[605]*work.v[240])*work.d_inv[241];
  work.v[51] = work.L[328]*work.d[51];
  work.v[177] = work.L[329]*work.d[177];
  work.v[240] = work.L[330]*work.d[240];
  work.v[241] = work.L[331]*work.d[241];
  work.v[242] = work.KKT[496]-work.L[328]*work.v[51]-work.L[329]*work.v[177]-work.L[330]*work.v[240]-work.L[331]*work.v[241];
  work.d[242] = work.v[242];
  if (work.d[242] > 0)
    work.d[242] = -settings.kkt_reg;
  else
    work.d[242] -= settings.kkt_reg;
  work.d_inv[242] = 1/work.d[242];
  work.L[335] = (work.KKT[497]-work.L[334]*work.v[241])*work.d_inv[242];
  work.L[382] = (-work.L[380]*work.v[240]-work.L[381]*work.v[241])*work.d_inv[242];
  work.L[587] = (-work.L[570]*work.v[177]-work.L[586]*work.v[241])*work.d_inv[242];
  work.L[607] = (-work.L[605]*work.v[240]-work.L[606]*work.v[241])*work.d_inv[242];
  work.v[170] = work.L[332]*work.d[170];
  work.v[171] = work.L[333]*work.d[171];
  work.v[241] = work.L[334]*work.d[241];
  work.v[242] = work.L[335]*work.d[242];
  work.v[243] = 0-work.L[332]*work.v[170]-work.L[333]*work.v[171]-work.L[334]*work.v[241]-work.L[335]*work.v[242];
  work.d[243] = work.v[243];
  if (work.d[243] < 0)
    work.d[243] = settings.kkt_reg;
  else
    work.d[243] += settings.kkt_reg;
  work.d_inv[243] = 1/work.d[243];
  work.L[383] = (work.KKT[498]-work.L[381]*work.v[241]-work.L[382]*work.v[242])*work.d_inv[243];
  work.L[549] = (-work.L[542]*work.v[170]-work.L[543]*work.v[171])*work.d_inv[243];
  work.L[588] = (-work.L[586]*work.v[241]-work.L[587]*work.v[242])*work.d_inv[243];
  work.L[608] = (work.KKT[499]-work.L[606]*work.v[241]-work.L[607]*work.v[242])*work.d_inv[243];
  work.v[244] = 0;
  work.d[244] = work.v[244];
  if (work.d[244] < 0)
    work.d[244] = settings.kkt_reg;
  else
    work.d[244] += settings.kkt_reg;
  work.d_inv[244] = 1/work.d[244];
  work.L[457] = (work.KKT[500])*work.d_inv[244];
  work.L[512] = (work.KKT[501])*work.d_inv[244];
  work.L[550] = (work.KKT[502])*work.d_inv[244];
  work.L[589] = (work.KKT[503])*work.d_inv[244];
  work.v[124] = work.L[336]*work.d[124];
  work.v[182] = work.L[337]*work.d[182];
  work.v[183] = work.L[338]*work.d[183];
  work.v[214] = work.L[339]*work.d[214];
  work.v[220] = work.L[340]*work.d[220];
  work.v[221] = work.L[341]*work.d[221];
  work.v[222] = work.L[342]*work.d[222];
  work.v[223] = work.L[343]*work.d[223];
  work.v[245] = 0-work.L[336]*work.v[124]-work.L[337]*work.v[182]-work.L[338]*work.v[183]-work.L[339]*work.v[214]-work.L[340]*work.v[220]-work.L[341]*work.v[221]-work.L[342]*work.v[222]-work.L[343]*work.v[223];
  work.d[245] = work.v[245];
  if (work.d[245] > 0)
    work.d[245] = -settings.kkt_reg;
  else
    work.d[245] -= settings.kkt_reg;
  work.d_inv[245] = 1/work.d[245];
  work.L[389] = (-work.L[384]*work.v[214]-work.L[385]*work.v[220]-work.L[386]*work.v[221]-work.L[387]*work.v[222]-work.L[388]*work.v[223])*work.d_inv[245];
  work.L[458] = (-work.L[434]*work.v[182]-work.L[435]*work.v[183]-work.L[451]*work.v[214])*work.d_inv[245];
  work.L[513] = (-work.L[479]*work.v[182]-work.L[480]*work.v[183]-work.L[506]*work.v[214])*work.d_inv[245];
  work.L[551] = (-work.L[544]*work.v[223])*work.d_inv[245];
  work.L[590] = (-work.L[571]*work.v[221]-work.L[572]*work.v[222]-work.L[573]*work.v[223])*work.d_inv[245];
  work.v[125] = work.L[344]*work.d[125];
  work.v[188] = work.L[345]*work.d[188];
  work.v[189] = work.L[346]*work.d[189];
  work.v[215] = work.L[347]*work.d[215];
  work.v[224] = work.L[348]*work.d[224];
  work.v[225] = work.L[349]*work.d[225];
  work.v[226] = work.L[350]*work.d[226];
  work.v[227] = work.L[351]*work.d[227];
  work.v[246] = 0-work.L[344]*work.v[125]-work.L[345]*work.v[188]-work.L[346]*work.v[189]-work.L[347]*work.v[215]-work.L[348]*work.v[224]-work.L[349]*work.v[225]-work.L[350]*work.v[226]-work.L[351]*work.v[227];
  work.d[246] = work.v[246];
  if (work.d[246] > 0)
    work.d[246] = -settings.kkt_reg;
  else
    work.d[246] -= settings.kkt_reg;
  work.d_inv[246] = 1/work.d[246];
  work.L[395] = (-work.L[390]*work.v[215]-work.L[391]*work.v[224]-work.L[392]*work.v[225]-work.L[393]*work.v[226]-work.L[394]*work.v[227])*work.d_inv[246];
  work.L[459] = (-work.L[437]*work.v[188]-work.L[438]*work.v[189]-work.L[452]*work.v[215])*work.d_inv[246];
  work.L[514] = (-work.L[484]*work.v[188]-work.L[485]*work.v[189]-work.L[507]*work.v[215])*work.d_inv[246];
  work.L[552] = (-work.L[545]*work.v[227])*work.d_inv[246];
  work.L[591] = (-work.L[574]*work.v[225]-work.L[575]*work.v[226]-work.L[576]*work.v[227])*work.d_inv[246];
  work.v[126] = work.L[352]*work.d[126];
  work.v[194] = work.L[353]*work.d[194];
  work.v[195] = work.L[354]*work.d[195];
  work.v[216] = work.L[355]*work.d[216];
  work.v[228] = work.L[356]*work.d[228];
  work.v[229] = work.L[357]*work.d[229];
  work.v[230] = work.L[358]*work.d[230];
  work.v[231] = work.L[359]*work.d[231];
  work.v[247] = 0-work.L[352]*work.v[126]-work.L[353]*work.v[194]-work.L[354]*work.v[195]-work.L[355]*work.v[216]-work.L[356]*work.v[228]-work.L[357]*work.v[229]-work.L[358]*work.v[230]-work.L[359]*work.v[231];
  work.d[247] = work.v[247];
  if (work.d[247] > 0)
    work.d[247] = -settings.kkt_reg;
  else
    work.d[247] -= settings.kkt_reg;
  work.d_inv[247] = 1/work.d[247];
  work.L[401] = (-work.L[396]*work.v[216]-work.L[397]*work.v[228]-work.L[398]*work.v[229]-work.L[399]*work.v[230]-work.L[400]*work.v[231])*work.d_inv[247];
  work.L[460] = (-work.L[440]*work.v[194]-work.L[441]*work.v[195]-work.L[453]*work.v[216])*work.d_inv[247];
  work.L[515] = (-work.L[489]*work.v[194]-work.L[490]*work.v[195]-work.L[508]*work.v[216])*work.d_inv[247];
  work.L[553] = (-work.L[546]*work.v[231])*work.d_inv[247];
  work.L[592] = (-work.L[577]*work.v[229]-work.L[578]*work.v[230]-work.L[579]*work.v[231])*work.d_inv[247];
  work.v[127] = work.L[360]*work.d[127];
  work.v[200] = work.L[361]*work.d[200];
  work.v[201] = work.L[362]*work.d[201];
  work.v[217] = work.L[363]*work.d[217];
  work.v[232] = work.L[364]*work.d[232];
  work.v[233] = work.L[365]*work.d[233];
  work.v[234] = work.L[366]*work.d[234];
  work.v[235] = work.L[367]*work.d[235];
  work.v[248] = 0-work.L[360]*work.v[127]-work.L[361]*work.v[200]-work.L[362]*work.v[201]-work.L[363]*work.v[217]-work.L[364]*work.v[232]-work.L[365]*work.v[233]-work.L[366]*work.v[234]-work.L[367]*work.v[235];
  work.d[248] = work.v[248];
  if (work.d[248] > 0)
    work.d[248] = -settings.kkt_reg;
  else
    work.d[248] -= settings.kkt_reg;
  work.d_inv[248] = 1/work.d[248];
  work.L[407] = (-work.L[402]*work.v[217]-work.L[403]*work.v[232]-work.L[404]*work.v[233]-work.L[405]*work.v[234]-work.L[406]*work.v[235])*work.d_inv[248];
  work.L[461] = (-work.L[443]*work.v[200]-work.L[444]*work.v[201]-work.L[454]*work.v[217])*work.d_inv[248];
  work.L[516] = (-work.L[494]*work.v[200]-work.L[495]*work.v[201]-work.L[509]*work.v[217])*work.d_inv[248];
  work.L[554] = (-work.L[547]*work.v[235])*work.d_inv[248];
  work.L[593] = (-work.L[580]*work.v[233]-work.L[581]*work.v[234]-work.L[582]*work.v[235])*work.d_inv[248];
  work.v[128] = work.L[368]*work.d[128];
  work.v[206] = work.L[369]*work.d[206];
  work.v[207] = work.L[370]*work.d[207];
  work.v[218] = work.L[371]*work.d[218];
  work.v[236] = work.L[372]*work.d[236];
  work.v[237] = work.L[373]*work.d[237];
  work.v[238] = work.L[374]*work.d[238];
  work.v[239] = work.L[375]*work.d[239];
  work.v[249] = 0-work.L[368]*work.v[128]-work.L[369]*work.v[206]-work.L[370]*work.v[207]-work.L[371]*work.v[218]-work.L[372]*work.v[236]-work.L[373]*work.v[237]-work.L[374]*work.v[238]-work.L[375]*work.v[239];
  work.d[249] = work.v[249];
  if (work.d[249] > 0)
    work.d[249] = -settings.kkt_reg;
  else
    work.d[249] -= settings.kkt_reg;
  work.d_inv[249] = 1/work.d[249];
  work.L[413] = (-work.L[408]*work.v[218]-work.L[409]*work.v[236]-work.L[410]*work.v[237]-work.L[411]*work.v[238]-work.L[412]*work.v[239])*work.d_inv[249];
  work.L[462] = (-work.L[446]*work.v[206]-work.L[447]*work.v[207]-work.L[455]*work.v[218])*work.d_inv[249];
  work.L[517] = (-work.L[499]*work.v[206]-work.L[500]*work.v[207]-work.L[510]*work.v[218])*work.d_inv[249];
  work.L[555] = (-work.L[548]*work.v[239])*work.d_inv[249];
  work.L[594] = (-work.L[583]*work.v[237]-work.L[584]*work.v[238]-work.L[585]*work.v[239])*work.d_inv[249];
  work.v[129] = work.L[376]*work.d[129];
  work.v[212] = work.L[377]*work.d[212];
  work.v[213] = work.L[378]*work.d[213];
  work.v[219] = work.L[379]*work.d[219];
  work.v[240] = work.L[380]*work.d[240];
  work.v[241] = work.L[381]*work.d[241];
  work.v[242] = work.L[382]*work.d[242];
  work.v[243] = work.L[383]*work.d[243];
  work.v[250] = 0-work.L[376]*work.v[129]-work.L[377]*work.v[212]-work.L[378]*work.v[213]-work.L[379]*work.v[219]-work.L[380]*work.v[240]-work.L[381]*work.v[241]-work.L[382]*work.v[242]-work.L[383]*work.v[243];
  work.d[250] = work.v[250];
  if (work.d[250] > 0)
    work.d[250] = -settings.kkt_reg;
  else
    work.d[250] -= settings.kkt_reg;
  work.d_inv[250] = 1/work.d[250];
  work.L[463] = (-work.L[449]*work.v[212]-work.L[450]*work.v[213]-work.L[456]*work.v[219])*work.d_inv[250];
  work.L[518] = (-work.L[504]*work.v[212]-work.L[505]*work.v[213]-work.L[511]*work.v[219])*work.d_inv[250];
  work.L[556] = (-work.L[549]*work.v[243])*work.d_inv[250];
  work.L[595] = (-work.L[586]*work.v[241]-work.L[587]*work.v[242]-work.L[588]*work.v[243])*work.d_inv[250];
  work.L[609] = (-work.L[604]*work.v[219]-work.L[605]*work.v[240]-work.L[606]*work.v[241]-work.L[607]*work.v[242]-work.L[608]*work.v[243])*work.d_inv[250];
  work.v[214] = work.L[384]*work.d[214];
  work.v[220] = work.L[385]*work.d[220];
  work.v[221] = work.L[386]*work.d[221];
  work.v[222] = work.L[387]*work.d[222];
  work.v[223] = work.L[388]*work.d[223];
  work.v[245] = work.L[389]*work.d[245];
  work.v[251] = 0-work.L[384]*work.v[214]-work.L[385]*work.v[220]-work.L[386]*work.v[221]-work.L[387]*work.v[222]-work.L[388]*work.v[223]-work.L[389]*work.v[245];
  work.d[251] = work.v[251];
  if (work.d[251] > 0)
    work.d[251] = -settings.kkt_reg;
  else
    work.d[251] -= settings.kkt_reg;
  work.d_inv[251] = 1/work.d[251];
  work.L[464] = (-work.L[451]*work.v[214]-work.L[458]*work.v[245])*work.d_inv[251];
  work.L[519] = (-work.L[506]*work.v[214]-work.L[513]*work.v[245])*work.d_inv[251];
  work.L[557] = (-work.L[544]*work.v[223]-work.L[551]*work.v[245])*work.d_inv[251];
  work.L[596] = (-work.L[571]*work.v[221]-work.L[572]*work.v[222]-work.L[573]*work.v[223]-work.L[590]*work.v[245])*work.d_inv[251];
  work.v[215] = work.L[390]*work.d[215];
  work.v[224] = work.L[391]*work.d[224];
  work.v[225] = work.L[392]*work.d[225];
  work.v[226] = work.L[393]*work.d[226];
  work.v[227] = work.L[394]*work.d[227];
  work.v[246] = work.L[395]*work.d[246];
  work.v[252] = 0-work.L[390]*work.v[215]-work.L[391]*work.v[224]-work.L[392]*work.v[225]-work.L[393]*work.v[226]-work.L[394]*work.v[227]-work.L[395]*work.v[246];
  work.d[252] = work.v[252];
  if (work.d[252] > 0)
    work.d[252] = -settings.kkt_reg;
  else
    work.d[252] -= settings.kkt_reg;
  work.d_inv[252] = 1/work.d[252];
  work.L[465] = (-work.L[452]*work.v[215]-work.L[459]*work.v[246])*work.d_inv[252];
  work.L[520] = (-work.L[507]*work.v[215]-work.L[514]*work.v[246])*work.d_inv[252];
  work.L[558] = (-work.L[545]*work.v[227]-work.L[552]*work.v[246])*work.d_inv[252];
  work.L[597] = (-work.L[574]*work.v[225]-work.L[575]*work.v[226]-work.L[576]*work.v[227]-work.L[591]*work.v[246])*work.d_inv[252];
  work.v[216] = work.L[396]*work.d[216];
  work.v[228] = work.L[397]*work.d[228];
  work.v[229] = work.L[398]*work.d[229];
  work.v[230] = work.L[399]*work.d[230];
  work.v[231] = work.L[400]*work.d[231];
  work.v[247] = work.L[401]*work.d[247];
  work.v[253] = 0-work.L[396]*work.v[216]-work.L[397]*work.v[228]-work.L[398]*work.v[229]-work.L[399]*work.v[230]-work.L[400]*work.v[231]-work.L[401]*work.v[247];
  work.d[253] = work.v[253];
  if (work.d[253] > 0)
    work.d[253] = -settings.kkt_reg;
  else
    work.d[253] -= settings.kkt_reg;
  work.d_inv[253] = 1/work.d[253];
  work.L[466] = (-work.L[453]*work.v[216]-work.L[460]*work.v[247])*work.d_inv[253];
  work.L[521] = (-work.L[508]*work.v[216]-work.L[515]*work.v[247])*work.d_inv[253];
  work.L[559] = (-work.L[546]*work.v[231]-work.L[553]*work.v[247])*work.d_inv[253];
  work.L[598] = (-work.L[577]*work.v[229]-work.L[578]*work.v[230]-work.L[579]*work.v[231]-work.L[592]*work.v[247])*work.d_inv[253];
  work.v[217] = work.L[402]*work.d[217];
  work.v[232] = work.L[403]*work.d[232];
  work.v[233] = work.L[404]*work.d[233];
  work.v[234] = work.L[405]*work.d[234];
  work.v[235] = work.L[406]*work.d[235];
  work.v[248] = work.L[407]*work.d[248];
  work.v[254] = 0-work.L[402]*work.v[217]-work.L[403]*work.v[232]-work.L[404]*work.v[233]-work.L[405]*work.v[234]-work.L[406]*work.v[235]-work.L[407]*work.v[248];
  work.d[254] = work.v[254];
  if (work.d[254] > 0)
    work.d[254] = -settings.kkt_reg;
  else
    work.d[254] -= settings.kkt_reg;
  work.d_inv[254] = 1/work.d[254];
  work.L[467] = (-work.L[454]*work.v[217]-work.L[461]*work.v[248])*work.d_inv[254];
  work.L[522] = (-work.L[509]*work.v[217]-work.L[516]*work.v[248])*work.d_inv[254];
  work.L[560] = (-work.L[547]*work.v[235]-work.L[554]*work.v[248])*work.d_inv[254];
  work.L[599] = (-work.L[580]*work.v[233]-work.L[581]*work.v[234]-work.L[582]*work.v[235]-work.L[593]*work.v[248])*work.d_inv[254];
  work.v[218] = work.L[408]*work.d[218];
  work.v[236] = work.L[409]*work.d[236];
  work.v[237] = work.L[410]*work.d[237];
  work.v[238] = work.L[411]*work.d[238];
  work.v[239] = work.L[412]*work.d[239];
  work.v[249] = work.L[413]*work.d[249];
  work.v[255] = 0-work.L[408]*work.v[218]-work.L[409]*work.v[236]-work.L[410]*work.v[237]-work.L[411]*work.v[238]-work.L[412]*work.v[239]-work.L[413]*work.v[249];
  work.d[255] = work.v[255];
  if (work.d[255] > 0)
    work.d[255] = -settings.kkt_reg;
  else
    work.d[255] -= settings.kkt_reg;
  work.d_inv[255] = 1/work.d[255];
  work.L[468] = (-work.L[455]*work.v[218]-work.L[462]*work.v[249])*work.d_inv[255];
  work.L[523] = (-work.L[510]*work.v[218]-work.L[517]*work.v[249])*work.d_inv[255];
  work.L[561] = (-work.L[548]*work.v[239]-work.L[555]*work.v[249])*work.d_inv[255];
  work.L[600] = (-work.L[583]*work.v[237]-work.L[584]*work.v[238]-work.L[585]*work.v[239]-work.L[594]*work.v[249])*work.d_inv[255];
  work.v[0] = work.L[414]*work.d[0];
  work.v[130] = work.L[415]*work.d[130];
  work.v[131] = work.L[416]*work.d[131];
  work.v[132] = work.L[417]*work.d[132];
  work.v[133] = work.L[418]*work.d[133];
  work.v[134] = work.L[419]*work.d[134];
  work.v[135] = work.L[420]*work.d[135];
  work.v[136] = work.L[421]*work.d[136];
  work.v[137] = work.L[422]*work.d[137];
  work.v[138] = work.L[423]*work.d[138];
  work.v[139] = work.L[424]*work.d[139];
  work.v[140] = work.L[425]*work.d[140];
  work.v[141] = work.L[426]*work.d[141];
  work.v[142] = work.L[427]*work.d[142];
  work.v[143] = work.L[428]*work.d[143];
  work.v[144] = work.L[429]*work.d[144];
  work.v[145] = work.L[430]*work.d[145];
  work.v[146] = work.L[431]*work.d[146];
  work.v[147] = work.L[432]*work.d[147];
  work.v[181] = work.L[433]*work.d[181];
  work.v[182] = work.L[434]*work.d[182];
  work.v[183] = work.L[435]*work.d[183];
  work.v[187] = work.L[436]*work.d[187];
  work.v[188] = work.L[437]*work.d[188];
  work.v[189] = work.L[438]*work.d[189];
  work.v[193] = work.L[439]*work.d[193];
  work.v[194] = work.L[440]*work.d[194];
  work.v[195] = work.L[441]*work.d[195];
  work.v[199] = work.L[442]*work.d[199];
  work.v[200] = work.L[443]*work.d[200];
  work.v[201] = work.L[444]*work.d[201];
  work.v[205] = work.L[445]*work.d[205];
  work.v[206] = work.L[446]*work.d[206];
  work.v[207] = work.L[447]*work.d[207];
  work.v[211] = work.L[448]*work.d[211];
  work.v[212] = work.L[449]*work.d[212];
  work.v[213] = work.L[450]*work.d[213];
  work.v[214] = work.L[451]*work.d[214];
  work.v[215] = work.L[452]*work.d[215];
  work.v[216] = work.L[453]*work.d[216];
  work.v[217] = work.L[454]*work.d[217];
  work.v[218] = work.L[455]*work.d[218];
  work.v[219] = work.L[456]*work.d[219];
  work.v[244] = work.L[457]*work.d[244];
  work.v[245] = work.L[458]*work.d[245];
  work.v[246] = work.L[459]*work.d[246];
  work.v[247] = work.L[460]*work.d[247];
  work.v[248] = work.L[461]*work.d[248];
  work.v[249] = work.L[462]*work.d[249];
  work.v[250] = work.L[463]*work.d[250];
  work.v[251] = work.L[464]*work.d[251];
  work.v[252] = work.L[465]*work.d[252];
  work.v[253] = work.L[466]*work.d[253];
  work.v[254] = work.L[467]*work.d[254];
  work.v[255] = work.L[468]*work.d[255];
  work.v[256] = work.KKT[504]-work.L[414]*work.v[0]-work.L[415]*work.v[130]-work.L[416]*work.v[131]-work.L[417]*work.v[132]-work.L[418]*work.v[133]-work.L[419]*work.v[134]-work.L[420]*work.v[135]-work.L[421]*work.v[136]-work.L[422]*work.v[137]-work.L[423]*work.v[138]-work.L[424]*work.v[139]-work.L[425]*work.v[140]-work.L[426]*work.v[141]-work.L[427]*work.v[142]-work.L[428]*work.v[143]-work.L[429]*work.v[144]-work.L[430]*work.v[145]-work.L[431]*work.v[146]-work.L[432]*work.v[147]-work.L[433]*work.v[181]-work.L[434]*work.v[182]-work.L[435]*work.v[183]-work.L[436]*work.v[187]-work.L[437]*work.v[188]-work.L[438]*work.v[189]-work.L[439]*work.v[193]-work.L[440]*work.v[194]-work.L[441]*work.v[195]-work.L[442]*work.v[199]-work.L[443]*work.v[200]-work.L[444]*work.v[201]-work.L[445]*work.v[205]-work.L[446]*work.v[206]-work.L[447]*work.v[207]-work.L[448]*work.v[211]-work.L[449]*work.v[212]-work.L[450]*work.v[213]-work.L[451]*work.v[214]-work.L[452]*work.v[215]-work.L[453]*work.v[216]-work.L[454]*work.v[217]-work.L[455]*work.v[218]-work.L[456]*work.v[219]-work.L[457]*work.v[244]-work.L[458]*work.v[245]-work.L[459]*work.v[246]-work.L[460]*work.v[247]-work.L[461]*work.v[248]-work.L[462]*work.v[249]-work.L[463]*work.v[250]-work.L[464]*work.v[251]-work.L[465]*work.v[252]-work.L[466]*work.v[253]-work.L[467]*work.v[254]-work.L[468]*work.v[255];
  work.d[256] = work.v[256];
  if (work.d[256] > 0)
    work.d[256] = -settings.kkt_reg;
  else
    work.d[256] -= settings.kkt_reg;
  work.d_inv[256] = 1/work.d[256];
  work.L[524] = (-work.L[478]*work.v[181]-work.L[479]*work.v[182]-work.L[480]*work.v[183]-work.L[483]*work.v[187]-work.L[484]*work.v[188]-work.L[485]*work.v[189]-work.L[488]*work.v[193]-work.L[489]*work.v[194]-work.L[490]*work.v[195]-work.L[493]*work.v[199]-work.L[494]*work.v[200]-work.L[495]*work.v[201]-work.L[498]*work.v[205]-work.L[499]*work.v[206]-work.L[500]*work.v[207]-work.L[503]*work.v[211]-work.L[504]*work.v[212]-work.L[505]*work.v[213]-work.L[506]*work.v[214]-work.L[507]*work.v[215]-work.L[508]*work.v[216]-work.L[509]*work.v[217]-work.L[510]*work.v[218]-work.L[511]*work.v[219]-work.L[512]*work.v[244]-work.L[513]*work.v[245]-work.L[514]*work.v[246]-work.L[515]*work.v[247]-work.L[516]*work.v[248]-work.L[517]*work.v[249]-work.L[518]*work.v[250]-work.L[519]*work.v[251]-work.L[520]*work.v[252]-work.L[521]*work.v[253]-work.L[522]*work.v[254]-work.L[523]*work.v[255])*work.d_inv[256];
  work.L[562] = (-work.L[550]*work.v[244]-work.L[551]*work.v[245]-work.L[552]*work.v[246]-work.L[553]*work.v[247]-work.L[554]*work.v[248]-work.L[555]*work.v[249]-work.L[556]*work.v[250]-work.L[557]*work.v[251]-work.L[558]*work.v[252]-work.L[559]*work.v[253]-work.L[560]*work.v[254]-work.L[561]*work.v[255])*work.d_inv[256];
  work.L[601] = (-work.L[589]*work.v[244]-work.L[590]*work.v[245]-work.L[591]*work.v[246]-work.L[592]*work.v[247]-work.L[593]*work.v[248]-work.L[594]*work.v[249]-work.L[595]*work.v[250]-work.L[596]*work.v[251]-work.L[597]*work.v[252]-work.L[598]*work.v[253]-work.L[599]*work.v[254]-work.L[600]*work.v[255])*work.d_inv[256];
  work.L[610] = (-work.L[604]*work.v[219]-work.L[609]*work.v[250])*work.d_inv[256];
  work.v[13] = work.L[469]*work.d[13];
  work.v[148] = work.L[470]*work.d[148];
  work.v[149] = work.L[471]*work.d[149];
  work.v[150] = work.L[472]*work.d[150];
  work.v[151] = work.L[473]*work.d[151];
  work.v[152] = work.L[474]*work.d[152];
  work.v[153] = work.L[475]*work.d[153];
  work.v[179] = work.L[476]*work.d[179];
  work.v[180] = work.L[477]*work.d[180];
  work.v[181] = work.L[478]*work.d[181];
  work.v[182] = work.L[479]*work.d[182];
  work.v[183] = work.L[480]*work.d[183];
  work.v[185] = work.L[481]*work.d[185];
  work.v[186] = work.L[482]*work.d[186];
  work.v[187] = work.L[483]*work.d[187];
  work.v[188] = work.L[484]*work.d[188];
  work.v[189] = work.L[485]*work.d[189];
  work.v[191] = work.L[486]*work.d[191];
  work.v[192] = work.L[487]*work.d[192];
  work.v[193] = work.L[488]*work.d[193];
  work.v[194] = work.L[489]*work.d[194];
  work.v[195] = work.L[490]*work.d[195];
  work.v[197] = work.L[491]*work.d[197];
  work.v[198] = work.L[492]*work.d[198];
  work.v[199] = work.L[493]*work.d[199];
  work.v[200] = work.L[494]*work.d[200];
  work.v[201] = work.L[495]*work.d[201];
  work.v[203] = work.L[496]*work.d[203];
  work.v[204] = work.L[497]*work.d[204];
  work.v[205] = work.L[498]*work.d[205];
  work.v[206] = work.L[499]*work.d[206];
  work.v[207] = work.L[500]*work.d[207];
  work.v[209] = work.L[501]*work.d[209];
  work.v[210] = work.L[502]*work.d[210];
  work.v[211] = work.L[503]*work.d[211];
  work.v[212] = work.L[504]*work.d[212];
  work.v[213] = work.L[505]*work.d[213];
  work.v[214] = work.L[506]*work.d[214];
  work.v[215] = work.L[507]*work.d[215];
  work.v[216] = work.L[508]*work.d[216];
  work.v[217] = work.L[509]*work.d[217];
  work.v[218] = work.L[510]*work.d[218];
  work.v[219] = work.L[511]*work.d[219];
  work.v[244] = work.L[512]*work.d[244];
  work.v[245] = work.L[513]*work.d[245];
  work.v[246] = work.L[514]*work.d[246];
  work.v[247] = work.L[515]*work.d[247];
  work.v[248] = work.L[516]*work.d[248];
  work.v[249] = work.L[517]*work.d[249];
  work.v[250] = work.L[518]*work.d[250];
  work.v[251] = work.L[519]*work.d[251];
  work.v[252] = work.L[520]*work.d[252];
  work.v[253] = work.L[521]*work.d[253];
  work.v[254] = work.L[522]*work.d[254];
  work.v[255] = work.L[523]*work.d[255];
  work.v[256] = work.L[524]*work.d[256];
  work.v[257] = work.KKT[505]-work.L[469]*work.v[13]-work.L[470]*work.v[148]-work.L[471]*work.v[149]-work.L[472]*work.v[150]-work.L[473]*work.v[151]-work.L[474]*work.v[152]-work.L[475]*work.v[153]-work.L[476]*work.v[179]-work.L[477]*work.v[180]-work.L[478]*work.v[181]-work.L[479]*work.v[182]-work.L[480]*work.v[183]-work.L[481]*work.v[185]-work.L[482]*work.v[186]-work.L[483]*work.v[187]-work.L[484]*work.v[188]-work.L[485]*work.v[189]-work.L[486]*work.v[191]-work.L[487]*work.v[192]-work.L[488]*work.v[193]-work.L[489]*work.v[194]-work.L[490]*work.v[195]-work.L[491]*work.v[197]-work.L[492]*work.v[198]-work.L[493]*work.v[199]-work.L[494]*work.v[200]-work.L[495]*work.v[201]-work.L[496]*work.v[203]-work.L[497]*work.v[204]-work.L[498]*work.v[205]-work.L[499]*work.v[206]-work.L[500]*work.v[207]-work.L[501]*work.v[209]-work.L[502]*work.v[210]-work.L[503]*work.v[211]-work.L[504]*work.v[212]-work.L[505]*work.v[213]-work.L[506]*work.v[214]-work.L[507]*work.v[215]-work.L[508]*work.v[216]-work.L[509]*work.v[217]-work.L[510]*work.v[218]-work.L[511]*work.v[219]-work.L[512]*work.v[244]-work.L[513]*work.v[245]-work.L[514]*work.v[246]-work.L[515]*work.v[247]-work.L[516]*work.v[248]-work.L[517]*work.v[249]-work.L[518]*work.v[250]-work.L[519]*work.v[251]-work.L[520]*work.v[252]-work.L[521]*work.v[253]-work.L[522]*work.v[254]-work.L[523]*work.v[255]-work.L[524]*work.v[256];
  work.d[257] = work.v[257];
  if (work.d[257] > 0)
    work.d[257] = -settings.kkt_reg;
  else
    work.d[257] -= settings.kkt_reg;
  work.d_inv[257] = 1/work.d[257];
  work.L[563] = (-work.L[550]*work.v[244]-work.L[551]*work.v[245]-work.L[552]*work.v[246]-work.L[553]*work.v[247]-work.L[554]*work.v[248]-work.L[555]*work.v[249]-work.L[556]*work.v[250]-work.L[557]*work.v[251]-work.L[558]*work.v[252]-work.L[559]*work.v[253]-work.L[560]*work.v[254]-work.L[561]*work.v[255]-work.L[562]*work.v[256])*work.d_inv[257];
  work.L[602] = (-work.L[589]*work.v[244]-work.L[590]*work.v[245]-work.L[591]*work.v[246]-work.L[592]*work.v[247]-work.L[593]*work.v[248]-work.L[594]*work.v[249]-work.L[595]*work.v[250]-work.L[596]*work.v[251]-work.L[597]*work.v[252]-work.L[598]*work.v[253]-work.L[599]*work.v[254]-work.L[600]*work.v[255]-work.L[601]*work.v[256])*work.d_inv[257];
  work.L[611] = (-work.L[604]*work.v[219]-work.L[609]*work.v[250]-work.L[610]*work.v[256])*work.d_inv[257];
  work.v[26] = work.L[525]*work.d[26];
  work.v[154] = work.L[526]*work.d[154];
  work.v[155] = work.L[527]*work.d[155];
  work.v[156] = work.L[528]*work.d[156];
  work.v[157] = work.L[529]*work.d[157];
  work.v[158] = work.L[530]*work.d[158];
  work.v[159] = work.L[531]*work.d[159];
  work.v[160] = work.L[532]*work.d[160];
  work.v[161] = work.L[533]*work.d[161];
  work.v[162] = work.L[534]*work.d[162];
  work.v[163] = work.L[535]*work.d[163];
  work.v[164] = work.L[536]*work.d[164];
  work.v[165] = work.L[537]*work.d[165];
  work.v[166] = work.L[538]*work.d[166];
  work.v[167] = work.L[539]*work.d[167];
  work.v[168] = work.L[540]*work.d[168];
  work.v[169] = work.L[541]*work.d[169];
  work.v[170] = work.L[542]*work.d[170];
  work.v[171] = work.L[543]*work.d[171];
  work.v[223] = work.L[544]*work.d[223];
  work.v[227] = work.L[545]*work.d[227];
  work.v[231] = work.L[546]*work.d[231];
  work.v[235] = work.L[547]*work.d[235];
  work.v[239] = work.L[548]*work.d[239];
  work.v[243] = work.L[549]*work.d[243];
  work.v[244] = work.L[550]*work.d[244];
  work.v[245] = work.L[551]*work.d[245];
  work.v[246] = work.L[552]*work.d[246];
  work.v[247] = work.L[553]*work.d[247];
  work.v[248] = work.L[554]*work.d[248];
  work.v[249] = work.L[555]*work.d[249];
  work.v[250] = work.L[556]*work.d[250];
  work.v[251] = work.L[557]*work.d[251];
  work.v[252] = work.L[558]*work.d[252];
  work.v[253] = work.L[559]*work.d[253];
  work.v[254] = work.L[560]*work.d[254];
  work.v[255] = work.L[561]*work.d[255];
  work.v[256] = work.L[562]*work.d[256];
  work.v[257] = work.L[563]*work.d[257];
  work.v[258] = work.KKT[506]-work.L[525]*work.v[26]-work.L[526]*work.v[154]-work.L[527]*work.v[155]-work.L[528]*work.v[156]-work.L[529]*work.v[157]-work.L[530]*work.v[158]-work.L[531]*work.v[159]-work.L[532]*work.v[160]-work.L[533]*work.v[161]-work.L[534]*work.v[162]-work.L[535]*work.v[163]-work.L[536]*work.v[164]-work.L[537]*work.v[165]-work.L[538]*work.v[166]-work.L[539]*work.v[167]-work.L[540]*work.v[168]-work.L[541]*work.v[169]-work.L[542]*work.v[170]-work.L[543]*work.v[171]-work.L[544]*work.v[223]-work.L[545]*work.v[227]-work.L[546]*work.v[231]-work.L[547]*work.v[235]-work.L[548]*work.v[239]-work.L[549]*work.v[243]-work.L[550]*work.v[244]-work.L[551]*work.v[245]-work.L[552]*work.v[246]-work.L[553]*work.v[247]-work.L[554]*work.v[248]-work.L[555]*work.v[249]-work.L[556]*work.v[250]-work.L[557]*work.v[251]-work.L[558]*work.v[252]-work.L[559]*work.v[253]-work.L[560]*work.v[254]-work.L[561]*work.v[255]-work.L[562]*work.v[256]-work.L[563]*work.v[257];
  work.d[258] = work.v[258];
  if (work.d[258] > 0)
    work.d[258] = -settings.kkt_reg;
  else
    work.d[258] -= settings.kkt_reg;
  work.d_inv[258] = 1/work.d[258];
  work.L[603] = (-work.L[573]*work.v[223]-work.L[576]*work.v[227]-work.L[579]*work.v[231]-work.L[582]*work.v[235]-work.L[585]*work.v[239]-work.L[588]*work.v[243]-work.L[589]*work.v[244]-work.L[590]*work.v[245]-work.L[591]*work.v[246]-work.L[592]*work.v[247]-work.L[593]*work.v[248]-work.L[594]*work.v[249]-work.L[595]*work.v[250]-work.L[596]*work.v[251]-work.L[597]*work.v[252]-work.L[598]*work.v[253]-work.L[599]*work.v[254]-work.L[600]*work.v[255]-work.L[601]*work.v[256]-work.L[602]*work.v[257])*work.d_inv[258];
  work.L[612] = (-work.L[608]*work.v[243]-work.L[609]*work.v[250]-work.L[610]*work.v[256]-work.L[611]*work.v[257])*work.d_inv[258];
  work.v[39] = work.L[564]*work.d[39];
  work.v[172] = work.L[565]*work.d[172];
  work.v[173] = work.L[566]*work.d[173];
  work.v[174] = work.L[567]*work.d[174];
  work.v[175] = work.L[568]*work.d[175];
  work.v[176] = work.L[569]*work.d[176];
  work.v[177] = work.L[570]*work.d[177];
  work.v[221] = work.L[571]*work.d[221];
  work.v[222] = work.L[572]*work.d[222];
  work.v[223] = work.L[573]*work.d[223];
  work.v[225] = work.L[574]*work.d[225];
  work.v[226] = work.L[575]*work.d[226];
  work.v[227] = work.L[576]*work.d[227];
  work.v[229] = work.L[577]*work.d[229];
  work.v[230] = work.L[578]*work.d[230];
  work.v[231] = work.L[579]*work.d[231];
  work.v[233] = work.L[580]*work.d[233];
  work.v[234] = work.L[581]*work.d[234];
  work.v[235] = work.L[582]*work.d[235];
  work.v[237] = work.L[583]*work.d[237];
  work.v[238] = work.L[584]*work.d[238];
  work.v[239] = work.L[585]*work.d[239];
  work.v[241] = work.L[586]*work.d[241];
  work.v[242] = work.L[587]*work.d[242];
  work.v[243] = work.L[588]*work.d[243];
  work.v[244] = work.L[589]*work.d[244];
  work.v[245] = work.L[590]*work.d[245];
  work.v[246] = work.L[591]*work.d[246];
  work.v[247] = work.L[592]*work.d[247];
  work.v[248] = work.L[593]*work.d[248];
  work.v[249] = work.L[594]*work.d[249];
  work.v[250] = work.L[595]*work.d[250];
  work.v[251] = work.L[596]*work.d[251];
  work.v[252] = work.L[597]*work.d[252];
  work.v[253] = work.L[598]*work.d[253];
  work.v[254] = work.L[599]*work.d[254];
  work.v[255] = work.L[600]*work.d[255];
  work.v[256] = work.L[601]*work.d[256];
  work.v[257] = work.L[602]*work.d[257];
  work.v[258] = work.L[603]*work.d[258];
  work.v[259] = work.KKT[507]-work.L[564]*work.v[39]-work.L[565]*work.v[172]-work.L[566]*work.v[173]-work.L[567]*work.v[174]-work.L[568]*work.v[175]-work.L[569]*work.v[176]-work.L[570]*work.v[177]-work.L[571]*work.v[221]-work.L[572]*work.v[222]-work.L[573]*work.v[223]-work.L[574]*work.v[225]-work.L[575]*work.v[226]-work.L[576]*work.v[227]-work.L[577]*work.v[229]-work.L[578]*work.v[230]-work.L[579]*work.v[231]-work.L[580]*work.v[233]-work.L[581]*work.v[234]-work.L[582]*work.v[235]-work.L[583]*work.v[237]-work.L[584]*work.v[238]-work.L[585]*work.v[239]-work.L[586]*work.v[241]-work.L[587]*work.v[242]-work.L[588]*work.v[243]-work.L[589]*work.v[244]-work.L[590]*work.v[245]-work.L[591]*work.v[246]-work.L[592]*work.v[247]-work.L[593]*work.v[248]-work.L[594]*work.v[249]-work.L[595]*work.v[250]-work.L[596]*work.v[251]-work.L[597]*work.v[252]-work.L[598]*work.v[253]-work.L[599]*work.v[254]-work.L[600]*work.v[255]-work.L[601]*work.v[256]-work.L[602]*work.v[257]-work.L[603]*work.v[258];
  work.d[259] = work.v[259];
  if (work.d[259] > 0)
    work.d[259] = -settings.kkt_reg;
  else
    work.d[259] -= settings.kkt_reg;
  work.d_inv[259] = 1/work.d[259];
  work.L[613] = (-work.L[606]*work.v[241]-work.L[607]*work.v[242]-work.L[608]*work.v[243]-work.L[609]*work.v[250]-work.L[610]*work.v[256]-work.L[611]*work.v[257]-work.L[612]*work.v[258])*work.d_inv[259];
  work.v[219] = work.L[604]*work.d[219];
  work.v[240] = work.L[605]*work.d[240];
  work.v[241] = work.L[606]*work.d[241];
  work.v[242] = work.L[607]*work.d[242];
  work.v[243] = work.L[608]*work.d[243];
  work.v[250] = work.L[609]*work.d[250];
  work.v[256] = work.L[610]*work.d[256];
  work.v[257] = work.L[611]*work.d[257];
  work.v[258] = work.L[612]*work.d[258];
  work.v[259] = work.L[613]*work.d[259];
  work.v[260] = 0-work.L[604]*work.v[219]-work.L[605]*work.v[240]-work.L[606]*work.v[241]-work.L[607]*work.v[242]-work.L[608]*work.v[243]-work.L[609]*work.v[250]-work.L[610]*work.v[256]-work.L[611]*work.v[257]-work.L[612]*work.v[258]-work.L[613]*work.v[259];
  work.d[260] = work.v[260];
  if (work.d[260] > 0)
    work.d[260] = -settings.kkt_reg;
  else
    work.d[260] -= settings.kkt_reg;
  work.d_inv[260] = 1/work.d[260];
#ifndef ZERO_LIBRARY_MODE
  if (settings.debug) {
    printf("Squared Frobenius for factorization is %.8g.\n", check_factorization());
  }
#endif
}
double check_factorization(void) {
  /* Returns the squared Frobenius norm of A - L*D*L'. */
  double temp, residual;
  /* Only check the lower triangle. */
  residual = 0;
  temp = work.KKT[0]-1*work.d[0]*1;
  residual += temp*temp;
  temp = work.KKT[2]-1*work.d[1]*1;
  residual += temp*temp;
  temp = work.KKT[4]-1*work.d[2]*1;
  residual += temp*temp;
  temp = work.KKT[6]-1*work.d[3]*1;
  residual += temp*temp;
  temp = work.KKT[8]-1*work.d[4]*1;
  residual += temp*temp;
  temp = work.KKT[10]-1*work.d[5]*1;
  residual += temp*temp;
  temp = work.KKT[12]-1*work.d[6]*1;
  residual += temp*temp;
  temp = work.KKT[14]-1*work.d[7]*1;
  residual += temp*temp;
  temp = work.KKT[16]-1*work.d[8]*1;
  residual += temp*temp;
  temp = work.KKT[18]-1*work.d[9]*1;
  residual += temp*temp;
  temp = work.KKT[20]-1*work.d[10]*1;
  residual += temp*temp;
  temp = work.KKT[22]-1*work.d[11]*1;
  residual += temp*temp;
  temp = work.KKT[24]-1*work.d[12]*1;
  residual += temp*temp;
  temp = work.KKT[26]-1*work.d[13]*1;
  residual += temp*temp;
  temp = work.KKT[28]-1*work.d[14]*1;
  residual += temp*temp;
  temp = work.KKT[30]-1*work.d[15]*1;
  residual += temp*temp;
  temp = work.KKT[32]-1*work.d[16]*1;
  residual += temp*temp;
  temp = work.KKT[34]-1*work.d[17]*1;
  residual += temp*temp;
  temp = work.KKT[36]-1*work.d[18]*1;
  residual += temp*temp;
  temp = work.KKT[38]-1*work.d[19]*1;
  residual += temp*temp;
  temp = work.KKT[40]-1*work.d[20]*1;
  residual += temp*temp;
  temp = work.KKT[42]-1*work.d[21]*1;
  residual += temp*temp;
  temp = work.KKT[44]-1*work.d[22]*1;
  residual += temp*temp;
  temp = work.KKT[46]-1*work.d[23]*1;
  residual += temp*temp;
  temp = work.KKT[48]-1*work.d[24]*1;
  residual += temp*temp;
  temp = work.KKT[50]-1*work.d[25]*1;
  residual += temp*temp;
  temp = work.KKT[52]-1*work.d[26]*1;
  residual += temp*temp;
  temp = work.KKT[54]-1*work.d[27]*1;
  residual += temp*temp;
  temp = work.KKT[56]-1*work.d[28]*1;
  residual += temp*temp;
  temp = work.KKT[58]-1*work.d[29]*1;
  residual += temp*temp;
  temp = work.KKT[60]-1*work.d[30]*1;
  residual += temp*temp;
  temp = work.KKT[62]-1*work.d[31]*1;
  residual += temp*temp;
  temp = work.KKT[64]-1*work.d[32]*1;
  residual += temp*temp;
  temp = work.KKT[66]-1*work.d[33]*1;
  residual += temp*temp;
  temp = work.KKT[68]-1*work.d[34]*1;
  residual += temp*temp;
  temp = work.KKT[70]-1*work.d[35]*1;
  residual += temp*temp;
  temp = work.KKT[72]-1*work.d[36]*1;
  residual += temp*temp;
  temp = work.KKT[74]-1*work.d[37]*1;
  residual += temp*temp;
  temp = work.KKT[76]-1*work.d[38]*1;
  residual += temp*temp;
  temp = work.KKT[78]-1*work.d[39]*1;
  residual += temp*temp;
  temp = work.KKT[80]-1*work.d[40]*1;
  residual += temp*temp;
  temp = work.KKT[82]-1*work.d[41]*1;
  residual += temp*temp;
  temp = work.KKT[84]-1*work.d[42]*1;
  residual += temp*temp;
  temp = work.KKT[86]-1*work.d[43]*1;
  residual += temp*temp;
  temp = work.KKT[88]-1*work.d[44]*1;
  residual += temp*temp;
  temp = work.KKT[90]-1*work.d[45]*1;
  residual += temp*temp;
  temp = work.KKT[92]-1*work.d[46]*1;
  residual += temp*temp;
  temp = work.KKT[94]-1*work.d[47]*1;
  residual += temp*temp;
  temp = work.KKT[96]-1*work.d[48]*1;
  residual += temp*temp;
  temp = work.KKT[98]-1*work.d[49]*1;
  residual += temp*temp;
  temp = work.KKT[100]-1*work.d[50]*1;
  residual += temp*temp;
  temp = work.KKT[102]-1*work.d[51]*1;
  residual += temp*temp;
  temp = work.KKT[104]-1*work.d[52]*1;
  residual += temp*temp;
  temp = work.KKT[106]-1*work.d[53]*1;
  residual += temp*temp;
  temp = work.KKT[108]-1*work.d[54]*1;
  residual += temp*temp;
  temp = work.KKT[110]-1*work.d[55]*1;
  residual += temp*temp;
  temp = work.KKT[112]-1*work.d[56]*1;
  residual += temp*temp;
  temp = work.KKT[114]-1*work.d[57]*1;
  residual += temp*temp;
  temp = work.KKT[116]-1*work.d[58]*1;
  residual += temp*temp;
  temp = work.KKT[118]-1*work.d[59]*1;
  residual += temp*temp;
  temp = work.KKT[120]-1*work.d[60]*1;
  residual += temp*temp;
  temp = work.KKT[122]-1*work.d[61]*1;
  residual += temp*temp;
  temp = work.KKT[124]-1*work.d[62]*1;
  residual += temp*temp;
  temp = work.KKT[126]-1*work.d[63]*1;
  residual += temp*temp;
  temp = work.KKT[128]-1*work.d[64]*1;
  residual += temp*temp;
  temp = work.KKT[130]-1*work.d[65]*1;
  residual += temp*temp;
  temp = work.KKT[132]-1*work.d[66]*1;
  residual += temp*temp;
  temp = work.KKT[134]-1*work.d[67]*1;
  residual += temp*temp;
  temp = work.KKT[136]-1*work.d[68]*1;
  residual += temp*temp;
  temp = work.KKT[138]-1*work.d[69]*1;
  residual += temp*temp;
  temp = work.KKT[140]-1*work.d[70]*1;
  residual += temp*temp;
  temp = work.KKT[142]-1*work.d[71]*1;
  residual += temp*temp;
  temp = work.KKT[144]-1*work.d[72]*1;
  residual += temp*temp;
  temp = work.KKT[146]-1*work.d[73]*1;
  residual += temp*temp;
  temp = work.KKT[148]-1*work.d[74]*1;
  residual += temp*temp;
  temp = work.KKT[150]-1*work.d[75]*1;
  residual += temp*temp;
  temp = work.KKT[1]-work.L[414]*work.d[0]*1;
  residual += temp*temp;
  temp = work.KKT[3]-work.L[48]*work.d[1]*1;
  residual += temp*temp;
  temp = work.KKT[5]-work.L[53]*work.d[2]*1;
  residual += temp*temp;
  temp = work.KKT[7]-work.L[58]*work.d[3]*1;
  residual += temp*temp;
  temp = work.KKT[9]-work.L[63]*work.d[4]*1;
  residual += temp*temp;
  temp = work.KKT[11]-work.L[68]*work.d[5]*1;
  residual += temp*temp;
  temp = work.KKT[13]-work.L[73]*work.d[6]*1;
  residual += temp*temp;
  temp = work.KKT[15]-work.L[50]*work.d[7]*1;
  residual += temp*temp;
  temp = work.KKT[17]-work.L[55]*work.d[8]*1;
  residual += temp*temp;
  temp = work.KKT[19]-work.L[60]*work.d[9]*1;
  residual += temp*temp;
  temp = work.KKT[21]-work.L[65]*work.d[10]*1;
  residual += temp*temp;
  temp = work.KKT[23]-work.L[70]*work.d[11]*1;
  residual += temp*temp;
  temp = work.KKT[25]-work.L[75]*work.d[12]*1;
  residual += temp*temp;
  temp = work.KKT[27]-work.L[469]*work.d[13]*1;
  residual += temp*temp;
  temp = work.KKT[29]-work.L[108]*work.d[14]*1;
  residual += temp*temp;
  temp = work.KKT[31]-work.L[132]*work.d[15]*1;
  residual += temp*temp;
  temp = work.KKT[33]-work.L[156]*work.d[16]*1;
  residual += temp*temp;
  temp = work.KKT[35]-work.L[180]*work.d[17]*1;
  residual += temp*temp;
  temp = work.KKT[37]-work.L[204]*work.d[18]*1;
  residual += temp*temp;
  temp = work.KKT[39]-work.L[228]*work.d[19]*1;
  residual += temp*temp;
  temp = work.KKT[41]-work.L[111]*work.d[20]*1;
  residual += temp*temp;
  temp = work.KKT[43]-work.L[135]*work.d[21]*1;
  residual += temp*temp;
  temp = work.KKT[45]-work.L[159]*work.d[22]*1;
  residual += temp*temp;
  temp = work.KKT[47]-work.L[183]*work.d[23]*1;
  residual += temp*temp;
  temp = work.KKT[49]-work.L[207]*work.d[24]*1;
  residual += temp*temp;
  temp = work.KKT[51]-work.L[231]*work.d[25]*1;
  residual += temp*temp;
  temp = work.KKT[53]-work.L[525]*work.d[26]*1;
  residual += temp*temp;
  temp = work.KKT[55]-work.L[78]*work.d[27]*1;
  residual += temp*temp;
  temp = work.KKT[57]-work.L[83]*work.d[28]*1;
  residual += temp*temp;
  temp = work.KKT[59]-work.L[88]*work.d[29]*1;
  residual += temp*temp;
  temp = work.KKT[61]-work.L[93]*work.d[30]*1;
  residual += temp*temp;
  temp = work.KKT[63]-work.L[98]*work.d[31]*1;
  residual += temp*temp;
  temp = work.KKT[65]-work.L[103]*work.d[32]*1;
  residual += temp*temp;
  temp = work.KKT[67]-work.L[80]*work.d[33]*1;
  residual += temp*temp;
  temp = work.KKT[69]-work.L[85]*work.d[34]*1;
  residual += temp*temp;
  temp = work.KKT[71]-work.L[90]*work.d[35]*1;
  residual += temp*temp;
  temp = work.KKT[73]-work.L[95]*work.d[36]*1;
  residual += temp*temp;
  temp = work.KKT[75]-work.L[100]*work.d[37]*1;
  residual += temp*temp;
  temp = work.KKT[77]-work.L[105]*work.d[38]*1;
  residual += temp*temp;
  temp = work.KKT[79]-work.L[564]*work.d[39]*1;
  residual += temp*temp;
  temp = work.KKT[81]-work.L[270]*work.d[40]*1;
  residual += temp*temp;
  temp = work.KKT[83]-work.L[281]*work.d[41]*1;
  residual += temp*temp;
  temp = work.KKT[85]-work.L[292]*work.d[42]*1;
  residual += temp*temp;
  temp = work.KKT[87]-work.L[303]*work.d[43]*1;
  residual += temp*temp;
  temp = work.KKT[89]-work.L[314]*work.d[44]*1;
  residual += temp*temp;
  temp = work.KKT[91]-work.L[325]*work.d[45]*1;
  residual += temp*temp;
  temp = work.KKT[93]-work.L[273]*work.d[46]*1;
  residual += temp*temp;
  temp = work.KKT[95]-work.L[284]*work.d[47]*1;
  residual += temp*temp;
  temp = work.KKT[97]-work.L[295]*work.d[48]*1;
  residual += temp*temp;
  temp = work.KKT[99]-work.L[306]*work.d[49]*1;
  residual += temp*temp;
  temp = work.KKT[101]-work.L[317]*work.d[50]*1;
  residual += temp*temp;
  temp = work.KKT[103]-work.L[328]*work.d[51]*1;
  residual += temp*temp;
  temp = work.KKT[105]-work.L[0]*work.d[52]*1;
  residual += temp*temp;
  temp = work.KKT[107]-work.L[2]*work.d[53]*1;
  residual += temp*temp;
  temp = work.KKT[109]-work.L[4]*work.d[54]*1;
  residual += temp*temp;
  temp = work.KKT[111]-work.L[6]*work.d[55]*1;
  residual += temp*temp;
  temp = work.KKT[113]-work.L[8]*work.d[56]*1;
  residual += temp*temp;
  temp = work.KKT[115]-work.L[10]*work.d[57]*1;
  residual += temp*temp;
  temp = work.KKT[117]-work.L[1]*work.d[58]*1;
  residual += temp*temp;
  temp = work.KKT[119]-work.L[3]*work.d[59]*1;
  residual += temp*temp;
  temp = work.KKT[121]-work.L[5]*work.d[60]*1;
  residual += temp*temp;
  temp = work.KKT[123]-work.L[7]*work.d[61]*1;
  residual += temp*temp;
  temp = work.KKT[125]-work.L[9]*work.d[62]*1;
  residual += temp*temp;
  temp = work.KKT[127]-work.L[11]*work.d[63]*1;
  residual += temp*temp;
  temp = work.KKT[129]-work.L[12]*work.d[64]*1;
  residual += temp*temp;
  temp = work.KKT[131]-work.L[14]*work.d[65]*1;
  residual += temp*temp;
  temp = work.KKT[133]-work.L[16]*work.d[66]*1;
  residual += temp*temp;
  temp = work.KKT[135]-work.L[18]*work.d[67]*1;
  residual += temp*temp;
  temp = work.KKT[137]-work.L[20]*work.d[68]*1;
  residual += temp*temp;
  temp = work.KKT[139]-work.L[22]*work.d[69]*1;
  residual += temp*temp;
  temp = work.KKT[141]-work.L[13]*work.d[70]*1;
  residual += temp*temp;
  temp = work.KKT[143]-work.L[15]*work.d[71]*1;
  residual += temp*temp;
  temp = work.KKT[145]-work.L[17]*work.d[72]*1;
  residual += temp*temp;
  temp = work.KKT[147]-work.L[19]*work.d[73]*1;
  residual += temp*temp;
  temp = work.KKT[149]-work.L[21]*work.d[74]*1;
  residual += temp*temp;
  temp = work.KKT[151]-work.L[23]*work.d[75]*1;
  residual += temp*temp;
  temp = work.KKT[504]-work.L[414]*work.d[0]*work.L[414]-1*work.d[256]*1-work.L[457]*work.d[244]*work.L[457]-work.L[415]*work.d[130]*work.L[415]-work.L[418]*work.d[133]*work.L[418]-work.L[421]*work.d[136]*work.L[421]-work.L[424]*work.d[139]*work.L[424]-work.L[427]*work.d[142]*work.L[427]-work.L[430]*work.d[145]*work.L[430]-work.L[416]*work.d[131]*work.L[416]-work.L[417]*work.d[132]*work.L[417]-work.L[433]*work.d[181]*work.L[433]-work.L[419]*work.d[134]*work.L[419]-work.L[420]*work.d[135]*work.L[420]-work.L[436]*work.d[187]*work.L[436]-work.L[422]*work.d[137]*work.L[422]-work.L[423]*work.d[138]*work.L[423]-work.L[439]*work.d[193]*work.L[439]-work.L[425]*work.d[140]*work.L[425]-work.L[426]*work.d[141]*work.L[426]-work.L[442]*work.d[199]*work.L[442]-work.L[428]*work.d[143]*work.L[428]-work.L[429]*work.d[144]*work.L[429]-work.L[445]*work.d[205]*work.L[445]-work.L[431]*work.d[146]*work.L[431]-work.L[432]*work.d[147]*work.L[432]-work.L[448]*work.d[211]*work.L[448]-work.L[434]*work.d[182]*work.L[434]-work.L[435]*work.d[183]*work.L[435]-work.L[458]*work.d[245]*work.L[458]-work.L[451]*work.d[214]*work.L[451]-work.L[437]*work.d[188]*work.L[437]-work.L[438]*work.d[189]*work.L[438]-work.L[459]*work.d[246]*work.L[459]-work.L[452]*work.d[215]*work.L[452]-work.L[440]*work.d[194]*work.L[440]-work.L[441]*work.d[195]*work.L[441]-work.L[460]*work.d[247]*work.L[460]-work.L[453]*work.d[216]*work.L[453]-work.L[443]*work.d[200]*work.L[443]-work.L[444]*work.d[201]*work.L[444]-work.L[461]*work.d[248]*work.L[461]-work.L[454]*work.d[217]*work.L[454]-work.L[446]*work.d[206]*work.L[446]-work.L[447]*work.d[207]*work.L[447]-work.L[462]*work.d[249]*work.L[462]-work.L[455]*work.d[218]*work.L[455]-work.L[449]*work.d[212]*work.L[449]-work.L[450]*work.d[213]*work.L[450]-work.L[463]*work.d[250]*work.L[463]-work.L[456]*work.d[219]*work.L[456]-work.L[464]*work.d[251]*work.L[464]-work.L[465]*work.d[252]*work.L[465]-work.L[466]*work.d[253]*work.L[466]-work.L[467]*work.d[254]*work.L[467]-work.L[468]*work.d[255]*work.L[468];
  residual += temp*temp;
  temp = work.KKT[245]-work.L[48]*work.d[1]*work.L[48]-1*work.d[131]*1-work.L[49]*work.d[130]*work.L[49];
  residual += temp*temp;
  temp = work.KKT[252]-work.L[53]*work.d[2]*work.L[53]-1*work.d[134]*1-work.L[54]*work.d[133]*work.L[54];
  residual += temp*temp;
  temp = work.KKT[259]-work.L[58]*work.d[3]*work.L[58]-1*work.d[137]*1-work.L[59]*work.d[136]*work.L[59];
  residual += temp*temp;
  temp = work.KKT[266]-work.L[63]*work.d[4]*work.L[63]-1*work.d[140]*1-work.L[64]*work.d[139]*work.L[64];
  residual += temp*temp;
  temp = work.KKT[273]-work.L[68]*work.d[5]*work.L[68]-1*work.d[143]*1-work.L[69]*work.d[142]*work.L[69];
  residual += temp*temp;
  temp = work.KKT[280]-work.L[73]*work.d[6]*work.L[73]-1*work.d[146]*1-work.L[74]*work.d[145]*work.L[74];
  residual += temp*temp;
  temp = work.KKT[247]-work.L[50]*work.d[7]*work.L[50]-1*work.d[132]*1-work.L[51]*work.d[130]*work.L[51]-work.L[52]*work.d[131]*work.L[52];
  residual += temp*temp;
  temp = work.KKT[254]-work.L[55]*work.d[8]*work.L[55]-1*work.d[135]*1-work.L[56]*work.d[133]*work.L[56]-work.L[57]*work.d[134]*work.L[57];
  residual += temp*temp;
  temp = work.KKT[261]-work.L[60]*work.d[9]*work.L[60]-1*work.d[138]*1-work.L[61]*work.d[136]*work.L[61]-work.L[62]*work.d[137]*work.L[62];
  residual += temp*temp;
  temp = work.KKT[268]-work.L[65]*work.d[10]*work.L[65]-1*work.d[141]*1-work.L[66]*work.d[139]*work.L[66]-work.L[67]*work.d[140]*work.L[67];
  residual += temp*temp;
  temp = work.KKT[275]-work.L[70]*work.d[11]*work.L[70]-1*work.d[144]*1-work.L[71]*work.d[142]*work.L[71]-work.L[72]*work.d[143]*work.L[72];
  residual += temp*temp;
  temp = work.KKT[282]-work.L[75]*work.d[12]*work.L[75]-1*work.d[147]*1-work.L[76]*work.d[145]*work.L[76]-work.L[77]*work.d[146]*work.L[77];
  residual += temp*temp;
  temp = work.KKT[505]-work.L[469]*work.d[13]*work.L[469]-1*work.d[257]*1-work.L[512]*work.d[244]*work.L[512]-work.L[470]*work.d[148]*work.L[470]-work.L[471]*work.d[149]*work.L[471]-work.L[472]*work.d[150]*work.L[472]-work.L[473]*work.d[151]*work.L[473]-work.L[474]*work.d[152]*work.L[474]-work.L[475]*work.d[153]*work.L[475]-work.L[476]*work.d[179]*work.L[476]-work.L[477]*work.d[180]*work.L[477]-work.L[481]*work.d[185]*work.L[481]-work.L[482]*work.d[186]*work.L[482]-work.L[486]*work.d[191]*work.L[486]-work.L[487]*work.d[192]*work.L[487]-work.L[491]*work.d[197]*work.L[491]-work.L[492]*work.d[198]*work.L[492]-work.L[496]*work.d[203]*work.L[496]-work.L[497]*work.d[204]*work.L[497]-work.L[501]*work.d[209]*work.L[501]-work.L[502]*work.d[210]*work.L[502]-work.L[478]*work.d[181]*work.L[478]-work.L[479]*work.d[182]*work.L[479]-work.L[480]*work.d[183]*work.L[480]-work.L[524]*work.d[256]*work.L[524]-work.L[513]*work.d[245]*work.L[513]-work.L[506]*work.d[214]*work.L[506]-work.L[483]*work.d[187]*work.L[483]-work.L[484]*work.d[188]*work.L[484]-work.L[485]*work.d[189]*work.L[485]-work.L[514]*work.d[246]*work.L[514]-work.L[507]*work.d[215]*work.L[507]-work.L[488]*work.d[193]*work.L[488]-work.L[489]*work.d[194]*work.L[489]-work.L[490]*work.d[195]*work.L[490]-work.L[515]*work.d[247]*work.L[515]-work.L[508]*work.d[216]*work.L[508]-work.L[493]*work.d[199]*work.L[493]-work.L[494]*work.d[200]*work.L[494]-work.L[495]*work.d[201]*work.L[495]-work.L[516]*work.d[248]*work.L[516]-work.L[509]*work.d[217]*work.L[509]-work.L[498]*work.d[205]*work.L[498]-work.L[499]*work.d[206]*work.L[499]-work.L[500]*work.d[207]*work.L[500]-work.L[517]*work.d[249]*work.L[517]-work.L[510]*work.d[218]*work.L[510]-work.L[503]*work.d[211]*work.L[503]-work.L[504]*work.d[212]*work.L[504]-work.L[505]*work.d[213]*work.L[505]-work.L[518]*work.d[250]*work.L[518]-work.L[511]*work.d[219]*work.L[511]-work.L[519]*work.d[251]*work.L[519]-work.L[520]*work.d[252]*work.L[520]-work.L[521]*work.d[253]*work.L[521]-work.L[522]*work.d[254]*work.L[522]-work.L[523]*work.d[255]*work.L[523];
  residual += temp*temp;
  temp = work.KKT[366]-work.L[108]*work.d[14]*work.L[108]-1*work.d[179]*1-work.L[110]*work.d[178]*work.L[110]-work.L[109]*work.d[148]*work.L[109];
  residual += temp*temp;
  temp = work.KKT[377]-work.L[132]*work.d[15]*work.L[132]-1*work.d[185]*1-work.L[134]*work.d[184]*work.L[134]-work.L[133]*work.d[149]*work.L[133];
  residual += temp*temp;
  temp = work.KKT[388]-work.L[156]*work.d[16]*work.L[156]-1*work.d[191]*1-work.L[158]*work.d[190]*work.L[158]-work.L[157]*work.d[150]*work.L[157];
  residual += temp*temp;
  temp = work.KKT[399]-work.L[180]*work.d[17]*work.L[180]-1*work.d[197]*1-work.L[182]*work.d[196]*work.L[182]-work.L[181]*work.d[151]*work.L[181];
  residual += temp*temp;
  temp = work.KKT[410]-work.L[204]*work.d[18]*work.L[204]-1*work.d[203]*1-work.L[206]*work.d[202]*work.L[206]-work.L[205]*work.d[152]*work.L[205];
  residual += temp*temp;
  temp = work.KKT[421]-work.L[228]*work.d[19]*work.L[228]-1*work.d[209]*1-work.L[230]*work.d[208]*work.L[230]-work.L[229]*work.d[153]*work.L[229];
  residual += temp*temp;
  temp = work.KKT[368]-work.L[111]*work.d[20]*work.L[111]-1*work.d[180]*1-work.L[113]*work.d[178]*work.L[113]-work.L[112]*work.d[148]*work.L[112]-work.L[114]*work.d[179]*work.L[114];
  residual += temp*temp;
  temp = work.KKT[379]-work.L[135]*work.d[21]*work.L[135]-1*work.d[186]*1-work.L[137]*work.d[184]*work.L[137]-work.L[136]*work.d[149]*work.L[136]-work.L[138]*work.d[185]*work.L[138];
  residual += temp*temp;
  temp = work.KKT[390]-work.L[159]*work.d[22]*work.L[159]-1*work.d[192]*1-work.L[161]*work.d[190]*work.L[161]-work.L[160]*work.d[150]*work.L[160]-work.L[162]*work.d[191]*work.L[162];
  residual += temp*temp;
  temp = work.KKT[401]-work.L[183]*work.d[23]*work.L[183]-1*work.d[198]*1-work.L[185]*work.d[196]*work.L[185]-work.L[184]*work.d[151]*work.L[184]-work.L[186]*work.d[197]*work.L[186];
  residual += temp*temp;
  temp = work.KKT[412]-work.L[207]*work.d[24]*work.L[207]-1*work.d[204]*1-work.L[209]*work.d[202]*work.L[209]-work.L[208]*work.d[152]*work.L[208]-work.L[210]*work.d[203]*work.L[210];
  residual += temp*temp;
  temp = work.KKT[423]-work.L[231]*work.d[25]*work.L[231]-1*work.d[210]*1-work.L[233]*work.d[208]*work.L[233]-work.L[232]*work.d[153]*work.L[232]-work.L[234]*work.d[209]*work.L[234];
  residual += temp*temp;
  temp = work.KKT[506]-work.L[525]*work.d[26]*work.L[525]-1*work.d[258]*1-work.L[550]*work.d[244]*work.L[550]-work.L[526]*work.d[154]*work.L[526]-work.L[529]*work.d[157]*work.L[529]-work.L[532]*work.d[160]*work.L[532]-work.L[535]*work.d[163]*work.L[535]-work.L[538]*work.d[166]*work.L[538]-work.L[541]*work.d[169]*work.L[541]-work.L[527]*work.d[155]*work.L[527]-work.L[528]*work.d[156]*work.L[528]-work.L[544]*work.d[223]*work.L[544]-work.L[530]*work.d[158]*work.L[530]-work.L[531]*work.d[159]*work.L[531]-work.L[545]*work.d[227]*work.L[545]-work.L[533]*work.d[161]*work.L[533]-work.L[534]*work.d[162]*work.L[534]-work.L[546]*work.d[231]*work.L[546]-work.L[536]*work.d[164]*work.L[536]-work.L[537]*work.d[165]*work.L[537]-work.L[547]*work.d[235]*work.L[547]-work.L[539]*work.d[167]*work.L[539]-work.L[540]*work.d[168]*work.L[540]-work.L[548]*work.d[239]*work.L[548]-work.L[542]*work.d[170]*work.L[542]-work.L[543]*work.d[171]*work.L[543]-work.L[549]*work.d[243]*work.L[549]-work.L[551]*work.d[245]*work.L[551]-work.L[557]*work.d[251]*work.L[557]-work.L[552]*work.d[246]*work.L[552]-work.L[558]*work.d[252]*work.L[558]-work.L[553]*work.d[247]*work.L[553]-work.L[559]*work.d[253]*work.L[559]-work.L[554]*work.d[248]*work.L[554]-work.L[560]*work.d[254]*work.L[560]-work.L[555]*work.d[249]*work.L[555]-work.L[561]*work.d[255]*work.L[561]-work.L[556]*work.d[250]*work.L[556]-work.L[562]*work.d[256]*work.L[562]-work.L[563]*work.d[257]*work.L[563];
  residual += temp*temp;
  temp = work.KKT[305]-work.L[78]*work.d[27]*work.L[78]-1*work.d[155]*1-work.L[79]*work.d[154]*work.L[79];
  residual += temp*temp;
  temp = work.KKT[312]-work.L[83]*work.d[28]*work.L[83]-1*work.d[158]*1-work.L[84]*work.d[157]*work.L[84];
  residual += temp*temp;
  temp = work.KKT[319]-work.L[88]*work.d[29]*work.L[88]-1*work.d[161]*1-work.L[89]*work.d[160]*work.L[89];
  residual += temp*temp;
  temp = work.KKT[326]-work.L[93]*work.d[30]*work.L[93]-1*work.d[164]*1-work.L[94]*work.d[163]*work.L[94];
  residual += temp*temp;
  temp = work.KKT[333]-work.L[98]*work.d[31]*work.L[98]-1*work.d[167]*1-work.L[99]*work.d[166]*work.L[99];
  residual += temp*temp;
  temp = work.KKT[340]-work.L[103]*work.d[32]*work.L[103]-1*work.d[170]*1-work.L[104]*work.d[169]*work.L[104];
  residual += temp*temp;
  temp = work.KKT[307]-work.L[80]*work.d[33]*work.L[80]-1*work.d[156]*1-work.L[81]*work.d[154]*work.L[81]-work.L[82]*work.d[155]*work.L[82];
  residual += temp*temp;
  temp = work.KKT[314]-work.L[85]*work.d[34]*work.L[85]-1*work.d[159]*1-work.L[86]*work.d[157]*work.L[86]-work.L[87]*work.d[158]*work.L[87];
  residual += temp*temp;
  temp = work.KKT[321]-work.L[90]*work.d[35]*work.L[90]-1*work.d[162]*1-work.L[91]*work.d[160]*work.L[91]-work.L[92]*work.d[161]*work.L[92];
  residual += temp*temp;
  temp = work.KKT[328]-work.L[95]*work.d[36]*work.L[95]-1*work.d[165]*1-work.L[96]*work.d[163]*work.L[96]-work.L[97]*work.d[164]*work.L[97];
  residual += temp*temp;
  temp = work.KKT[335]-work.L[100]*work.d[37]*work.L[100]-1*work.d[168]*1-work.L[101]*work.d[166]*work.L[101]-work.L[102]*work.d[167]*work.L[102];
  residual += temp*temp;
  temp = work.KKT[342]-work.L[105]*work.d[38]*work.L[105]-1*work.d[171]*1-work.L[106]*work.d[169]*work.L[106]-work.L[107]*work.d[170]*work.L[107];
  residual += temp*temp;
  temp = work.KKT[507]-work.L[564]*work.d[39]*work.L[564]-1*work.d[259]*1-work.L[589]*work.d[244]*work.L[589]-work.L[565]*work.d[172]*work.L[565]-work.L[566]*work.d[173]*work.L[566]-work.L[567]*work.d[174]*work.L[567]-work.L[568]*work.d[175]*work.L[568]-work.L[569]*work.d[176]*work.L[569]-work.L[570]*work.d[177]*work.L[570]-work.L[571]*work.d[221]*work.L[571]-work.L[572]*work.d[222]*work.L[572]-work.L[574]*work.d[225]*work.L[574]-work.L[575]*work.d[226]*work.L[575]-work.L[577]*work.d[229]*work.L[577]-work.L[578]*work.d[230]*work.L[578]-work.L[580]*work.d[233]*work.L[580]-work.L[581]*work.d[234]*work.L[581]-work.L[583]*work.d[237]*work.L[583]-work.L[584]*work.d[238]*work.L[584]-work.L[586]*work.d[241]*work.L[586]-work.L[587]*work.d[242]*work.L[587]-work.L[573]*work.d[223]*work.L[573]-work.L[590]*work.d[245]*work.L[590]-work.L[596]*work.d[251]*work.L[596]-work.L[603]*work.d[258]*work.L[603]-work.L[576]*work.d[227]*work.L[576]-work.L[591]*work.d[246]*work.L[591]-work.L[597]*work.d[252]*work.L[597]-work.L[579]*work.d[231]*work.L[579]-work.L[592]*work.d[247]*work.L[592]-work.L[598]*work.d[253]*work.L[598]-work.L[582]*work.d[235]*work.L[582]-work.L[593]*work.d[248]*work.L[593]-work.L[599]*work.d[254]*work.L[599]-work.L[585]*work.d[239]*work.L[585]-work.L[594]*work.d[249]*work.L[594]-work.L[600]*work.d[255]*work.L[600]-work.L[588]*work.d[243]*work.L[588]-work.L[595]*work.d[250]*work.L[595]-work.L[601]*work.d[256]*work.L[601]-work.L[602]*work.d[257]*work.L[602];
  residual += temp*temp;
  temp = work.KKT[444]-work.L[270]*work.d[40]*work.L[270]-1*work.d[221]*1-work.L[272]*work.d[220]*work.L[272]-work.L[271]*work.d[172]*work.L[271];
  residual += temp*temp;
  temp = work.KKT[454]-work.L[281]*work.d[41]*work.L[281]-1*work.d[225]*1-work.L[283]*work.d[224]*work.L[283]-work.L[282]*work.d[173]*work.L[282];
  residual += temp*temp;
  temp = work.KKT[464]-work.L[292]*work.d[42]*work.L[292]-1*work.d[229]*1-work.L[294]*work.d[228]*work.L[294]-work.L[293]*work.d[174]*work.L[293];
  residual += temp*temp;
  temp = work.KKT[474]-work.L[303]*work.d[43]*work.L[303]-1*work.d[233]*1-work.L[305]*work.d[232]*work.L[305]-work.L[304]*work.d[175]*work.L[304];
  residual += temp*temp;
  temp = work.KKT[484]-work.L[314]*work.d[44]*work.L[314]-1*work.d[237]*1-work.L[316]*work.d[236]*work.L[316]-work.L[315]*work.d[176]*work.L[315];
  residual += temp*temp;
  temp = work.KKT[494]-work.L[325]*work.d[45]*work.L[325]-1*work.d[241]*1-work.L[327]*work.d[240]*work.L[327]-work.L[326]*work.d[177]*work.L[326];
  residual += temp*temp;
  temp = work.KKT[446]-work.L[273]*work.d[46]*work.L[273]-1*work.d[222]*1-work.L[275]*work.d[220]*work.L[275]-work.L[274]*work.d[172]*work.L[274]-work.L[276]*work.d[221]*work.L[276];
  residual += temp*temp;
  temp = work.KKT[456]-work.L[284]*work.d[47]*work.L[284]-1*work.d[226]*1-work.L[286]*work.d[224]*work.L[286]-work.L[285]*work.d[173]*work.L[285]-work.L[287]*work.d[225]*work.L[287];
  residual += temp*temp;
  temp = work.KKT[466]-work.L[295]*work.d[48]*work.L[295]-1*work.d[230]*1-work.L[297]*work.d[228]*work.L[297]-work.L[296]*work.d[174]*work.L[296]-work.L[298]*work.d[229]*work.L[298];
  residual += temp*temp;
  temp = work.KKT[476]-work.L[306]*work.d[49]*work.L[306]-1*work.d[234]*1-work.L[308]*work.d[232]*work.L[308]-work.L[307]*work.d[175]*work.L[307]-work.L[309]*work.d[233]*work.L[309];
  residual += temp*temp;
  temp = work.KKT[486]-work.L[317]*work.d[50]*work.L[317]-1*work.d[238]*1-work.L[319]*work.d[236]*work.L[319]-work.L[318]*work.d[176]*work.L[318]-work.L[320]*work.d[237]*work.L[320];
  residual += temp*temp;
  temp = work.KKT[496]-work.L[328]*work.d[51]*work.L[328]-1*work.d[242]*1-work.L[330]*work.d[240]*work.L[330]-work.L[329]*work.d[177]*work.L[329]-work.L[331]*work.d[241]*work.L[331];
  residual += temp*temp;
  temp = work.KKT[152]-work.L[0]*work.d[52]*work.L[0]-1*work.d[76]*1;
  residual += temp*temp;
  temp = work.KKT[156]-work.L[2]*work.d[53]*work.L[2]-1*work.d[78]*1;
  residual += temp*temp;
  temp = work.KKT[160]-work.L[4]*work.d[54]*work.L[4]-1*work.d[80]*1;
  residual += temp*temp;
  temp = work.KKT[164]-work.L[6]*work.d[55]*work.L[6]-1*work.d[82]*1;
  residual += temp*temp;
  temp = work.KKT[168]-work.L[8]*work.d[56]*work.L[8]-1*work.d[84]*1;
  residual += temp*temp;
  temp = work.KKT[172]-work.L[10]*work.d[57]*work.L[10]-1*work.d[86]*1;
  residual += temp*temp;
  temp = work.KKT[154]-work.L[1]*work.d[58]*work.L[1]-1*work.d[77]*1;
  residual += temp*temp;
  temp = work.KKT[158]-work.L[3]*work.d[59]*work.L[3]-1*work.d[79]*1;
  residual += temp*temp;
  temp = work.KKT[162]-work.L[5]*work.d[60]*work.L[5]-1*work.d[81]*1;
  residual += temp*temp;
  temp = work.KKT[166]-work.L[7]*work.d[61]*work.L[7]-1*work.d[83]*1;
  residual += temp*temp;
  temp = work.KKT[170]-work.L[9]*work.d[62]*work.L[9]-1*work.d[85]*1;
  residual += temp*temp;
  temp = work.KKT[174]-work.L[11]*work.d[63]*work.L[11]-1*work.d[87]*1;
  residual += temp*temp;
  temp = work.KKT[176]-work.L[12]*work.d[64]*work.L[12]-1*work.d[88]*1;
  residual += temp*temp;
  temp = work.KKT[180]-work.L[14]*work.d[65]*work.L[14]-1*work.d[90]*1;
  residual += temp*temp;
  temp = work.KKT[184]-work.L[16]*work.d[66]*work.L[16]-1*work.d[92]*1;
  residual += temp*temp;
  temp = work.KKT[188]-work.L[18]*work.d[67]*work.L[18]-1*work.d[94]*1;
  residual += temp*temp;
  temp = work.KKT[192]-work.L[20]*work.d[68]*work.L[20]-1*work.d[96]*1;
  residual += temp*temp;
  temp = work.KKT[196]-work.L[22]*work.d[69]*work.L[22]-1*work.d[98]*1;
  residual += temp*temp;
  temp = work.KKT[178]-work.L[13]*work.d[70]*work.L[13]-1*work.d[89]*1;
  residual += temp*temp;
  temp = work.KKT[182]-work.L[15]*work.d[71]*work.L[15]-1*work.d[91]*1;
  residual += temp*temp;
  temp = work.KKT[186]-work.L[17]*work.d[72]*work.L[17]-1*work.d[93]*1;
  residual += temp*temp;
  temp = work.KKT[190]-work.L[19]*work.d[73]*work.L[19]-1*work.d[95]*1;
  residual += temp*temp;
  temp = work.KKT[194]-work.L[21]*work.d[74]*work.L[21]-1*work.d[97]*1;
  residual += temp*temp;
  temp = work.KKT[198]-work.L[23]*work.d[75]*work.L[23]-1*work.d[99]*1;
  residual += temp*temp;
  temp = work.KKT[500]-work.L[457]*work.d[244]*1;
  residual += temp*temp;
  temp = work.KKT[244]-work.L[415]*work.d[130]*1;
  residual += temp*temp;
  temp = work.KKT[251]-work.L[418]*work.d[133]*1;
  residual += temp*temp;
  temp = work.KKT[258]-work.L[421]*work.d[136]*1;
  residual += temp*temp;
  temp = work.KKT[265]-work.L[424]*work.d[139]*1;
  residual += temp*temp;
  temp = work.KKT[272]-work.L[427]*work.d[142]*1;
  residual += temp*temp;
  temp = work.KKT[279]-work.L[430]*work.d[145]*1;
  residual += temp*temp;
  temp = work.KKT[246]-1*work.d[131]*work.L[115];
  residual += temp*temp;
  temp = work.KKT[253]-1*work.d[134]*work.L[139];
  residual += temp*temp;
  temp = work.KKT[260]-1*work.d[137]*work.L[163];
  residual += temp*temp;
  temp = work.KKT[267]-1*work.d[140]*work.L[187];
  residual += temp*temp;
  temp = work.KKT[274]-1*work.d[143]*work.L[211];
  residual += temp*temp;
  temp = work.KKT[281]-1*work.d[146]*work.L[235];
  residual += temp*temp;
  temp = work.KKT[242]-work.L[49]*work.d[130]*1;
  residual += temp*temp;
  temp = work.KKT[249]-work.L[54]*work.d[133]*1;
  residual += temp*temp;
  temp = work.KKT[256]-work.L[59]*work.d[136]*1;
  residual += temp*temp;
  temp = work.KKT[263]-work.L[64]*work.d[139]*1;
  residual += temp*temp;
  temp = work.KKT[270]-work.L[69]*work.d[142]*1;
  residual += temp*temp;
  temp = work.KKT[277]-work.L[74]*work.d[145]*1;
  residual += temp*temp;
  temp = work.KKT[248]-1*work.d[132]*work.L[116]-work.L[52]*work.d[131]*work.L[115];
  residual += temp*temp;
  temp = work.KKT[255]-1*work.d[135]*work.L[140]-work.L[57]*work.d[134]*work.L[139];
  residual += temp*temp;
  temp = work.KKT[262]-1*work.d[138]*work.L[164]-work.L[62]*work.d[137]*work.L[163];
  residual += temp*temp;
  temp = work.KKT[269]-1*work.d[141]*work.L[188]-work.L[67]*work.d[140]*work.L[187];
  residual += temp*temp;
  temp = work.KKT[276]-1*work.d[144]*work.L[212]-work.L[72]*work.d[143]*work.L[211];
  residual += temp*temp;
  temp = work.KKT[283]-1*work.d[147]*work.L[236]-work.L[77]*work.d[146]*work.L[235];
  residual += temp*temp;
  temp = work.KKT[243]-work.L[51]*work.d[130]*1;
  residual += temp*temp;
  temp = work.KKT[250]-work.L[56]*work.d[133]*1;
  residual += temp*temp;
  temp = work.KKT[257]-work.L[61]*work.d[136]*1;
  residual += temp*temp;
  temp = work.KKT[264]-work.L[66]*work.d[139]*1;
  residual += temp*temp;
  temp = work.KKT[271]-work.L[71]*work.d[142]*1;
  residual += temp*temp;
  temp = work.KKT[278]-work.L[76]*work.d[145]*1;
  residual += temp*temp;
  temp = work.KKT[501]-work.L[512]*work.d[244]*1;
  residual += temp*temp;
  temp = work.KKT[286]-work.L[470]*work.d[148]*1;
  residual += temp*temp;
  temp = work.KKT[289]-work.L[471]*work.d[149]*1;
  residual += temp*temp;
  temp = work.KKT[292]-work.L[472]*work.d[150]*1;
  residual += temp*temp;
  temp = work.KKT[295]-work.L[473]*work.d[151]*1;
  residual += temp*temp;
  temp = work.KKT[298]-work.L[474]*work.d[152]*1;
  residual += temp*temp;
  temp = work.KKT[301]-work.L[475]*work.d[153]*1;
  residual += temp*temp;
  temp = work.KKT[367]-1*work.d[179]*work.L[117];
  residual += temp*temp;
  temp = work.KKT[378]-1*work.d[185]*work.L[141];
  residual += temp*temp;
  temp = work.KKT[389]-1*work.d[191]*work.L[165];
  residual += temp*temp;
  temp = work.KKT[400]-1*work.d[197]*work.L[189];
  residual += temp*temp;
  temp = work.KKT[411]-1*work.d[203]*work.L[213];
  residual += temp*temp;
  temp = work.KKT[422]-1*work.d[209]*work.L[237];
  residual += temp*temp;
  temp = work.KKT[362]-work.L[110]*work.d[178]*1;
  residual += temp*temp;
  temp = work.KKT[373]-work.L[134]*work.d[184]*1;
  residual += temp*temp;
  temp = work.KKT[384]-work.L[158]*work.d[190]*1;
  residual += temp*temp;
  temp = work.KKT[395]-work.L[182]*work.d[196]*1;
  residual += temp*temp;
  temp = work.KKT[406]-work.L[206]*work.d[202]*1;
  residual += temp*temp;
  temp = work.KKT[417]-work.L[230]*work.d[208]*1;
  residual += temp*temp;
  temp = work.KKT[284]-work.L[109]*work.d[148]*1;
  residual += temp*temp;
  temp = work.KKT[287]-work.L[133]*work.d[149]*1;
  residual += temp*temp;
  temp = work.KKT[290]-work.L[157]*work.d[150]*1;
  residual += temp*temp;
  temp = work.KKT[293]-work.L[181]*work.d[151]*1;
  residual += temp*temp;
  temp = work.KKT[296]-work.L[205]*work.d[152]*1;
  residual += temp*temp;
  temp = work.KKT[299]-work.L[229]*work.d[153]*1;
  residual += temp*temp;
  temp = work.KKT[369]-1*work.d[180]*work.L[118]-work.L[114]*work.d[179]*work.L[117];
  residual += temp*temp;
  temp = work.KKT[380]-1*work.d[186]*work.L[142]-work.L[138]*work.d[185]*work.L[141];
  residual += temp*temp;
  temp = work.KKT[391]-1*work.d[192]*work.L[166]-work.L[162]*work.d[191]*work.L[165];
  residual += temp*temp;
  temp = work.KKT[402]-1*work.d[198]*work.L[190]-work.L[186]*work.d[197]*work.L[189];
  residual += temp*temp;
  temp = work.KKT[413]-1*work.d[204]*work.L[214]-work.L[210]*work.d[203]*work.L[213];
  residual += temp*temp;
  temp = work.KKT[424]-1*work.d[210]*work.L[238]-work.L[234]*work.d[209]*work.L[237];
  residual += temp*temp;
  temp = work.KKT[363]-work.L[113]*work.d[178]*1;
  residual += temp*temp;
  temp = work.KKT[374]-work.L[137]*work.d[184]*1;
  residual += temp*temp;
  temp = work.KKT[385]-work.L[161]*work.d[190]*1;
  residual += temp*temp;
  temp = work.KKT[396]-work.L[185]*work.d[196]*1;
  residual += temp*temp;
  temp = work.KKT[407]-work.L[209]*work.d[202]*1;
  residual += temp*temp;
  temp = work.KKT[418]-work.L[233]*work.d[208]*1;
  residual += temp*temp;
  temp = work.KKT[285]-work.L[112]*work.d[148]*1;
  residual += temp*temp;
  temp = work.KKT[288]-work.L[136]*work.d[149]*1;
  residual += temp*temp;
  temp = work.KKT[291]-work.L[160]*work.d[150]*1;
  residual += temp*temp;
  temp = work.KKT[294]-work.L[184]*work.d[151]*1;
  residual += temp*temp;
  temp = work.KKT[297]-work.L[208]*work.d[152]*1;
  residual += temp*temp;
  temp = work.KKT[300]-work.L[232]*work.d[153]*1;
  residual += temp*temp;
  temp = work.KKT[502]-work.L[550]*work.d[244]*1;
  residual += temp*temp;
  temp = work.KKT[304]-work.L[526]*work.d[154]*1;
  residual += temp*temp;
  temp = work.KKT[311]-work.L[529]*work.d[157]*1;
  residual += temp*temp;
  temp = work.KKT[318]-work.L[532]*work.d[160]*1;
  residual += temp*temp;
  temp = work.KKT[325]-work.L[535]*work.d[163]*1;
  residual += temp*temp;
  temp = work.KKT[332]-work.L[538]*work.d[166]*1;
  residual += temp*temp;
  temp = work.KKT[339]-work.L[541]*work.d[169]*1;
  residual += temp*temp;
  temp = work.KKT[306]-1*work.d[155]*work.L[277];
  residual += temp*temp;
  temp = work.KKT[313]-1*work.d[158]*work.L[288];
  residual += temp*temp;
  temp = work.KKT[320]-1*work.d[161]*work.L[299];
  residual += temp*temp;
  temp = work.KKT[327]-1*work.d[164]*work.L[310];
  residual += temp*temp;
  temp = work.KKT[334]-1*work.d[167]*work.L[321];
  residual += temp*temp;
  temp = work.KKT[341]-1*work.d[170]*work.L[332];
  residual += temp*temp;
  temp = work.KKT[302]-work.L[79]*work.d[154]*1;
  residual += temp*temp;
  temp = work.KKT[309]-work.L[84]*work.d[157]*1;
  residual += temp*temp;
  temp = work.KKT[316]-work.L[89]*work.d[160]*1;
  residual += temp*temp;
  temp = work.KKT[323]-work.L[94]*work.d[163]*1;
  residual += temp*temp;
  temp = work.KKT[330]-work.L[99]*work.d[166]*1;
  residual += temp*temp;
  temp = work.KKT[337]-work.L[104]*work.d[169]*1;
  residual += temp*temp;
  temp = work.KKT[308]-1*work.d[156]*work.L[278]-work.L[82]*work.d[155]*work.L[277];
  residual += temp*temp;
  temp = work.KKT[315]-1*work.d[159]*work.L[289]-work.L[87]*work.d[158]*work.L[288];
  residual += temp*temp;
  temp = work.KKT[322]-1*work.d[162]*work.L[300]-work.L[92]*work.d[161]*work.L[299];
  residual += temp*temp;
  temp = work.KKT[329]-1*work.d[165]*work.L[311]-work.L[97]*work.d[164]*work.L[310];
  residual += temp*temp;
  temp = work.KKT[336]-1*work.d[168]*work.L[322]-work.L[102]*work.d[167]*work.L[321];
  residual += temp*temp;
  temp = work.KKT[343]-1*work.d[171]*work.L[333]-work.L[107]*work.d[170]*work.L[332];
  residual += temp*temp;
  temp = work.KKT[303]-work.L[81]*work.d[154]*1;
  residual += temp*temp;
  temp = work.KKT[310]-work.L[86]*work.d[157]*1;
  residual += temp*temp;
  temp = work.KKT[317]-work.L[91]*work.d[160]*1;
  residual += temp*temp;
  temp = work.KKT[324]-work.L[96]*work.d[163]*1;
  residual += temp*temp;
  temp = work.KKT[331]-work.L[101]*work.d[166]*1;
  residual += temp*temp;
  temp = work.KKT[338]-work.L[106]*work.d[169]*1;
  residual += temp*temp;
  temp = work.KKT[503]-work.L[589]*work.d[244]*1;
  residual += temp*temp;
  temp = work.KKT[346]-work.L[565]*work.d[172]*1;
  residual += temp*temp;
  temp = work.KKT[349]-work.L[566]*work.d[173]*1;
  residual += temp*temp;
  temp = work.KKT[352]-work.L[567]*work.d[174]*1;
  residual += temp*temp;
  temp = work.KKT[355]-work.L[568]*work.d[175]*1;
  residual += temp*temp;
  temp = work.KKT[358]-work.L[569]*work.d[176]*1;
  residual += temp*temp;
  temp = work.KKT[361]-work.L[570]*work.d[177]*1;
  residual += temp*temp;
  temp = work.KKT[445]-1*work.d[221]*work.L[279];
  residual += temp*temp;
  temp = work.KKT[455]-1*work.d[225]*work.L[290];
  residual += temp*temp;
  temp = work.KKT[465]-1*work.d[229]*work.L[301];
  residual += temp*temp;
  temp = work.KKT[475]-1*work.d[233]*work.L[312];
  residual += temp*temp;
  temp = work.KKT[485]-1*work.d[237]*work.L[323];
  residual += temp*temp;
  temp = work.KKT[495]-1*work.d[241]*work.L[334];
  residual += temp*temp;
  temp = work.KKT[440]-work.L[272]*work.d[220]*1;
  residual += temp*temp;
  temp = work.KKT[450]-work.L[283]*work.d[224]*1;
  residual += temp*temp;
  temp = work.KKT[460]-work.L[294]*work.d[228]*1;
  residual += temp*temp;
  temp = work.KKT[470]-work.L[305]*work.d[232]*1;
  residual += temp*temp;
  temp = work.KKT[480]-work.L[316]*work.d[236]*1;
  residual += temp*temp;
  temp = work.KKT[490]-work.L[327]*work.d[240]*1;
  residual += temp*temp;
  temp = work.KKT[344]-work.L[271]*work.d[172]*1;
  residual += temp*temp;
  temp = work.KKT[347]-work.L[282]*work.d[173]*1;
  residual += temp*temp;
  temp = work.KKT[350]-work.L[293]*work.d[174]*1;
  residual += temp*temp;
  temp = work.KKT[353]-work.L[304]*work.d[175]*1;
  residual += temp*temp;
  temp = work.KKT[356]-work.L[315]*work.d[176]*1;
  residual += temp*temp;
  temp = work.KKT[359]-work.L[326]*work.d[177]*1;
  residual += temp*temp;
  temp = work.KKT[447]-1*work.d[222]*work.L[280]-work.L[276]*work.d[221]*work.L[279];
  residual += temp*temp;
  temp = work.KKT[457]-1*work.d[226]*work.L[291]-work.L[287]*work.d[225]*work.L[290];
  residual += temp*temp;
  temp = work.KKT[467]-1*work.d[230]*work.L[302]-work.L[298]*work.d[229]*work.L[301];
  residual += temp*temp;
  temp = work.KKT[477]-1*work.d[234]*work.L[313]-work.L[309]*work.d[233]*work.L[312];
  residual += temp*temp;
  temp = work.KKT[487]-1*work.d[238]*work.L[324]-work.L[320]*work.d[237]*work.L[323];
  residual += temp*temp;
  temp = work.KKT[497]-1*work.d[242]*work.L[335]-work.L[331]*work.d[241]*work.L[334];
  residual += temp*temp;
  temp = work.KKT[441]-work.L[275]*work.d[220]*1;
  residual += temp*temp;
  temp = work.KKT[451]-work.L[286]*work.d[224]*1;
  residual += temp*temp;
  temp = work.KKT[461]-work.L[297]*work.d[228]*1;
  residual += temp*temp;
  temp = work.KKT[471]-work.L[308]*work.d[232]*1;
  residual += temp*temp;
  temp = work.KKT[481]-work.L[319]*work.d[236]*1;
  residual += temp*temp;
  temp = work.KKT[491]-work.L[330]*work.d[240]*1;
  residual += temp*temp;
  temp = work.KKT[345]-work.L[274]*work.d[172]*1;
  residual += temp*temp;
  temp = work.KKT[348]-work.L[285]*work.d[173]*1;
  residual += temp*temp;
  temp = work.KKT[351]-work.L[296]*work.d[174]*1;
  residual += temp*temp;
  temp = work.KKT[354]-work.L[307]*work.d[175]*1;
  residual += temp*temp;
  temp = work.KKT[357]-work.L[318]*work.d[176]*1;
  residual += temp*temp;
  temp = work.KKT[360]-work.L[329]*work.d[177]*1;
  residual += temp*temp;
  temp = work.KKT[153]-1*work.d[76]*work.L[36];
  residual += temp*temp;
  temp = work.KKT[157]-1*work.d[78]*work.L[38];
  residual += temp*temp;
  temp = work.KKT[161]-1*work.d[80]*work.L[40];
  residual += temp*temp;
  temp = work.KKT[165]-1*work.d[82]*work.L[42];
  residual += temp*temp;
  temp = work.KKT[169]-1*work.d[84]*work.L[44];
  residual += temp*temp;
  temp = work.KKT[173]-1*work.d[86]*work.L[46];
  residual += temp*temp;
  temp = work.KKT[155]-1*work.d[77]*work.L[37];
  residual += temp*temp;
  temp = work.KKT[159]-1*work.d[79]*work.L[39];
  residual += temp*temp;
  temp = work.KKT[163]-1*work.d[81]*work.L[41];
  residual += temp*temp;
  temp = work.KKT[167]-1*work.d[83]*work.L[43];
  residual += temp*temp;
  temp = work.KKT[171]-1*work.d[85]*work.L[45];
  residual += temp*temp;
  temp = work.KKT[175]-1*work.d[87]*work.L[47];
  residual += temp*temp;
  temp = work.KKT[177]-1*work.d[88]*work.L[252];
  residual += temp*temp;
  temp = work.KKT[181]-1*work.d[90]*work.L[255];
  residual += temp*temp;
  temp = work.KKT[185]-1*work.d[92]*work.L[258];
  residual += temp*temp;
  temp = work.KKT[189]-1*work.d[94]*work.L[261];
  residual += temp*temp;
  temp = work.KKT[193]-1*work.d[96]*work.L[264];
  residual += temp*temp;
  temp = work.KKT[197]-1*work.d[98]*work.L[267];
  residual += temp*temp;
  temp = work.KKT[179]-1*work.d[89]*work.L[253];
  residual += temp*temp;
  temp = work.KKT[183]-1*work.d[91]*work.L[256];
  residual += temp*temp;
  temp = work.KKT[187]-1*work.d[93]*work.L[259];
  residual += temp*temp;
  temp = work.KKT[191]-1*work.d[95]*work.L[262];
  residual += temp*temp;
  temp = work.KKT[195]-1*work.d[97]*work.L[265];
  residual += temp*temp;
  temp = work.KKT[199]-1*work.d[99]*work.L[268];
  residual += temp*temp;
  temp = work.KKT[201]-work.L[119]*work.d[101]*1;
  residual += temp*temp;
  temp = work.KKT[203]-work.L[143]*work.d[103]*1;
  residual += temp*temp;
  temp = work.KKT[205]-work.L[167]*work.d[105]*1;
  residual += temp*temp;
  temp = work.KKT[207]-work.L[191]*work.d[107]*1;
  residual += temp*temp;
  temp = work.KKT[209]-work.L[215]*work.d[109]*1;
  residual += temp*temp;
  temp = work.KKT[211]-work.L[239]*work.d[111]*1;
  residual += temp*temp;
  temp = work.KKT[218]-work.L[120]*work.d[118]*1;
  residual += temp*temp;
  temp = work.KKT[220]-work.L[144]*work.d[119]*1;
  residual += temp*temp;
  temp = work.KKT[222]-work.L[168]*work.d[120]*1;
  residual += temp*temp;
  temp = work.KKT[224]-work.L[192]*work.d[121]*1;
  residual += temp*temp;
  temp = work.KKT[226]-work.L[216]*work.d[122]*1;
  residual += temp*temp;
  temp = work.KKT[228]-work.L[240]*work.d[123]*1;
  residual += temp*temp;
  temp = work.KKT[370]-work.L[125]*work.d[181]*1-work.L[123]*work.d[179]*work.L[117]-work.L[124]*work.d[180]*work.L[118];
  residual += temp*temp;
  temp = work.KKT[381]-work.L[149]*work.d[187]*1-work.L[147]*work.d[185]*work.L[141]-work.L[148]*work.d[186]*work.L[142];
  residual += temp*temp;
  temp = work.KKT[392]-work.L[173]*work.d[193]*1-work.L[171]*work.d[191]*work.L[165]-work.L[172]*work.d[192]*work.L[166];
  residual += temp*temp;
  temp = work.KKT[403]-work.L[197]*work.d[199]*1-work.L[195]*work.d[197]*work.L[189]-work.L[196]*work.d[198]*work.L[190];
  residual += temp*temp;
  temp = work.KKT[414]-work.L[221]*work.d[205]*1-work.L[219]*work.d[203]*work.L[213]-work.L[220]*work.d[204]*work.L[214];
  residual += temp*temp;
  temp = work.KKT[425]-work.L[245]*work.d[211]*1-work.L[243]*work.d[209]*work.L[237]-work.L[244]*work.d[210]*work.L[238];
  residual += temp*temp;
  temp = work.KKT[364]-work.L[122]*work.d[178]*1;
  residual += temp*temp;
  temp = work.KKT[375]-work.L[146]*work.d[184]*1;
  residual += temp*temp;
  temp = work.KKT[386]-work.L[170]*work.d[190]*1;
  residual += temp*temp;
  temp = work.KKT[397]-work.L[194]*work.d[196]*1;
  residual += temp*temp;
  temp = work.KKT[408]-work.L[218]*work.d[202]*1;
  residual += temp*temp;
  temp = work.KKT[419]-work.L[242]*work.d[208]*1;
  residual += temp*temp;
  temp = work.KKT[230]-work.L[121]*work.d[124]*1;
  residual += temp*temp;
  temp = work.KKT[232]-work.L[145]*work.d[125]*1;
  residual += temp*temp;
  temp = work.KKT[234]-work.L[169]*work.d[126]*1;
  residual += temp*temp;
  temp = work.KKT[236]-work.L[193]*work.d[127]*1;
  residual += temp*temp;
  temp = work.KKT[238]-work.L[217]*work.d[128]*1;
  residual += temp*temp;
  temp = work.KKT[240]-work.L[241]*work.d[129]*1;
  residual += temp*temp;
  temp = work.KKT[219]-work.L[126]*work.d[118]*1;
  residual += temp*temp;
  temp = work.KKT[221]-work.L[150]*work.d[119]*1;
  residual += temp*temp;
  temp = work.KKT[223]-work.L[174]*work.d[120]*1;
  residual += temp*temp;
  temp = work.KKT[225]-work.L[198]*work.d[121]*1;
  residual += temp*temp;
  temp = work.KKT[227]-work.L[222]*work.d[122]*1;
  residual += temp*temp;
  temp = work.KKT[229]-work.L[246]*work.d[123]*1;
  residual += temp*temp;
  temp = work.KKT[371]-work.L[130]*work.d[181]*1-work.L[128]*work.d[179]*work.L[117]-work.L[129]*work.d[180]*work.L[118];
  residual += temp*temp;
  temp = work.KKT[382]-work.L[154]*work.d[187]*1-work.L[152]*work.d[185]*work.L[141]-work.L[153]*work.d[186]*work.L[142];
  residual += temp*temp;
  temp = work.KKT[393]-work.L[178]*work.d[193]*1-work.L[176]*work.d[191]*work.L[165]-work.L[177]*work.d[192]*work.L[166];
  residual += temp*temp;
  temp = work.KKT[404]-work.L[202]*work.d[199]*1-work.L[200]*work.d[197]*work.L[189]-work.L[201]*work.d[198]*work.L[190];
  residual += temp*temp;
  temp = work.KKT[415]-work.L[226]*work.d[205]*1-work.L[224]*work.d[203]*work.L[213]-work.L[225]*work.d[204]*work.L[214];
  residual += temp*temp;
  temp = work.KKT[426]-work.L[250]*work.d[211]*1-work.L[248]*work.d[209]*work.L[237]-work.L[249]*work.d[210]*work.L[238];
  residual += temp*temp;
  temp = work.KKT[365]-work.L[127]*work.d[178]*1;
  residual += temp*temp;
  temp = work.KKT[376]-work.L[151]*work.d[184]*1;
  residual += temp*temp;
  temp = work.KKT[387]-work.L[175]*work.d[190]*1;
  residual += temp*temp;
  temp = work.KKT[398]-work.L[199]*work.d[196]*1;
  residual += temp*temp;
  temp = work.KKT[409]-work.L[223]*work.d[202]*1;
  residual += temp*temp;
  temp = work.KKT[420]-work.L[247]*work.d[208]*1;
  residual += temp*temp;
  temp = work.KKT[372]-1*work.d[183]*work.L[254];
  residual += temp*temp;
  temp = work.KKT[383]-1*work.d[189]*work.L[257];
  residual += temp*temp;
  temp = work.KKT[394]-1*work.d[195]*work.L[260];
  residual += temp*temp;
  temp = work.KKT[405]-1*work.d[201]*work.L[263];
  residual += temp*temp;
  temp = work.KKT[416]-1*work.d[207]*work.L[266];
  residual += temp*temp;
  temp = work.KKT[427]-1*work.d[213]*work.L[269];
  residual += temp*temp;
  temp = work.KKT[200]-1*work.d[100]*work.L[24];
  residual += temp*temp;
  temp = work.KKT[202]-1*work.d[102]*work.L[25];
  residual += temp*temp;
  temp = work.KKT[204]-1*work.d[104]*work.L[26];
  residual += temp*temp;
  temp = work.KKT[206]-1*work.d[106]*work.L[27];
  residual += temp*temp;
  temp = work.KKT[208]-1*work.d[108]*work.L[28];
  residual += temp*temp;
  temp = work.KKT[210]-1*work.d[110]*work.L[29];
  residual += temp*temp;
  temp = work.KKT[212]-1*work.d[112]*work.L[30];
  residual += temp*temp;
  temp = work.KKT[213]-1*work.d[113]*work.L[31];
  residual += temp*temp;
  temp = work.KKT[214]-1*work.d[114]*work.L[32];
  residual += temp*temp;
  temp = work.KKT[215]-1*work.d[115]*work.L[33];
  residual += temp*temp;
  temp = work.KKT[216]-1*work.d[116]*work.L[34];
  residual += temp*temp;
  temp = work.KKT[217]-1*work.d[117]*work.L[35];
  residual += temp*temp;
  temp = work.KKT[231]-work.L[336]*work.d[124]*1;
  residual += temp*temp;
  temp = work.KKT[233]-work.L[344]*work.d[125]*1;
  residual += temp*temp;
  temp = work.KKT[235]-work.L[352]*work.d[126]*1;
  residual += temp*temp;
  temp = work.KKT[237]-work.L[360]*work.d[127]*1;
  residual += temp*temp;
  temp = work.KKT[239]-work.L[368]*work.d[128]*1;
  residual += temp*temp;
  temp = work.KKT[241]-work.L[376]*work.d[129]*1;
  residual += temp*temp;
  temp = work.KKT[428]-work.L[339]*work.d[214]*1-work.L[338]*work.d[183]*work.L[254];
  residual += temp*temp;
  temp = work.KKT[430]-work.L[347]*work.d[215]*1-work.L[346]*work.d[189]*work.L[257];
  residual += temp*temp;
  temp = work.KKT[432]-work.L[355]*work.d[216]*1-work.L[354]*work.d[195]*work.L[260];
  residual += temp*temp;
  temp = work.KKT[434]-work.L[363]*work.d[217]*1-work.L[362]*work.d[201]*work.L[263];
  residual += temp*temp;
  temp = work.KKT[436]-work.L[371]*work.d[218]*1-work.L[370]*work.d[207]*work.L[266];
  residual += temp*temp;
  temp = work.KKT[438]-work.L[379]*work.d[219]*1-work.L[378]*work.d[213]*work.L[269];
  residual += temp*temp;
  temp = work.KKT[448]-work.L[343]*work.d[223]*1-work.L[341]*work.d[221]*work.L[279]-work.L[342]*work.d[222]*work.L[280];
  residual += temp*temp;
  temp = work.KKT[458]-work.L[351]*work.d[227]*1-work.L[349]*work.d[225]*work.L[290]-work.L[350]*work.d[226]*work.L[291];
  residual += temp*temp;
  temp = work.KKT[468]-work.L[359]*work.d[231]*1-work.L[357]*work.d[229]*work.L[301]-work.L[358]*work.d[230]*work.L[302];
  residual += temp*temp;
  temp = work.KKT[478]-work.L[367]*work.d[235]*1-work.L[365]*work.d[233]*work.L[312]-work.L[366]*work.d[234]*work.L[313];
  residual += temp*temp;
  temp = work.KKT[488]-work.L[375]*work.d[239]*1-work.L[373]*work.d[237]*work.L[323]-work.L[374]*work.d[238]*work.L[324];
  residual += temp*temp;
  temp = work.KKT[498]-work.L[383]*work.d[243]*1-work.L[381]*work.d[241]*work.L[334]-work.L[382]*work.d[242]*work.L[335];
  residual += temp*temp;
  temp = work.KKT[442]-work.L[340]*work.d[220]*1;
  residual += temp*temp;
  temp = work.KKT[452]-work.L[348]*work.d[224]*1;
  residual += temp*temp;
  temp = work.KKT[462]-work.L[356]*work.d[228]*1;
  residual += temp*temp;
  temp = work.KKT[472]-work.L[364]*work.d[232]*1;
  residual += temp*temp;
  temp = work.KKT[482]-work.L[372]*work.d[236]*1;
  residual += temp*temp;
  temp = work.KKT[492]-work.L[380]*work.d[240]*1;
  residual += temp*temp;
  temp = work.KKT[429]-work.L[384]*work.d[214]*1;
  residual += temp*temp;
  temp = work.KKT[431]-work.L[390]*work.d[215]*1;
  residual += temp*temp;
  temp = work.KKT[433]-work.L[396]*work.d[216]*1;
  residual += temp*temp;
  temp = work.KKT[435]-work.L[402]*work.d[217]*1;
  residual += temp*temp;
  temp = work.KKT[437]-work.L[408]*work.d[218]*1;
  residual += temp*temp;
  temp = work.KKT[439]-work.L[604]*work.d[219]*1;
  residual += temp*temp;
  temp = work.KKT[449]-work.L[388]*work.d[223]*1-work.L[386]*work.d[221]*work.L[279]-work.L[387]*work.d[222]*work.L[280];
  residual += temp*temp;
  temp = work.KKT[459]-work.L[394]*work.d[227]*1-work.L[392]*work.d[225]*work.L[290]-work.L[393]*work.d[226]*work.L[291];
  residual += temp*temp;
  temp = work.KKT[469]-work.L[400]*work.d[231]*1-work.L[398]*work.d[229]*work.L[301]-work.L[399]*work.d[230]*work.L[302];
  residual += temp*temp;
  temp = work.KKT[479]-work.L[406]*work.d[235]*1-work.L[404]*work.d[233]*work.L[312]-work.L[405]*work.d[234]*work.L[313];
  residual += temp*temp;
  temp = work.KKT[489]-work.L[412]*work.d[239]*1-work.L[410]*work.d[237]*work.L[323]-work.L[411]*work.d[238]*work.L[324];
  residual += temp*temp;
  temp = work.KKT[499]-work.L[608]*work.d[243]*1-work.L[606]*work.d[241]*work.L[334]-work.L[607]*work.d[242]*work.L[335];
  residual += temp*temp;
  temp = work.KKT[443]-work.L[385]*work.d[220]*1;
  residual += temp*temp;
  temp = work.KKT[453]-work.L[391]*work.d[224]*1;
  residual += temp*temp;
  temp = work.KKT[463]-work.L[397]*work.d[228]*1;
  residual += temp*temp;
  temp = work.KKT[473]-work.L[403]*work.d[232]*1;
  residual += temp*temp;
  temp = work.KKT[483]-work.L[409]*work.d[236]*1;
  residual += temp*temp;
  temp = work.KKT[493]-work.L[605]*work.d[240]*1;
  residual += temp*temp;
  return residual;
}
void matrix_multiply(double *result, double *source) {
  /* Finds result = A*source. */
  result[0] = work.KKT[201]*source[225]+work.KKT[200]*source[237];
  result[1] = work.KKT[203]*source[226]+work.KKT[202]*source[238];
  result[2] = work.KKT[205]*source[227]+work.KKT[204]*source[239];
  result[3] = work.KKT[207]*source[228]+work.KKT[206]*source[240];
  result[4] = work.KKT[209]*source[229]+work.KKT[208]*source[241];
  result[5] = work.KKT[211]*source[230]+work.KKT[210]*source[242];
  result[6] = work.KKT[218]*source[225]+work.KKT[219]*source[231]+work.KKT[212]*source[243];
  result[7] = work.KKT[220]*source[226]+work.KKT[221]*source[232]+work.KKT[213]*source[244];
  result[8] = work.KKT[222]*source[227]+work.KKT[223]*source[233]+work.KKT[214]*source[245];
  result[9] = work.KKT[224]*source[228]+work.KKT[225]*source[234]+work.KKT[215]*source[246];
  result[10] = work.KKT[226]*source[229]+work.KKT[227]*source[235]+work.KKT[216]*source[247];
  result[11] = work.KKT[228]*source[230]+work.KKT[229]*source[236]+work.KKT[217]*source[248];
  result[12] = work.KKT[246]*source[150]+work.KKT[248]*source[156]+work.KKT[367]*source[163]+work.KKT[369]*source[169]+work.KKT[370]*source[225]+work.KKT[371]*source[231];
  result[13] = work.KKT[253]*source[151]+work.KKT[255]*source[157]+work.KKT[378]*source[164]+work.KKT[380]*source[170]+work.KKT[381]*source[226]+work.KKT[382]*source[232];
  result[14] = work.KKT[260]*source[152]+work.KKT[262]*source[158]+work.KKT[389]*source[165]+work.KKT[391]*source[171]+work.KKT[392]*source[227]+work.KKT[393]*source[233];
  result[15] = work.KKT[267]*source[153]+work.KKT[269]*source[159]+work.KKT[400]*source[166]+work.KKT[402]*source[172]+work.KKT[403]*source[228]+work.KKT[404]*source[234];
  result[16] = work.KKT[274]*source[154]+work.KKT[276]*source[160]+work.KKT[411]*source[167]+work.KKT[413]*source[173]+work.KKT[414]*source[229]+work.KKT[415]*source[235];
  result[17] = work.KKT[281]*source[155]+work.KKT[283]*source[161]+work.KKT[422]*source[168]+work.KKT[424]*source[174]+work.KKT[425]*source[230]+work.KKT[426]*source[236];
  result[18] = work.KKT[362]*source[163]+work.KKT[363]*source[169]+work.KKT[364]*source[225]+work.KKT[365]*source[231];
  result[19] = work.KKT[373]*source[164]+work.KKT[374]*source[170]+work.KKT[375]*source[226]+work.KKT[376]*source[232];
  result[20] = work.KKT[384]*source[165]+work.KKT[385]*source[171]+work.KKT[386]*source[227]+work.KKT[387]*source[233];
  result[21] = work.KKT[395]*source[166]+work.KKT[396]*source[172]+work.KKT[397]*source[228]+work.KKT[398]*source[234];
  result[22] = work.KKT[406]*source[167]+work.KKT[407]*source[173]+work.KKT[408]*source[229]+work.KKT[409]*source[235];
  result[23] = work.KKT[417]*source[168]+work.KKT[418]*source[174]+work.KKT[419]*source[230]+work.KKT[420]*source[236];
  result[24] = work.KKT[153]*source[201]+work.KKT[155]*source[207]+work.KKT[230]*source[225]+work.KKT[231]*source[249];
  result[25] = work.KKT[157]*source[202]+work.KKT[159]*source[208]+work.KKT[232]*source[226]+work.KKT[233]*source[250];
  result[26] = work.KKT[161]*source[203]+work.KKT[163]*source[209]+work.KKT[234]*source[227]+work.KKT[235]*source[251];
  result[27] = work.KKT[165]*source[204]+work.KKT[167]*source[210]+work.KKT[236]*source[228]+work.KKT[237]*source[252];
  result[28] = work.KKT[169]*source[205]+work.KKT[171]*source[211]+work.KKT[238]*source[229]+work.KKT[239]*source[253];
  result[29] = work.KKT[173]*source[206]+work.KKT[175]*source[212]+work.KKT[240]*source[230]+work.KKT[241]*source[254];
  result[30] = work.KKT[177]*source[213]+work.KKT[179]*source[219]+work.KKT[372]*source[231]+work.KKT[428]*source[249]+work.KKT[429]*source[255];
  result[31] = work.KKT[181]*source[214]+work.KKT[183]*source[220]+work.KKT[383]*source[232]+work.KKT[430]*source[250]+work.KKT[431]*source[256];
  result[32] = work.KKT[185]*source[215]+work.KKT[187]*source[221]+work.KKT[394]*source[233]+work.KKT[432]*source[251]+work.KKT[433]*source[257];
  result[33] = work.KKT[189]*source[216]+work.KKT[191]*source[222]+work.KKT[405]*source[234]+work.KKT[434]*source[252]+work.KKT[435]*source[258];
  result[34] = work.KKT[193]*source[217]+work.KKT[195]*source[223]+work.KKT[416]*source[235]+work.KKT[436]*source[253]+work.KKT[437]*source[259];
  result[35] = work.KKT[197]*source[218]+work.KKT[199]*source[224]+work.KKT[427]*source[236]+work.KKT[438]*source[254]+work.KKT[439]*source[260];
  result[36] = work.KKT[306]*source[176]+work.KKT[308]*source[182]+work.KKT[445]*source[189]+work.KKT[447]*source[195]+work.KKT[448]*source[249]+work.KKT[449]*source[255];
  result[37] = work.KKT[313]*source[177]+work.KKT[315]*source[183]+work.KKT[455]*source[190]+work.KKT[457]*source[196]+work.KKT[458]*source[250]+work.KKT[459]*source[256];
  result[38] = work.KKT[320]*source[178]+work.KKT[322]*source[184]+work.KKT[465]*source[191]+work.KKT[467]*source[197]+work.KKT[468]*source[251]+work.KKT[469]*source[257];
  result[39] = work.KKT[327]*source[179]+work.KKT[329]*source[185]+work.KKT[475]*source[192]+work.KKT[477]*source[198]+work.KKT[478]*source[252]+work.KKT[479]*source[258];
  result[40] = work.KKT[334]*source[180]+work.KKT[336]*source[186]+work.KKT[485]*source[193]+work.KKT[487]*source[199]+work.KKT[488]*source[253]+work.KKT[489]*source[259];
  result[41] = work.KKT[341]*source[181]+work.KKT[343]*source[187]+work.KKT[495]*source[194]+work.KKT[497]*source[200]+work.KKT[498]*source[254]+work.KKT[499]*source[260];
  result[42] = work.KKT[440]*source[189]+work.KKT[441]*source[195]+work.KKT[442]*source[249]+work.KKT[443]*source[255];
  result[43] = work.KKT[450]*source[190]+work.KKT[451]*source[196]+work.KKT[452]*source[250]+work.KKT[453]*source[256];
  result[44] = work.KKT[460]*source[191]+work.KKT[461]*source[197]+work.KKT[462]*source[251]+work.KKT[463]*source[257];
  result[45] = work.KKT[470]*source[192]+work.KKT[471]*source[198]+work.KKT[472]*source[252]+work.KKT[473]*source[258];
  result[46] = work.KKT[480]*source[193]+work.KKT[481]*source[199]+work.KKT[482]*source[253]+work.KKT[483]*source[259];
  result[47] = work.KKT[490]*source[194]+work.KKT[491]*source[200]+work.KKT[492]*source[254]+work.KKT[493]*source[260];
  result[48] = work.KKT[500]*source[149]+work.KKT[501]*source[162]+work.KKT[502]*source[175]+work.KKT[503]*source[188];
  result[49] = work.KKT[244]*source[149]+work.KKT[242]*source[150]+work.KKT[243]*source[156];
  result[50] = work.KKT[251]*source[149]+work.KKT[249]*source[151]+work.KKT[250]*source[157];
  result[51] = work.KKT[258]*source[149]+work.KKT[256]*source[152]+work.KKT[257]*source[158];
  result[52] = work.KKT[265]*source[149]+work.KKT[263]*source[153]+work.KKT[264]*source[159];
  result[53] = work.KKT[272]*source[149]+work.KKT[270]*source[154]+work.KKT[271]*source[160];
  result[54] = work.KKT[279]*source[149]+work.KKT[277]*source[155]+work.KKT[278]*source[161];
  result[55] = work.KKT[286]*source[162]+work.KKT[284]*source[163]+work.KKT[285]*source[169];
  result[56] = work.KKT[289]*source[162]+work.KKT[287]*source[164]+work.KKT[288]*source[170];
  result[57] = work.KKT[292]*source[162]+work.KKT[290]*source[165]+work.KKT[291]*source[171];
  result[58] = work.KKT[295]*source[162]+work.KKT[293]*source[166]+work.KKT[294]*source[172];
  result[59] = work.KKT[298]*source[162]+work.KKT[296]*source[167]+work.KKT[297]*source[173];
  result[60] = work.KKT[301]*source[162]+work.KKT[299]*source[168]+work.KKT[300]*source[174];
  result[61] = work.KKT[304]*source[175]+work.KKT[302]*source[176]+work.KKT[303]*source[182];
  result[62] = work.KKT[311]*source[175]+work.KKT[309]*source[177]+work.KKT[310]*source[183];
  result[63] = work.KKT[318]*source[175]+work.KKT[316]*source[178]+work.KKT[317]*source[184];
  result[64] = work.KKT[325]*source[175]+work.KKT[323]*source[179]+work.KKT[324]*source[185];
  result[65] = work.KKT[332]*source[175]+work.KKT[330]*source[180]+work.KKT[331]*source[186];
  result[66] = work.KKT[339]*source[175]+work.KKT[337]*source[181]+work.KKT[338]*source[187];
  result[67] = work.KKT[346]*source[188]+work.KKT[344]*source[189]+work.KKT[345]*source[195];
  result[68] = work.KKT[349]*source[188]+work.KKT[347]*source[190]+work.KKT[348]*source[196];
  result[69] = work.KKT[352]*source[188]+work.KKT[350]*source[191]+work.KKT[351]*source[197];
  result[70] = work.KKT[355]*source[188]+work.KKT[353]*source[192]+work.KKT[354]*source[198];
  result[71] = work.KKT[358]*source[188]+work.KKT[356]*source[193]+work.KKT[357]*source[199];
  result[72] = work.KKT[361]*source[188]+work.KKT[359]*source[194]+work.KKT[360]*source[200];
  result[73] = work.KKT[0]*source[73]+work.KKT[1]*source[149];
  result[74] = work.KKT[2]*source[74]+work.KKT[3]*source[150];
  result[75] = work.KKT[4]*source[75]+work.KKT[5]*source[151];
  result[76] = work.KKT[6]*source[76]+work.KKT[7]*source[152];
  result[77] = work.KKT[8]*source[77]+work.KKT[9]*source[153];
  result[78] = work.KKT[10]*source[78]+work.KKT[11]*source[154];
  result[79] = work.KKT[12]*source[79]+work.KKT[13]*source[155];
  result[80] = work.KKT[14]*source[80]+work.KKT[15]*source[156];
  result[81] = work.KKT[16]*source[81]+work.KKT[17]*source[157];
  result[82] = work.KKT[18]*source[82]+work.KKT[19]*source[158];
  result[83] = work.KKT[20]*source[83]+work.KKT[21]*source[159];
  result[84] = work.KKT[22]*source[84]+work.KKT[23]*source[160];
  result[85] = work.KKT[24]*source[85]+work.KKT[25]*source[161];
  result[86] = work.KKT[26]*source[86]+work.KKT[27]*source[162];
  result[87] = work.KKT[28]*source[87]+work.KKT[29]*source[163];
  result[88] = work.KKT[30]*source[88]+work.KKT[31]*source[164];
  result[89] = work.KKT[32]*source[89]+work.KKT[33]*source[165];
  result[90] = work.KKT[34]*source[90]+work.KKT[35]*source[166];
  result[91] = work.KKT[36]*source[91]+work.KKT[37]*source[167];
  result[92] = work.KKT[38]*source[92]+work.KKT[39]*source[168];
  result[93] = work.KKT[40]*source[93]+work.KKT[41]*source[169];
  result[94] = work.KKT[42]*source[94]+work.KKT[43]*source[170];
  result[95] = work.KKT[44]*source[95]+work.KKT[45]*source[171];
  result[96] = work.KKT[46]*source[96]+work.KKT[47]*source[172];
  result[97] = work.KKT[48]*source[97]+work.KKT[49]*source[173];
  result[98] = work.KKT[50]*source[98]+work.KKT[51]*source[174];
  result[99] = work.KKT[52]*source[99]+work.KKT[53]*source[175];
  result[100] = work.KKT[54]*source[100]+work.KKT[55]*source[176];
  result[101] = work.KKT[56]*source[101]+work.KKT[57]*source[177];
  result[102] = work.KKT[58]*source[102]+work.KKT[59]*source[178];
  result[103] = work.KKT[60]*source[103]+work.KKT[61]*source[179];
  result[104] = work.KKT[62]*source[104]+work.KKT[63]*source[180];
  result[105] = work.KKT[64]*source[105]+work.KKT[65]*source[181];
  result[106] = work.KKT[66]*source[106]+work.KKT[67]*source[182];
  result[107] = work.KKT[68]*source[107]+work.KKT[69]*source[183];
  result[108] = work.KKT[70]*source[108]+work.KKT[71]*source[184];
  result[109] = work.KKT[72]*source[109]+work.KKT[73]*source[185];
  result[110] = work.KKT[74]*source[110]+work.KKT[75]*source[186];
  result[111] = work.KKT[76]*source[111]+work.KKT[77]*source[187];
  result[112] = work.KKT[78]*source[112]+work.KKT[79]*source[188];
  result[113] = work.KKT[80]*source[113]+work.KKT[81]*source[189];
  result[114] = work.KKT[82]*source[114]+work.KKT[83]*source[190];
  result[115] = work.KKT[84]*source[115]+work.KKT[85]*source[191];
  result[116] = work.KKT[86]*source[116]+work.KKT[87]*source[192];
  result[117] = work.KKT[88]*source[117]+work.KKT[89]*source[193];
  result[118] = work.KKT[90]*source[118]+work.KKT[91]*source[194];
  result[119] = work.KKT[92]*source[119]+work.KKT[93]*source[195];
  result[120] = work.KKT[94]*source[120]+work.KKT[95]*source[196];
  result[121] = work.KKT[96]*source[121]+work.KKT[97]*source[197];
  result[122] = work.KKT[98]*source[122]+work.KKT[99]*source[198];
  result[123] = work.KKT[100]*source[123]+work.KKT[101]*source[199];
  result[124] = work.KKT[102]*source[124]+work.KKT[103]*source[200];
  result[125] = work.KKT[104]*source[125]+work.KKT[105]*source[201];
  result[126] = work.KKT[106]*source[126]+work.KKT[107]*source[202];
  result[127] = work.KKT[108]*source[127]+work.KKT[109]*source[203];
  result[128] = work.KKT[110]*source[128]+work.KKT[111]*source[204];
  result[129] = work.KKT[112]*source[129]+work.KKT[113]*source[205];
  result[130] = work.KKT[114]*source[130]+work.KKT[115]*source[206];
  result[131] = work.KKT[116]*source[131]+work.KKT[117]*source[207];
  result[132] = work.KKT[118]*source[132]+work.KKT[119]*source[208];
  result[133] = work.KKT[120]*source[133]+work.KKT[121]*source[209];
  result[134] = work.KKT[122]*source[134]+work.KKT[123]*source[210];
  result[135] = work.KKT[124]*source[135]+work.KKT[125]*source[211];
  result[136] = work.KKT[126]*source[136]+work.KKT[127]*source[212];
  result[137] = work.KKT[128]*source[137]+work.KKT[129]*source[213];
  result[138] = work.KKT[130]*source[138]+work.KKT[131]*source[214];
  result[139] = work.KKT[132]*source[139]+work.KKT[133]*source[215];
  result[140] = work.KKT[134]*source[140]+work.KKT[135]*source[216];
  result[141] = work.KKT[136]*source[141]+work.KKT[137]*source[217];
  result[142] = work.KKT[138]*source[142]+work.KKT[139]*source[218];
  result[143] = work.KKT[140]*source[143]+work.KKT[141]*source[219];
  result[144] = work.KKT[142]*source[144]+work.KKT[143]*source[220];
  result[145] = work.KKT[144]*source[145]+work.KKT[145]*source[221];
  result[146] = work.KKT[146]*source[146]+work.KKT[147]*source[222];
  result[147] = work.KKT[148]*source[147]+work.KKT[149]*source[223];
  result[148] = work.KKT[150]*source[148]+work.KKT[151]*source[224];
  result[149] = work.KKT[1]*source[73]+work.KKT[504]*source[149]+work.KKT[500]*source[48]+work.KKT[244]*source[49]+work.KKT[251]*source[50]+work.KKT[258]*source[51]+work.KKT[265]*source[52]+work.KKT[272]*source[53]+work.KKT[279]*source[54];
  result[150] = work.KKT[3]*source[74]+work.KKT[245]*source[150]+work.KKT[246]*source[12]+work.KKT[242]*source[49];
  result[151] = work.KKT[5]*source[75]+work.KKT[252]*source[151]+work.KKT[253]*source[13]+work.KKT[249]*source[50];
  result[152] = work.KKT[7]*source[76]+work.KKT[259]*source[152]+work.KKT[260]*source[14]+work.KKT[256]*source[51];
  result[153] = work.KKT[9]*source[77]+work.KKT[266]*source[153]+work.KKT[267]*source[15]+work.KKT[263]*source[52];
  result[154] = work.KKT[11]*source[78]+work.KKT[273]*source[154]+work.KKT[274]*source[16]+work.KKT[270]*source[53];
  result[155] = work.KKT[13]*source[79]+work.KKT[280]*source[155]+work.KKT[281]*source[17]+work.KKT[277]*source[54];
  result[156] = work.KKT[15]*source[80]+work.KKT[247]*source[156]+work.KKT[248]*source[12]+work.KKT[243]*source[49];
  result[157] = work.KKT[17]*source[81]+work.KKT[254]*source[157]+work.KKT[255]*source[13]+work.KKT[250]*source[50];
  result[158] = work.KKT[19]*source[82]+work.KKT[261]*source[158]+work.KKT[262]*source[14]+work.KKT[257]*source[51];
  result[159] = work.KKT[21]*source[83]+work.KKT[268]*source[159]+work.KKT[269]*source[15]+work.KKT[264]*source[52];
  result[160] = work.KKT[23]*source[84]+work.KKT[275]*source[160]+work.KKT[276]*source[16]+work.KKT[271]*source[53];
  result[161] = work.KKT[25]*source[85]+work.KKT[282]*source[161]+work.KKT[283]*source[17]+work.KKT[278]*source[54];
  result[162] = work.KKT[27]*source[86]+work.KKT[505]*source[162]+work.KKT[501]*source[48]+work.KKT[286]*source[55]+work.KKT[289]*source[56]+work.KKT[292]*source[57]+work.KKT[295]*source[58]+work.KKT[298]*source[59]+work.KKT[301]*source[60];
  result[163] = work.KKT[29]*source[87]+work.KKT[366]*source[163]+work.KKT[367]*source[12]+work.KKT[362]*source[18]+work.KKT[284]*source[55];
  result[164] = work.KKT[31]*source[88]+work.KKT[377]*source[164]+work.KKT[378]*source[13]+work.KKT[373]*source[19]+work.KKT[287]*source[56];
  result[165] = work.KKT[33]*source[89]+work.KKT[388]*source[165]+work.KKT[389]*source[14]+work.KKT[384]*source[20]+work.KKT[290]*source[57];
  result[166] = work.KKT[35]*source[90]+work.KKT[399]*source[166]+work.KKT[400]*source[15]+work.KKT[395]*source[21]+work.KKT[293]*source[58];
  result[167] = work.KKT[37]*source[91]+work.KKT[410]*source[167]+work.KKT[411]*source[16]+work.KKT[406]*source[22]+work.KKT[296]*source[59];
  result[168] = work.KKT[39]*source[92]+work.KKT[421]*source[168]+work.KKT[422]*source[17]+work.KKT[417]*source[23]+work.KKT[299]*source[60];
  result[169] = work.KKT[41]*source[93]+work.KKT[368]*source[169]+work.KKT[369]*source[12]+work.KKT[363]*source[18]+work.KKT[285]*source[55];
  result[170] = work.KKT[43]*source[94]+work.KKT[379]*source[170]+work.KKT[380]*source[13]+work.KKT[374]*source[19]+work.KKT[288]*source[56];
  result[171] = work.KKT[45]*source[95]+work.KKT[390]*source[171]+work.KKT[391]*source[14]+work.KKT[385]*source[20]+work.KKT[291]*source[57];
  result[172] = work.KKT[47]*source[96]+work.KKT[401]*source[172]+work.KKT[402]*source[15]+work.KKT[396]*source[21]+work.KKT[294]*source[58];
  result[173] = work.KKT[49]*source[97]+work.KKT[412]*source[173]+work.KKT[413]*source[16]+work.KKT[407]*source[22]+work.KKT[297]*source[59];
  result[174] = work.KKT[51]*source[98]+work.KKT[423]*source[174]+work.KKT[424]*source[17]+work.KKT[418]*source[23]+work.KKT[300]*source[60];
  result[175] = work.KKT[53]*source[99]+work.KKT[506]*source[175]+work.KKT[502]*source[48]+work.KKT[304]*source[61]+work.KKT[311]*source[62]+work.KKT[318]*source[63]+work.KKT[325]*source[64]+work.KKT[332]*source[65]+work.KKT[339]*source[66];
  result[176] = work.KKT[55]*source[100]+work.KKT[305]*source[176]+work.KKT[306]*source[36]+work.KKT[302]*source[61];
  result[177] = work.KKT[57]*source[101]+work.KKT[312]*source[177]+work.KKT[313]*source[37]+work.KKT[309]*source[62];
  result[178] = work.KKT[59]*source[102]+work.KKT[319]*source[178]+work.KKT[320]*source[38]+work.KKT[316]*source[63];
  result[179] = work.KKT[61]*source[103]+work.KKT[326]*source[179]+work.KKT[327]*source[39]+work.KKT[323]*source[64];
  result[180] = work.KKT[63]*source[104]+work.KKT[333]*source[180]+work.KKT[334]*source[40]+work.KKT[330]*source[65];
  result[181] = work.KKT[65]*source[105]+work.KKT[340]*source[181]+work.KKT[341]*source[41]+work.KKT[337]*source[66];
  result[182] = work.KKT[67]*source[106]+work.KKT[307]*source[182]+work.KKT[308]*source[36]+work.KKT[303]*source[61];
  result[183] = work.KKT[69]*source[107]+work.KKT[314]*source[183]+work.KKT[315]*source[37]+work.KKT[310]*source[62];
  result[184] = work.KKT[71]*source[108]+work.KKT[321]*source[184]+work.KKT[322]*source[38]+work.KKT[317]*source[63];
  result[185] = work.KKT[73]*source[109]+work.KKT[328]*source[185]+work.KKT[329]*source[39]+work.KKT[324]*source[64];
  result[186] = work.KKT[75]*source[110]+work.KKT[335]*source[186]+work.KKT[336]*source[40]+work.KKT[331]*source[65];
  result[187] = work.KKT[77]*source[111]+work.KKT[342]*source[187]+work.KKT[343]*source[41]+work.KKT[338]*source[66];
  result[188] = work.KKT[79]*source[112]+work.KKT[507]*source[188]+work.KKT[503]*source[48]+work.KKT[346]*source[67]+work.KKT[349]*source[68]+work.KKT[352]*source[69]+work.KKT[355]*source[70]+work.KKT[358]*source[71]+work.KKT[361]*source[72];
  result[189] = work.KKT[81]*source[113]+work.KKT[444]*source[189]+work.KKT[445]*source[36]+work.KKT[440]*source[42]+work.KKT[344]*source[67];
  result[190] = work.KKT[83]*source[114]+work.KKT[454]*source[190]+work.KKT[455]*source[37]+work.KKT[450]*source[43]+work.KKT[347]*source[68];
  result[191] = work.KKT[85]*source[115]+work.KKT[464]*source[191]+work.KKT[465]*source[38]+work.KKT[460]*source[44]+work.KKT[350]*source[69];
  result[192] = work.KKT[87]*source[116]+work.KKT[474]*source[192]+work.KKT[475]*source[39]+work.KKT[470]*source[45]+work.KKT[353]*source[70];
  result[193] = work.KKT[89]*source[117]+work.KKT[484]*source[193]+work.KKT[485]*source[40]+work.KKT[480]*source[46]+work.KKT[356]*source[71];
  result[194] = work.KKT[91]*source[118]+work.KKT[494]*source[194]+work.KKT[495]*source[41]+work.KKT[490]*source[47]+work.KKT[359]*source[72];
  result[195] = work.KKT[93]*source[119]+work.KKT[446]*source[195]+work.KKT[447]*source[36]+work.KKT[441]*source[42]+work.KKT[345]*source[67];
  result[196] = work.KKT[95]*source[120]+work.KKT[456]*source[196]+work.KKT[457]*source[37]+work.KKT[451]*source[43]+work.KKT[348]*source[68];
  result[197] = work.KKT[97]*source[121]+work.KKT[466]*source[197]+work.KKT[467]*source[38]+work.KKT[461]*source[44]+work.KKT[351]*source[69];
  result[198] = work.KKT[99]*source[122]+work.KKT[476]*source[198]+work.KKT[477]*source[39]+work.KKT[471]*source[45]+work.KKT[354]*source[70];
  result[199] = work.KKT[101]*source[123]+work.KKT[486]*source[199]+work.KKT[487]*source[40]+work.KKT[481]*source[46]+work.KKT[357]*source[71];
  result[200] = work.KKT[103]*source[124]+work.KKT[496]*source[200]+work.KKT[497]*source[41]+work.KKT[491]*source[47]+work.KKT[360]*source[72];
  result[201] = work.KKT[105]*source[125]+work.KKT[152]*source[201]+work.KKT[153]*source[24];
  result[202] = work.KKT[107]*source[126]+work.KKT[156]*source[202]+work.KKT[157]*source[25];
  result[203] = work.KKT[109]*source[127]+work.KKT[160]*source[203]+work.KKT[161]*source[26];
  result[204] = work.KKT[111]*source[128]+work.KKT[164]*source[204]+work.KKT[165]*source[27];
  result[205] = work.KKT[113]*source[129]+work.KKT[168]*source[205]+work.KKT[169]*source[28];
  result[206] = work.KKT[115]*source[130]+work.KKT[172]*source[206]+work.KKT[173]*source[29];
  result[207] = work.KKT[117]*source[131]+work.KKT[154]*source[207]+work.KKT[155]*source[24];
  result[208] = work.KKT[119]*source[132]+work.KKT[158]*source[208]+work.KKT[159]*source[25];
  result[209] = work.KKT[121]*source[133]+work.KKT[162]*source[209]+work.KKT[163]*source[26];
  result[210] = work.KKT[123]*source[134]+work.KKT[166]*source[210]+work.KKT[167]*source[27];
  result[211] = work.KKT[125]*source[135]+work.KKT[170]*source[211]+work.KKT[171]*source[28];
  result[212] = work.KKT[127]*source[136]+work.KKT[174]*source[212]+work.KKT[175]*source[29];
  result[213] = work.KKT[129]*source[137]+work.KKT[176]*source[213]+work.KKT[177]*source[30];
  result[214] = work.KKT[131]*source[138]+work.KKT[180]*source[214]+work.KKT[181]*source[31];
  result[215] = work.KKT[133]*source[139]+work.KKT[184]*source[215]+work.KKT[185]*source[32];
  result[216] = work.KKT[135]*source[140]+work.KKT[188]*source[216]+work.KKT[189]*source[33];
  result[217] = work.KKT[137]*source[141]+work.KKT[192]*source[217]+work.KKT[193]*source[34];
  result[218] = work.KKT[139]*source[142]+work.KKT[196]*source[218]+work.KKT[197]*source[35];
  result[219] = work.KKT[141]*source[143]+work.KKT[178]*source[219]+work.KKT[179]*source[30];
  result[220] = work.KKT[143]*source[144]+work.KKT[182]*source[220]+work.KKT[183]*source[31];
  result[221] = work.KKT[145]*source[145]+work.KKT[186]*source[221]+work.KKT[187]*source[32];
  result[222] = work.KKT[147]*source[146]+work.KKT[190]*source[222]+work.KKT[191]*source[33];
  result[223] = work.KKT[149]*source[147]+work.KKT[194]*source[223]+work.KKT[195]*source[34];
  result[224] = work.KKT[151]*source[148]+work.KKT[198]*source[224]+work.KKT[199]*source[35];
  result[225] = work.KKT[201]*source[0]+work.KKT[218]*source[6]+work.KKT[370]*source[12]+work.KKT[364]*source[18]+work.KKT[230]*source[24];
  result[226] = work.KKT[203]*source[1]+work.KKT[220]*source[7]+work.KKT[381]*source[13]+work.KKT[375]*source[19]+work.KKT[232]*source[25];
  result[227] = work.KKT[205]*source[2]+work.KKT[222]*source[8]+work.KKT[392]*source[14]+work.KKT[386]*source[20]+work.KKT[234]*source[26];
  result[228] = work.KKT[207]*source[3]+work.KKT[224]*source[9]+work.KKT[403]*source[15]+work.KKT[397]*source[21]+work.KKT[236]*source[27];
  result[229] = work.KKT[209]*source[4]+work.KKT[226]*source[10]+work.KKT[414]*source[16]+work.KKT[408]*source[22]+work.KKT[238]*source[28];
  result[230] = work.KKT[211]*source[5]+work.KKT[228]*source[11]+work.KKT[425]*source[17]+work.KKT[419]*source[23]+work.KKT[240]*source[29];
  result[231] = work.KKT[219]*source[6]+work.KKT[371]*source[12]+work.KKT[365]*source[18]+work.KKT[372]*source[30];
  result[232] = work.KKT[221]*source[7]+work.KKT[382]*source[13]+work.KKT[376]*source[19]+work.KKT[383]*source[31];
  result[233] = work.KKT[223]*source[8]+work.KKT[393]*source[14]+work.KKT[387]*source[20]+work.KKT[394]*source[32];
  result[234] = work.KKT[225]*source[9]+work.KKT[404]*source[15]+work.KKT[398]*source[21]+work.KKT[405]*source[33];
  result[235] = work.KKT[227]*source[10]+work.KKT[415]*source[16]+work.KKT[409]*source[22]+work.KKT[416]*source[34];
  result[236] = work.KKT[229]*source[11]+work.KKT[426]*source[17]+work.KKT[420]*source[23]+work.KKT[427]*source[35];
  result[237] = work.KKT[200]*source[0];
  result[238] = work.KKT[202]*source[1];
  result[239] = work.KKT[204]*source[2];
  result[240] = work.KKT[206]*source[3];
  result[241] = work.KKT[208]*source[4];
  result[242] = work.KKT[210]*source[5];
  result[243] = work.KKT[212]*source[6];
  result[244] = work.KKT[213]*source[7];
  result[245] = work.KKT[214]*source[8];
  result[246] = work.KKT[215]*source[9];
  result[247] = work.KKT[216]*source[10];
  result[248] = work.KKT[217]*source[11];
  result[249] = work.KKT[231]*source[24]+work.KKT[428]*source[30]+work.KKT[448]*source[36]+work.KKT[442]*source[42];
  result[250] = work.KKT[233]*source[25]+work.KKT[430]*source[31]+work.KKT[458]*source[37]+work.KKT[452]*source[43];
  result[251] = work.KKT[235]*source[26]+work.KKT[432]*source[32]+work.KKT[468]*source[38]+work.KKT[462]*source[44];
  result[252] = work.KKT[237]*source[27]+work.KKT[434]*source[33]+work.KKT[478]*source[39]+work.KKT[472]*source[45];
  result[253] = work.KKT[239]*source[28]+work.KKT[436]*source[34]+work.KKT[488]*source[40]+work.KKT[482]*source[46];
  result[254] = work.KKT[241]*source[29]+work.KKT[438]*source[35]+work.KKT[498]*source[41]+work.KKT[492]*source[47];
  result[255] = work.KKT[429]*source[30]+work.KKT[449]*source[36]+work.KKT[443]*source[42];
  result[256] = work.KKT[431]*source[31]+work.KKT[459]*source[37]+work.KKT[453]*source[43];
  result[257] = work.KKT[433]*source[32]+work.KKT[469]*source[38]+work.KKT[463]*source[44];
  result[258] = work.KKT[435]*source[33]+work.KKT[479]*source[39]+work.KKT[473]*source[45];
  result[259] = work.KKT[437]*source[34]+work.KKT[489]*source[40]+work.KKT[483]*source[46];
  result[260] = work.KKT[439]*source[35]+work.KKT[499]*source[41]+work.KKT[493]*source[47];
}
double check_residual(double *target, double *multiplicand) {
  /* Returns the squared 2-norm of lhs - A*rhs. */
  /* Reuses v to find the residual. */
  int i;
  double residual;
  residual = 0;
  matrix_multiply(work.v, multiplicand);
  for (i = 0; i < 73; i++) {
    residual += (target[i] - work.v[i])*(target[i] - work.v[i]);
  }
  return residual;
}
void fill_KKT(void) {
  work.KKT[0] = work.s_inv_z[0];
  work.KKT[2] = work.s_inv_z[1];
  work.KKT[4] = work.s_inv_z[2];
  work.KKT[6] = work.s_inv_z[3];
  work.KKT[8] = work.s_inv_z[4];
  work.KKT[10] = work.s_inv_z[5];
  work.KKT[12] = work.s_inv_z[6];
  work.KKT[14] = work.s_inv_z[7];
  work.KKT[16] = work.s_inv_z[8];
  work.KKT[18] = work.s_inv_z[9];
  work.KKT[20] = work.s_inv_z[10];
  work.KKT[22] = work.s_inv_z[11];
  work.KKT[24] = work.s_inv_z[12];
  work.KKT[26] = work.s_inv_z[13];
  work.KKT[28] = work.s_inv_z[14];
  work.KKT[30] = work.s_inv_z[15];
  work.KKT[32] = work.s_inv_z[16];
  work.KKT[34] = work.s_inv_z[17];
  work.KKT[36] = work.s_inv_z[18];
  work.KKT[38] = work.s_inv_z[19];
  work.KKT[40] = work.s_inv_z[20];
  work.KKT[42] = work.s_inv_z[21];
  work.KKT[44] = work.s_inv_z[22];
  work.KKT[46] = work.s_inv_z[23];
  work.KKT[48] = work.s_inv_z[24];
  work.KKT[50] = work.s_inv_z[25];
  work.KKT[52] = work.s_inv_z[26];
  work.KKT[54] = work.s_inv_z[27];
  work.KKT[56] = work.s_inv_z[28];
  work.KKT[58] = work.s_inv_z[29];
  work.KKT[60] = work.s_inv_z[30];
  work.KKT[62] = work.s_inv_z[31];
  work.KKT[64] = work.s_inv_z[32];
  work.KKT[66] = work.s_inv_z[33];
  work.KKT[68] = work.s_inv_z[34];
  work.KKT[70] = work.s_inv_z[35];
  work.KKT[72] = work.s_inv_z[36];
  work.KKT[74] = work.s_inv_z[37];
  work.KKT[76] = work.s_inv_z[38];
  work.KKT[78] = work.s_inv_z[39];
  work.KKT[80] = work.s_inv_z[40];
  work.KKT[82] = work.s_inv_z[41];
  work.KKT[84] = work.s_inv_z[42];
  work.KKT[86] = work.s_inv_z[43];
  work.KKT[88] = work.s_inv_z[44];
  work.KKT[90] = work.s_inv_z[45];
  work.KKT[92] = work.s_inv_z[46];
  work.KKT[94] = work.s_inv_z[47];
  work.KKT[96] = work.s_inv_z[48];
  work.KKT[98] = work.s_inv_z[49];
  work.KKT[100] = work.s_inv_z[50];
  work.KKT[102] = work.s_inv_z[51];
  work.KKT[104] = work.s_inv_z[52];
  work.KKT[106] = work.s_inv_z[53];
  work.KKT[108] = work.s_inv_z[54];
  work.KKT[110] = work.s_inv_z[55];
  work.KKT[112] = work.s_inv_z[56];
  work.KKT[114] = work.s_inv_z[57];
  work.KKT[116] = work.s_inv_z[58];
  work.KKT[118] = work.s_inv_z[59];
  work.KKT[120] = work.s_inv_z[60];
  work.KKT[122] = work.s_inv_z[61];
  work.KKT[124] = work.s_inv_z[62];
  work.KKT[126] = work.s_inv_z[63];
  work.KKT[128] = work.s_inv_z[64];
  work.KKT[130] = work.s_inv_z[65];
  work.KKT[132] = work.s_inv_z[66];
  work.KKT[134] = work.s_inv_z[67];
  work.KKT[136] = work.s_inv_z[68];
  work.KKT[138] = work.s_inv_z[69];
  work.KKT[140] = work.s_inv_z[70];
  work.KKT[142] = work.s_inv_z[71];
  work.KKT[144] = work.s_inv_z[72];
  work.KKT[146] = work.s_inv_z[73];
  work.KKT[148] = work.s_inv_z[74];
  work.KKT[150] = work.s_inv_z[75];
  work.KKT[1] = 1;
  work.KKT[3] = 1;
  work.KKT[5] = 1;
  work.KKT[7] = 1;
  work.KKT[9] = 1;
  work.KKT[11] = 1;
  work.KKT[13] = 1;
  work.KKT[15] = 1;
  work.KKT[17] = 1;
  work.KKT[19] = 1;
  work.KKT[21] = 1;
  work.KKT[23] = 1;
  work.KKT[25] = 1;
  work.KKT[27] = 1;
  work.KKT[29] = 1;
  work.KKT[31] = 1;
  work.KKT[33] = 1;
  work.KKT[35] = 1;
  work.KKT[37] = 1;
  work.KKT[39] = 1;
  work.KKT[41] = 1;
  work.KKT[43] = 1;
  work.KKT[45] = 1;
  work.KKT[47] = 1;
  work.KKT[49] = 1;
  work.KKT[51] = 1;
  work.KKT[53] = 1;
  work.KKT[55] = 1;
  work.KKT[57] = 1;
  work.KKT[59] = 1;
  work.KKT[61] = 1;
  work.KKT[63] = 1;
  work.KKT[65] = 1;
  work.KKT[67] = 1;
  work.KKT[69] = 1;
  work.KKT[71] = 1;
  work.KKT[73] = 1;
  work.KKT[75] = 1;
  work.KKT[77] = 1;
  work.KKT[79] = 1;
  work.KKT[81] = 1;
  work.KKT[83] = 1;
  work.KKT[85] = 1;
  work.KKT[87] = 1;
  work.KKT[89] = 1;
  work.KKT[91] = 1;
  work.KKT[93] = 1;
  work.KKT[95] = 1;
  work.KKT[97] = 1;
  work.KKT[99] = 1;
  work.KKT[101] = 1;
  work.KKT[103] = 1;
  work.KKT[105] = 1;
  work.KKT[107] = 1;
  work.KKT[109] = 1;
  work.KKT[111] = 1;
  work.KKT[113] = 1;
  work.KKT[115] = 1;
  work.KKT[117] = 1;
  work.KKT[119] = 1;
  work.KKT[121] = 1;
  work.KKT[123] = 1;
  work.KKT[125] = 1;
  work.KKT[127] = 1;
  work.KKT[129] = 1;
  work.KKT[131] = 1;
  work.KKT[133] = 1;
  work.KKT[135] = 1;
  work.KKT[137] = 1;
  work.KKT[139] = 1;
  work.KKT[141] = 1;
  work.KKT[143] = 1;
  work.KKT[145] = 1;
  work.KKT[147] = 1;
  work.KKT[149] = 1;
  work.KKT[151] = 1;
  work.KKT[504] = work.block_33[0];
  work.KKT[245] = work.block_33[0];
  work.KKT[252] = work.block_33[0];
  work.KKT[259] = work.block_33[0];
  work.KKT[266] = work.block_33[0];
  work.KKT[273] = work.block_33[0];
  work.KKT[280] = work.block_33[0];
  work.KKT[247] = work.block_33[0];
  work.KKT[254] = work.block_33[0];
  work.KKT[261] = work.block_33[0];
  work.KKT[268] = work.block_33[0];
  work.KKT[275] = work.block_33[0];
  work.KKT[282] = work.block_33[0];
  work.KKT[505] = work.block_33[0];
  work.KKT[366] = work.block_33[0];
  work.KKT[377] = work.block_33[0];
  work.KKT[388] = work.block_33[0];
  work.KKT[399] = work.block_33[0];
  work.KKT[410] = work.block_33[0];
  work.KKT[421] = work.block_33[0];
  work.KKT[368] = work.block_33[0];
  work.KKT[379] = work.block_33[0];
  work.KKT[390] = work.block_33[0];
  work.KKT[401] = work.block_33[0];
  work.KKT[412] = work.block_33[0];
  work.KKT[423] = work.block_33[0];
  work.KKT[506] = work.block_33[0];
  work.KKT[305] = work.block_33[0];
  work.KKT[312] = work.block_33[0];
  work.KKT[319] = work.block_33[0];
  work.KKT[326] = work.block_33[0];
  work.KKT[333] = work.block_33[0];
  work.KKT[340] = work.block_33[0];
  work.KKT[307] = work.block_33[0];
  work.KKT[314] = work.block_33[0];
  work.KKT[321] = work.block_33[0];
  work.KKT[328] = work.block_33[0];
  work.KKT[335] = work.block_33[0];
  work.KKT[342] = work.block_33[0];
  work.KKT[507] = work.block_33[0];
  work.KKT[444] = work.block_33[0];
  work.KKT[454] = work.block_33[0];
  work.KKT[464] = work.block_33[0];
  work.KKT[474] = work.block_33[0];
  work.KKT[484] = work.block_33[0];
  work.KKT[494] = work.block_33[0];
  work.KKT[446] = work.block_33[0];
  work.KKT[456] = work.block_33[0];
  work.KKT[466] = work.block_33[0];
  work.KKT[476] = work.block_33[0];
  work.KKT[486] = work.block_33[0];
  work.KKT[496] = work.block_33[0];
  work.KKT[152] = work.block_33[0];
  work.KKT[156] = work.block_33[0];
  work.KKT[160] = work.block_33[0];
  work.KKT[164] = work.block_33[0];
  work.KKT[168] = work.block_33[0];
  work.KKT[172] = work.block_33[0];
  work.KKT[154] = work.block_33[0];
  work.KKT[158] = work.block_33[0];
  work.KKT[162] = work.block_33[0];
  work.KKT[166] = work.block_33[0];
  work.KKT[170] = work.block_33[0];
  work.KKT[174] = work.block_33[0];
  work.KKT[176] = work.block_33[0];
  work.KKT[180] = work.block_33[0];
  work.KKT[184] = work.block_33[0];
  work.KKT[188] = work.block_33[0];
  work.KKT[192] = work.block_33[0];
  work.KKT[196] = work.block_33[0];
  work.KKT[178] = work.block_33[0];
  work.KKT[182] = work.block_33[0];
  work.KKT[186] = work.block_33[0];
  work.KKT[190] = work.block_33[0];
  work.KKT[194] = work.block_33[0];
  work.KKT[198] = work.block_33[0];
  work.KKT[500] = -1;
  work.KKT[244] = 1;
  work.KKT[251] = 1;
  work.KKT[258] = 1;
  work.KKT[265] = 1;
  work.KKT[272] = 1;
  work.KKT[279] = 1;
  work.KKT[246] = 2;
  work.KKT[253] = 2;
  work.KKT[260] = 2;
  work.KKT[267] = 2;
  work.KKT[274] = 2;
  work.KKT[281] = 2;
  work.KKT[242] = -1;
  work.KKT[249] = -1;
  work.KKT[256] = -1;
  work.KKT[263] = -1;
  work.KKT[270] = -1;
  work.KKT[277] = -1;
  work.KKT[248] = -2;
  work.KKT[255] = -2;
  work.KKT[262] = -2;
  work.KKT[269] = -2;
  work.KKT[276] = -2;
  work.KKT[283] = -2;
  work.KKT[243] = -1;
  work.KKT[250] = -1;
  work.KKT[257] = -1;
  work.KKT[264] = -1;
  work.KKT[271] = -1;
  work.KKT[278] = -1;
  work.KKT[501] = -1;
  work.KKT[286] = 1;
  work.KKT[289] = 1;
  work.KKT[292] = 1;
  work.KKT[295] = 1;
  work.KKT[298] = 1;
  work.KKT[301] = 1;
  work.KKT[367] = 2;
  work.KKT[378] = 2;
  work.KKT[389] = 2;
  work.KKT[400] = 2;
  work.KKT[411] = 2;
  work.KKT[422] = 2;
  work.KKT[362] = 6*params.t_switch[0];
  work.KKT[373] = 6*params.t_switch[0];
  work.KKT[384] = 6*params.t_switch[0];
  work.KKT[395] = 6*params.t_switch[0];
  work.KKT[406] = 6*params.t_switch[0];
  work.KKT[417] = 6*params.t_switch[0];
  work.KKT[284] = -1;
  work.KKT[287] = -1;
  work.KKT[290] = -1;
  work.KKT[293] = -1;
  work.KKT[296] = -1;
  work.KKT[299] = -1;
  work.KKT[369] = -2;
  work.KKT[380] = -2;
  work.KKT[391] = -2;
  work.KKT[402] = -2;
  work.KKT[413] = -2;
  work.KKT[424] = -2;
  work.KKT[363] = -6*params.t_switch[0];
  work.KKT[374] = -6*params.t_switch[0];
  work.KKT[385] = -6*params.t_switch[0];
  work.KKT[396] = -6*params.t_switch[0];
  work.KKT[407] = -6*params.t_switch[0];
  work.KKT[418] = -6*params.t_switch[0];
  work.KKT[285] = -1;
  work.KKT[288] = -1;
  work.KKT[291] = -1;
  work.KKT[294] = -1;
  work.KKT[297] = -1;
  work.KKT[300] = -1;
  work.KKT[502] = -1;
  work.KKT[304] = 1;
  work.KKT[311] = 1;
  work.KKT[318] = 1;
  work.KKT[325] = 1;
  work.KKT[332] = 1;
  work.KKT[339] = 1;
  work.KKT[306] = 2;
  work.KKT[313] = 2;
  work.KKT[320] = 2;
  work.KKT[327] = 2;
  work.KKT[334] = 2;
  work.KKT[341] = 2;
  work.KKT[302] = -1;
  work.KKT[309] = -1;
  work.KKT[316] = -1;
  work.KKT[323] = -1;
  work.KKT[330] = -1;
  work.KKT[337] = -1;
  work.KKT[308] = -2;
  work.KKT[315] = -2;
  work.KKT[322] = -2;
  work.KKT[329] = -2;
  work.KKT[336] = -2;
  work.KKT[343] = -2;
  work.KKT[303] = -1;
  work.KKT[310] = -1;
  work.KKT[317] = -1;
  work.KKT[324] = -1;
  work.KKT[331] = -1;
  work.KKT[338] = -1;
  work.KKT[503] = -1;
  work.KKT[346] = 1;
  work.KKT[349] = 1;
  work.KKT[352] = 1;
  work.KKT[355] = 1;
  work.KKT[358] = 1;
  work.KKT[361] = 1;
  work.KKT[445] = 2;
  work.KKT[455] = 2;
  work.KKT[465] = 2;
  work.KKT[475] = 2;
  work.KKT[485] = 2;
  work.KKT[495] = 2;
  work.KKT[440] = 6*(params.t_f[0]-params.t_switch[0]);
  work.KKT[450] = 6*(params.t_f[0]-params.t_switch[0]);
  work.KKT[460] = 6*(params.t_f[0]-params.t_switch[0]);
  work.KKT[470] = 6*(params.t_f[0]-params.t_switch[0]);
  work.KKT[480] = 6*(params.t_f[0]-params.t_switch[0]);
  work.KKT[490] = 6*(params.t_f[0]-params.t_switch[0]);
  work.KKT[344] = -1;
  work.KKT[347] = -1;
  work.KKT[350] = -1;
  work.KKT[353] = -1;
  work.KKT[356] = -1;
  work.KKT[359] = -1;
  work.KKT[447] = -2;
  work.KKT[457] = -2;
  work.KKT[467] = -2;
  work.KKT[477] = -2;
  work.KKT[487] = -2;
  work.KKT[497] = -2;
  work.KKT[441] = -6*(params.t_f[0]-params.t_switch[0]);
  work.KKT[451] = -6*(params.t_f[0]-params.t_switch[0]);
  work.KKT[461] = -6*(params.t_f[0]-params.t_switch[0]);
  work.KKT[471] = -6*(params.t_f[0]-params.t_switch[0]);
  work.KKT[481] = -6*(params.t_f[0]-params.t_switch[0]);
  work.KKT[491] = -6*(params.t_f[0]-params.t_switch[0]);
  work.KKT[345] = -1;
  work.KKT[348] = -1;
  work.KKT[351] = -1;
  work.KKT[354] = -1;
  work.KKT[357] = -1;
  work.KKT[360] = -1;
  work.KKT[153] = -1;
  work.KKT[157] = -1;
  work.KKT[161] = -1;
  work.KKT[165] = -1;
  work.KKT[169] = -1;
  work.KKT[173] = -1;
  work.KKT[155] = 1;
  work.KKT[159] = 1;
  work.KKT[163] = 1;
  work.KKT[167] = 1;
  work.KKT[171] = 1;
  work.KKT[175] = 1;
  work.KKT[177] = -1;
  work.KKT[181] = -1;
  work.KKT[185] = -1;
  work.KKT[189] = -1;
  work.KKT[193] = -1;
  work.KKT[197] = -1;
  work.KKT[179] = 1;
  work.KKT[183] = 1;
  work.KKT[187] = 1;
  work.KKT[191] = 1;
  work.KKT[195] = 1;
  work.KKT[199] = 1;
  work.KKT[201] = 1;
  work.KKT[203] = 1;
  work.KKT[205] = 1;
  work.KKT[207] = 1;
  work.KKT[209] = 1;
  work.KKT[211] = 1;
  work.KKT[218] = params.t_switch[0];
  work.KKT[220] = params.t_switch[0];
  work.KKT[222] = params.t_switch[0];
  work.KKT[224] = params.t_switch[0];
  work.KKT[226] = params.t_switch[0];
  work.KKT[228] = params.t_switch[0];
  work.KKT[370] = params.t_switch[0]*params.t_switch[0];
  work.KKT[381] = params.t_switch[0]*params.t_switch[0];
  work.KKT[392] = params.t_switch[0]*params.t_switch[0];
  work.KKT[403] = params.t_switch[0]*params.t_switch[0];
  work.KKT[414] = params.t_switch[0]*params.t_switch[0];
  work.KKT[425] = params.t_switch[0]*params.t_switch[0];
  work.KKT[364] = params.t_switch[0]*params.t_switch[0]*params.t_switch[0];
  work.KKT[375] = params.t_switch[0]*params.t_switch[0]*params.t_switch[0];
  work.KKT[386] = params.t_switch[0]*params.t_switch[0]*params.t_switch[0];
  work.KKT[397] = params.t_switch[0]*params.t_switch[0]*params.t_switch[0];
  work.KKT[408] = params.t_switch[0]*params.t_switch[0]*params.t_switch[0];
  work.KKT[419] = params.t_switch[0]*params.t_switch[0]*params.t_switch[0];
  work.KKT[230] = -1;
  work.KKT[232] = -1;
  work.KKT[234] = -1;
  work.KKT[236] = -1;
  work.KKT[238] = -1;
  work.KKT[240] = -1;
  work.KKT[219] = 1;
  work.KKT[221] = 1;
  work.KKT[223] = 1;
  work.KKT[225] = 1;
  work.KKT[227] = 1;
  work.KKT[229] = 1;
  work.KKT[371] = 2*params.t_switch[0];
  work.KKT[382] = 2*params.t_switch[0];
  work.KKT[393] = 2*params.t_switch[0];
  work.KKT[404] = 2*params.t_switch[0];
  work.KKT[415] = 2*params.t_switch[0];
  work.KKT[426] = 2*params.t_switch[0];
  work.KKT[365] = 3*params.t_switch[0]*params.t_switch[0];
  work.KKT[376] = 3*params.t_switch[0]*params.t_switch[0];
  work.KKT[387] = 3*params.t_switch[0]*params.t_switch[0];
  work.KKT[398] = 3*params.t_switch[0]*params.t_switch[0];
  work.KKT[409] = 3*params.t_switch[0]*params.t_switch[0];
  work.KKT[420] = 3*params.t_switch[0]*params.t_switch[0];
  work.KKT[372] = -1;
  work.KKT[383] = -1;
  work.KKT[394] = -1;
  work.KKT[405] = -1;
  work.KKT[416] = -1;
  work.KKT[427] = -1;
  work.KKT[200] = 1;
  work.KKT[202] = 1;
  work.KKT[204] = 1;
  work.KKT[206] = 1;
  work.KKT[208] = 1;
  work.KKT[210] = 1;
  work.KKT[212] = 1;
  work.KKT[213] = 1;
  work.KKT[214] = 1;
  work.KKT[215] = 1;
  work.KKT[216] = 1;
  work.KKT[217] = 1;
  work.KKT[231] = 1;
  work.KKT[233] = 1;
  work.KKT[235] = 1;
  work.KKT[237] = 1;
  work.KKT[239] = 1;
  work.KKT[241] = 1;
  work.KKT[428] = params.t_f[0]-params.t_switch[0];
  work.KKT[430] = params.t_f[0]-params.t_switch[0];
  work.KKT[432] = params.t_f[0]-params.t_switch[0];
  work.KKT[434] = params.t_f[0]-params.t_switch[0];
  work.KKT[436] = params.t_f[0]-params.t_switch[0];
  work.KKT[438] = params.t_f[0]-params.t_switch[0];
  work.KKT[448] = (params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0]);
  work.KKT[458] = (params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0]);
  work.KKT[468] = (params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0]);
  work.KKT[478] = (params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0]);
  work.KKT[488] = (params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0]);
  work.KKT[498] = (params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0]);
  work.KKT[442] = (params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0]);
  work.KKT[452] = (params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0]);
  work.KKT[462] = (params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0]);
  work.KKT[472] = (params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0]);
  work.KKT[482] = (params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0]);
  work.KKT[492] = (params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0]);
  work.KKT[429] = 1;
  work.KKT[431] = 1;
  work.KKT[433] = 1;
  work.KKT[435] = 1;
  work.KKT[437] = 1;
  work.KKT[439] = 1;
  work.KKT[449] = 2*(params.t_f[0]-params.t_switch[0]);
  work.KKT[459] = 2*(params.t_f[0]-params.t_switch[0]);
  work.KKT[469] = 2*(params.t_f[0]-params.t_switch[0]);
  work.KKT[479] = 2*(params.t_f[0]-params.t_switch[0]);
  work.KKT[489] = 2*(params.t_f[0]-params.t_switch[0]);
  work.KKT[499] = 2*(params.t_f[0]-params.t_switch[0]);
  work.KKT[443] = 3*(params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0]);
  work.KKT[453] = 3*(params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0]);
  work.KKT[463] = 3*(params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0]);
  work.KKT[473] = 3*(params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0]);
  work.KKT[483] = 3*(params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0]);
  work.KKT[493] = 3*(params.t_f[0]-params.t_switch[0])*(params.t_f[0]-params.t_switch[0]);
}
