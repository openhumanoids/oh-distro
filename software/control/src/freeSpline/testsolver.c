/* Produced by CVXGEN, 2015-03-26 23:25:31 -0400.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.t_switch[0] = 0.20319161029830202;
  params.t_f[0] = 0.8325912904724193;
  params.r0[0] = -0.8363810443482227;
  params.r0[1] = 0.04331042079065206;
  params.r0[2] = 1.5717878173906188;
  params.r0[3] = 1.5851723557337523;
  params.r0[4] = -1.497658758144655;
  params.r0[5] = -1.171028487447253;
  params.rd0[0] = -1.7941311867966805;
  params.rd0[1] = -0.23676062539745413;
  params.rd0[2] = -1.8804951564857322;
  params.rd0[3] = -0.17266710242115568;
  params.rd0[4] = 0.596576190459043;
  params.rd0[5] = -0.8860508694080989;
  params.rf[0] = 0.7050196079205251;
  params.rf[1] = 0.3634512696654033;
  params.rf[2] = -1.9040724704913385;
  params.rf[3] = 0.23541635196352795;
  params.rf[4] = -0.9629902123701384;
  params.rf[5] = -0.3395952119597214;
  params.rdf[0] = -0.865899672914725;
  params.rdf[1] = 0.7725516732519853;
  params.rdf[2] = -0.23818512931704205;
  params.rdf[3] = -1.372529046100147;
  params.rdf[4] = 0.17859607212737894;
  params.rdf[5] = 1.1212590580454682;
  params.r_switch_lb[0] = -0.774545870495281;
  params.r_switch_lb[1] = -1.1121684642712744;
  params.r_switch_lb[2] = -0.44811496977740495;
  params.r_switch_lb[3] = 1.7455345994417217;
  params.r_switch_lb[4] = 1.9039816898917352;
  params.r_switch_lb[5] = 0.6895347036512547;
  params.r_switch_ub[0] = 1.6113364341535923;
  params.r_switch_ub[1] = 1.383003485172717;
  params.r_switch_ub[2] = -0.48802383468444344;
  params.r_switch_ub[3] = -1.631131964513103;
  params.r_switch_ub[4] = 0.6136436100941447;
  params.r_switch_ub[5] = 0.2313630495538037;
  params.rd_switch_lb[0] = -0.5537409477496875;
  params.rd_switch_lb[1] = -1.0997819806406723;
  params.rd_switch_lb[2] = -0.3739203344950055;
  params.rd_switch_lb[3] = -0.12423900520332376;
  params.rd_switch_lb[4] = -0.923057686995755;
  params.rd_switch_lb[5] = -0.8328289030982696;
  params.rd_switch_ub[0] = -0.16925440270808823;
  params.rd_switch_ub[1] = 1.442135651787706;
  params.rd_switch_ub[2] = 0.34501161787128565;
  params.rd_switch_ub[3] = -0.8660485502711608;
  params.rd_switch_ub[4] = -0.8880899735055947;
  params.rd_switch_ub[5] = -0.1815116979122129;
}
