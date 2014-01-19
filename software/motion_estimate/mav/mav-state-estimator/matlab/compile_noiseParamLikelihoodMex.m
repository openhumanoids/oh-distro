function compile_noiseParamLikelihoodMex()
addpath_matlab_utils;
cflags_str = mex_pkg_config('mav-state-est-noise-id');
mex('-v','noiseParamLikelihoodMex.cpp',cflags_str)
rmpath_matlab_utils;