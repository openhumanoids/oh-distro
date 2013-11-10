function setup_datafusion()

cd([getenv('DRC_BASE'),'/software/motion_estimate/DataFusion/matlab']);

% Add DRC lcmtypes
javaaddpath([getenv('DRC_BASE'),'/software/build/share/java/lcmtypes_drc_lcmtypes.jar'])

% Add path to lcm.jar, to allow java execution of lcm code
[retval,cp] = system('pkg-config --variable=classpath lcm-java');
if (retval==0 && ~isempty(cp))
    disp(' Added the lcm jar to your javaclasspath (found via pkg-config)');
    javaaddpath(strtrim(cp));
end

% Add specific Kalman Filter and other functions -- to be used by the Data
% Fusion process
addpath('generic_functions');
addpath('generic_functions/kf');
addpath('motion_simulator');
addpath('AtlasSpecific')
addpath('INS')
