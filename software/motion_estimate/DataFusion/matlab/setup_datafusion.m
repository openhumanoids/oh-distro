function setup_datafusion()

cd([getenv('DRC_BASE'),'/software/motion_estimate/DataFusion/matlab']);

addjars;
addpath('generic_functions');
addpath('generic_functions/kf');

