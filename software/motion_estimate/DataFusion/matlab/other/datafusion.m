function [delta_pose, state] = datafusion(predicted_pose, reference_pose, prev_state, parameters)
% interface to the Kalman Filter data fusion computations
% estimate INS/IMU errors from some truth measurement, such as LegOdometry,
% but this may be expaned to Visual Odometry in the future
% returns the error estimate as a delta_pose and the updated state of the
% Kalman Filter
% Takes the INS predicted pose and measurement reference pose, the
% previous_state of the Kalman Filter state and covariance and parameters
% pertaining to the current estimation update (covariances, etc.)

% do we separate the time and measurement update cycles for the filter
% this will allow us to easily change the rate at which measurements are
% added. Separate measurement cycles via LCM will also ease the integration
% of different measurement sources, including visual odometry.
% separating these here will also simplify the feed forward to feed back
% conversion process, since a varying amount of estimated error will be fed
% back into the INS and IMU solutions.
% The disadvantage is that there are more LCM message definitions and more
% LCM traffic

% define the states used

% compute the Jacobian for first order dynamics
% kf.cont.F = 
% kf.cont.G =  
% kf.cont.Q =

% z-tranform assumption
% kf.disc.Phi = 
% kf.disc.Qd = 


% prediction update
% kf.prior.x =
% kf.prior.M = 
% kf.prior.K = 

% LegOdometry measurement update
% kf.meas.LO.residual = 
% kf.meas.LO.R =

% kf.posterior.x = 
% kf.posterior.P = 
