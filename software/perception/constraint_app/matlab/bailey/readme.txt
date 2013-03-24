MatLab Utility Functions
------------------------

This collection are m-files I have written over the years that are reusable and I have found very useful in many projects. Some of them are listed below.

See Contents.m for a full alphabetic list of functions.

Warning: the remainder of this readme is out-of-date. Go to Contents.m and the demo scripts for current information.

Kalman filter
-------------

KF_simple_update.m: 	Basic implementation of KF update.
KF_joseph_update.m: 	Numerically stable KF update (Joseph form).
KF_cholesky_update.m: 	Numerically stable KF update (best).
KF_IEKF_update.m: 	Iterated EKF update.

numerical_Jacobian.m: 	Compute an approximate Jacobian matrix for any non-linear function.
chi_square_bound.m: 	Compute a threshold for an innovation gate that encloses a specified probability mass. 
inv_posdef:		Compute the inverse of a positive-definite matrix

Demo filter application:
demo_ekf_filter.m

Unscented filter
----------------

Note, these implementations are of the basic unscented transform. For more sophisticated versions refer to the recent literature, eg:

	Julier S.J. and Uhlmann J.K., Unscented Filtering and Nonlinear Estimation, 
	Proceedings of the IEEE, pp 401-422, Volume 92, Number 3, 2004.

Two general-purpose functions:
unscented_transform.m: 	Transform a mean and covariance through a non-linear function.
unscented_update.m:	Perform an unscented Kalman update step with non-linear observe model.

Demo filter application:
demo_unscented_filter.m 

Note: The new versions of unscented_transform.m and unscented_update.m require 'vectorised' models. To 
operate with simple (non-vectorised) models, you may wish to use an old version of the functions. See the 'obsolete' directory and the '_old2' versions.

Particle filter
---------------

gauss_samples.m: 	Generate a set of samples from a multi-dimensional Gaussian.
gauss_likelihood:	Compute the likelihood of a set of innovations.
gauss_regularise:	Add jitter to particles after resampling according to Gaussian kernel.
stratified_random.m: 	Generate a sorted set of random numbers in range (0,1).
stratified_resample.m: 	Resample step for a particle filter.
sample_mean.m: 		Compute the mean and variance of a set of samples.

Demo filter application:
demo_particle_filter.m

2-D geometric transforms
------------------------

transform_to_global.m:	Convert a local coordinate to the global frame.
transform_to_relative.m:Convert a global coordinate to a local frame.
pi_to_pi.m: 		Normalise a polar value to within plus-minus pi.

Animation Utilities
-------------------

line_plot_conversion.m:	Convert an array of line-segments to a single polyline separated by NaNs.
sigma_ellipse.m: 	Create a polyline for an n-sigma covariance ellipse.
