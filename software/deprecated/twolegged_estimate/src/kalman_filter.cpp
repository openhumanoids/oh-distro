
#include <kalman-filter/kalman_filter.hpp>
#include <iostream>

KalmanFilter::KalmanFilter() {

}


KalmanFilter::KalmanFilter(KalmanFilter_Models::BaseModel &def_model) {
	//std::cout << "KalmanFilter::KalmanFilter(KalmanFilter_Models::BaseModel -- saying hi." << std::endl;

	// TODO -- Add more inheretance KF models here
	// Simpler, but no simpler
	// This will have to be done for each new model we define :-/ but avoids difficulty in variation of parameters with callback approach
	if (_model = dynamic_cast<KalmanFilter_Models::Joint_Model*>(&def_model) ) {
		// we have a joint filter
		//std::cout << "KalmanFilter::KalmanFilter -- Copying pointer for Joint filter" << std::endl;
	} else if (_model = dynamic_cast<KalmanFilter_Models::DataFusion_Model*>(&def_model)) {
		// we have a data fusion filter
		//std::cout << "KalmanFilter::KalmanFilter -- Copying pointer for DataFusion filter" << std::endl;
	}
}

KalmanFilter::~KalmanFilter() {
	//free()
	//delete
	
	
}


void KalmanFilter::Initialize() {
	//std::cout << "KalmanFilter::Initialize -- Initializing." << std::endl;
	_model->identify();
	
	ut_last_priori_update = 0;
	// we need to setup the different matrices to have the correct size
	_model->setup(priori, posterior);
	//std::cout << "KalmanFilter::Initialize -- priori.mu resized to: " << priori.mu.rows() << std::endl;
	
	return;
}

// Priori only update to the internal state variables
void KalmanFilter::step(const unsigned long &ut_now, const VAR_VECTORd &variables) {
	
	//std::cout << "KalmanFilter::step -- Time stepping the filter." << std::endl;
	
	//priori update
	propagatePriori(ut_now, variables, priori.mu, priori.M);
	
	return; // return the results of the filtering process here
}


// Does priori and posterior updates to the internal state variables
void KalmanFilter::step(const unsigned long &ut_now, const VAR_VECTORd &variables, const VAR_VECTORd &measurements) {
	//std::cout << "KalmanFilter::step -- Measurement step on filter states." << std::endl;
	//std::cout << "KalmanFilter::step -- measurements(0): " << measurements(0) << std::endl;
	
	//posterior ( and priori ) updates
	propagatePosterior(ut_now, variables, measurements, propagatePriori(ut_now, variables, priori.mu, priori.M));
	
	
	return; // return the results of the filtering process here
}


KalmanFilter_Models::MatricesUnit KalmanFilter::propagatePriori(const unsigned long &ut_now, const VAR_VECTORd &variables, const VAR_VECTORd &mu, const VAR_MATRIXd &cov) {
	
	//std::cout << "KalmanFilter::propagatePriori -- Time update" << std::endl;
	last_update_type = PRIORI_UPDATE;
	
	double dt;
	
	dt = 1E-6*(ut_now - ut_last_priori_update);
	ut_last_priori_update = ut_now;
	priori.utime = ut_now;
	//std::cout << "KalmanFilter::propagatePriori -- dt = " << dt << std::endl;
	
	// We want to propagate a current state mean and covariance estimate according to the some defined system model
	
	
	// Get continuous dynamics model -> convert to discrete -> propagate state mean and covariance
	KalmanFilter_Models::MatricesUnit cont_matrices, disc_matrices; // why am I keeping a local copy here?
	
	cont_matrices = _model->getContinuousMatrices(mu);

	// s -> z, lti_disc
	disc_matrices = lti_disc(dt, cont_matrices);
	
	// propagate mean (mu)
	if (_model->getSettings().propagate_with_linearized == true) {
		// Propagate the state with transition matrix
		
		priori.mu.resize(mu.size());
		//std::cout << "KalmanFilter::propagatePriori -- mu.size() -- " << mu.size() << "; rows in disc.A -- " << disc_matrices.A.rows() << std::endl;
		//std::cout << disc_matrices.A << std::endl;
		//std::cout << "mu -- " << mu.transpose() << std::endl;
		
		priori.mu = disc_matrices.A * mu;// TODO -- add B*u term, we going to assume this is noise for now
		
	} else {
		std::cerr << "KalmanFilter::propagatePriori -- oops, non-linear propagation not implemented" << std::endl;
				
		// Propagate with non-linear model
		// this should include an integration period and some good integration method
		//priori.mu = _model->propagation_model(post.mu);
	}
	
	// Prepare process covariance matrix
//	std::cout << "KalmanFilter::propagatePriori -- Matrices are: A " << std::endl << disc_matrices.A << std::endl << cov << std::endl;
//	std::cout << "KalmanFilter::propagatePriori -- Qd is" << std::endl << disc_matrices.Q << std::endl;
	
	priori.M = disc_matrices.A * cov * disc_matrices.A.transpose() + disc_matrices.Q;
	
	
	// Compute Kalman Gain matrix
	// We can change this if the need arises, but we shall pass the data required for computing the Kalman gain during the measurement update
	// This is supposedly the more efficient way of doing this, although it does break the abstraction a bit.
	
	
	return cont_matrices;
}


void KalmanFilter::propagatePosterior(const unsigned long &utime_now, const VAR_VECTORd &variables, const VAR_VECTORd &measurements, const KalmanFilter_Models::MatricesUnit &cont_matrices) {
	
	//std::cout << "KalmanFilter::propagatePosterior -- Measurement update" << std::endl;
	
	// Ensure that the priori update did occur
	// Excessive, but we don't want to make a mistake
	if (utime_now != priori.utime) {
		std::cerr << "KalmanFilter::propagatePosterior -- Error, priori update step time does not coincide with the attempted measurement update." << std::endl;
	}
	
	last_update_type = POSTERIOR_UPDATE;
	
//	KalmanFilter_Types::Posterior temp;
	
	//KalmanFilter_Models::MatricesUnit cont_matrices;
	
	// For computation of the Kalman gain and covariance posterior update
	// We need the linearized measurement matrix H
	// This is still in the continuous domain.
	
	// TODO -- some efficiency improvements can be done here
	// Compute the Kalman Gain using covariance from the priori data structure. We assume that this will be correct.
	posterior.S = cont_matrices.C * priori.M * cont_matrices.C.transpose() + cont_matrices.R;
	posterior.K = priori.M * cont_matrices.C.transpose() * posterior.S.inverse();


	// We can use non-linear, or linearized measurement model for obtain the innovation
	if (_model->getSettings().use_linearized_measurement == true) {
		posterior.innov = measurements - cont_matrices.C * priori.mu;
	} else {
		std::cerr << "KalmanFilter::propagatePosterior -- oops, non-linear measurement process not implemented yet." << std::endl;
	}
	
	// update the posterior mean state estimate
	posterior.mu = priori.mu + posterior.K * posterior.innov;
	
	// update the posterior covariance state estimate
	VAR_MATRIXd eye(priori.M.rows(),priori.M.cols());
	eye.setIdentity();
	
	posterior.P = (eye - posterior.K*cont_matrices.C) * priori.M;
	
	return;
}



KalmanFilter_Models::MatricesUnit KalmanFilter::lti_disc(const double &dt, const KalmanFilter_Models::MatricesUnit &cont) {
	
	
	// Compute discrete process noise covariance
	KalmanFilter_Models::MatricesUnit disc;

	//n   = size(F,1);
	//Phi = [F L*Q*L'; zeros(n,n) -F'];
	//AB  = expm(Phi*dt)*[zeros(n,n);eye(n)];
	//Qd   = AB(1:n,:)/AB((n+1):(2*n),:)
	
	disc.A = expm(dt*cont.A);
	
	int n = cont.A.rows();
	
	VAR_MATRIXd shaped_Q;
	
	shaped_Q = cont.V * cont.Q * (cont.V.transpose());
	
//	std::cout << "KalmanFilter::lti_disc -- cont.Q = " << cont.Q << std::endl;
//	std::cout << "KalmanFilter::lti_disc -- cont.V = " << cont.V << std::endl;
//	std::cout << "KalmanFilter::lti_disc -- shaped_Q = " << shaped_Q << std::endl;
	
	// Create and pack the Phi matrix
	VAR_MATRIXd Phi;
	Phi.resize(2*n, 2*n);
	VAR_MATRIXd rhsAB(2*n,n);
	rhsAB.setZero();
	
	int i,j;
	for (i=0;i<n;i++) {
		for (j=0;j<n;j++) {
			// North West
			Phi(i,j) = cont.A(i,j);
			//South West
			Phi(n+i,j) = 0.;
			//North East
			Phi(i,n+j) = shaped_Q(i,j);
			//South East
			Phi(n+i,n+j) = -cont.A(j,i);
		}
		rhsAB(n+i,i) = 1;
	}
	
	
	
	// compute AB
	VAR_MATRIXd AB;
	AB = expm(dt*Phi) * rhsAB;
	
	VAR_MATRIXd over(n,n);
	VAR_MATRIXd under(n,n);
	
	// Compute discrete process noise covariance matrix
	for (i=0;i<n;i++) {
		for (j=0;j<n;j++) {
			//Qd   = AB(1:n,:)/AB((n+1):(2*n),:)
			over(i,j) = AB(i,j);
			under(i,j) = AB(n+i,j);
		}
	}
	disc.Q = over*(under.inverse());
	
	if (false) {
		std::cout << "KalmanFilter::lti_disc -- " << n << " " << shaped_Q.rows() << " " << shaped_Q.cols() << std::endl;
		std::cout << "KalmanFilter::lti_disc -- cont.A = " << cont.A << std::endl;
		std::cout << "KalmanFilter::lti_disc -- Phi = " << std::endl << Phi << std::endl;
		std::cout << "KalmanFilter::lti_disc -- expm(dt*Phi) = " << std::endl << expm(dt*Phi) << std::endl;
		std::cout << "KalmanFilter::lti_disc -- rhsAB = " << std::endl << rhsAB << std::endl;
		std::cout << "KalmanFilter::lti_disc -- AB = " << std::endl << AB << std::endl;
		std::cout << "KalmanFilter::lti_disc -- over = " << std::endl << over << std::endl;
		std::cout << "KalmanFilter::lti_disc -- under = " << std::endl << under << std::endl;
		std::cout << "KalmanFilter::lti_disc -- disc.Q = " << std::endl << disc.Q << std::endl;
	}
	
	return disc;
	
}

/* This is what you should be getting for the joint filter model. 
 * KalmanFilter::lti_disc was debugged for n=2 Joint_Model, with Taylor version of expm
 * n =

     2


Phi =

         0    1.0000    0.0010         0
         0         0         0    0.0050
         0         0         0         0
         0         0   -1.0000         0

Phit =

    1.0000e+000     3.3333e-003     3.3333e-006    27.7777e-009
     0.0000e+000     1.0000e+000   -27.7777e-009    16.6667e-006
     0.0000e+000     0.0000e+000     1.0000e+000     0.0000e+000
     0.0000e+000     0.0000e+000    -3.3333e-003     1.0000e+000
     
     expm
     1.0000e+000     3.3333e-003     3.3333e-006    27.7777e-009
     0.0000e+000     1.0000e+000   -27.7777e-009    16.6667e-006
     0.0000e+000     0.0000e+000     1.0000e+000     0.0000e+000
     0.0000e+000     0.0000e+000    -3.3333e-003     1.0000e+000

AB =

    3.3333e-006    27.7777e-009
   -27.7777e-009    16.6667e-006
     1.0000e+000     0.0000e+000
    -3.3333e-003     1.0000e+000

over =

     3.3333e-006    27.7777e-009
   -27.7777e-009    16.6667e-006


under =

     1.0000e+000     0.0000e+000
    -3.3333e-003     1.0000e+000

Q =

   1.0e-04 *

    0.0333    0.0003
    0.0003    0.1667
 
 * */




// Return a copy of the internal Kalman Filter state estimate and Covariance
// Will return the structure with variables indicating whether the last was a priori of posterior update, including the utime of the last update
KalmanFilter_Types::State KalmanFilter::getState() {
	KalmanFilter_Types::State state;
	
	state.last_update_type = last_update_type;
	
	if (last_update_type == PRIORI_UPDATE) {
		state.utime = priori.utime;
		
		state.X = priori.mu;
		state.Cov = priori.M;
	} else {
		state.utime = posterior.utime;
		
		state.X = posterior.mu;
		state.Cov = posterior.P;
	}
		
	return state;
}


// TODO - This function should be updated to include the Pade method, as suggested by Twan.
VAR_MATRIXd KalmanFilter::expm(const VAR_MATRIXd &Ft) {
	
	VAR_MATRIXd Phi;
	VAR_MATRIXd eye;
	eye.setIdentity(Ft.rows(), Ft.cols());
	
	return (eye + Ft + 0.5*Ft*Ft);
}



