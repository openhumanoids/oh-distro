
#include <leg-odometry/kalman_filter.hpp>

#include <iostream>

KalmanFilter::KalmanFilter() {

}


KalmanFilter::KalmanFilter(KalmanFilter_Models::BaseModel &def_model) {
	std::cout << "KalmanFilter::KalmanFilter(KalmanFilter_Models::BaseModel -- saying hi." << std::endl;
	
	// Dont want to do this
	if (_model = dynamic_cast<KalmanFilter_Models::Joint_Model*>(&def_model) ) {
		// we have a joint filter
		std::cout << "KalmanFilter::KalmanFilter -- Copying pointer for Joint filter" << std::endl;
	} else if (_model = dynamic_cast<KalmanFilter_Models::DataFusion_Model*>(&def_model)) {
		// we have a data fusion filter
		std::cout << "KalmanFilter::KalmanFilter -- Copying pointer for DataFusion filter" << std::endl;
	}
}

KalmanFilter::~KalmanFilter() {
	//free()
	
}


void KalmanFilter::Initialize() {
	_model->identify();
	
	return;
}


KalmanFilter_Types::Priori KalmanFilter::propagatePriori(const unsigned long &ut_now, const KalmanFilter_Types::Posterior &post, const VAR_VECTORd &input) {
	
	KalmanFilter_Types::Priori priori;
	
	double dt;
	
	dt = 1E-6*(ut_now - post.utime);
	
	std::cout <<"test " << _model->getSettings().propagate_with_linearized <<std::endl;
	
	// We want to propagate a current state mean and covariance estimate according to the some defined system model
	
	// propagate mu
	
	
	if (_model->getSettings().propagate_with_linearized) {
		// Propagate the state with transition matrix
		
		// Get continuous dynamics model -> convert to discrete -> propagate state mean and covariance
		//VAR_MATRIXd F;
		//VAR_MATRIXd Phi;
				
		KalmanFilter_Models::MatricesUnit cont_matrices, disc_matrices;
		
		
		cont_matrices = _model->getContinuousMatrices(post.mu);
		
		// s -> z
		
		//lti_disc
		disc_matrices = lti_disc(dt, cont_matrices);
		
		priori.mu = disc_matrices.A * post.mu;
		
		std::cout <<"test2" <<std::endl;
		
	} else {
		
		// Propagate with non-linear model
		// this should include an integration period and some good integration method
		//priori.mu = _model->propagation_model(post.mu);
	}
	
	
	// Prepare process covariance matrix
	// Compute dynamics matrix
	// Compute state transition and discrete process covariance matrices
	
	// Compute priori covariance matrix
	
	// Compute Kalman Gain matrix
	
	
	return priori;
}



KalmanFilter_Types::Posterior KalmanFilter::propagatePosterior() {
	
	KalmanFilter_Types::Posterior temp;
	
	
	return temp;
}


void KalmanFilter::define_model() {
	
	
	return;
}

KalmanFilter_Models::MatricesUnit KalmanFilter::lti_disc(const double &dt, const KalmanFilter_Models::MatricesUnit &cont) {
	
	
	// Compute discrete process noise covariance
	KalmanFilter_Models::MatricesUnit disc;
			
	//n   = size(F,1);
	//Phi = [F L*Q*L'; zeros(n,n) -F'];
	//AB  = expm(Phi*dt)*[zeros(n,n);eye(n)];
	//Qd   = AB(1:n,:)/AB((n+1):(2*n),:)
	
	disc.A = expm(dt, cont.A);
	
	int n = cont.A.rows();
	
	VAR_MATRIXd shaped_Q;
	
	shaped_Q = cont.B * cont.Q * (cont.B.transpose());
	
	// Create and pack the Phi matrix
	VAR_MATRIXd Phi(2*n, 2*n);
	VAR_MATRIXd rhsAB(2*n,2);
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
		rhsAB(i,i) = 1;
	}
	
	// compute AB
	VAR_MATRIXd AB;
	AB = expm(dt,Phi) * rhsAB;
	
	VAR_MATRIXd over(n,n);
	VAR_MATRIXd under(n,n);
	
	// Compute discrete process noise covariance matrix
	for (i=0;i<n;i++) {
		for (j=0;j<n;j++) {
			//Qd   = AB(1:n,:)/AB((n+1):(2*n),:)
			over(i,j) = AB(i,j);
			under(i,j) = AB(n+1,j);
		}
	}
	
	disc.Q = over*(under.inverse());
	
	return disc;
	
}


VAR_MATRIXd KalmanFilter::expm(const double &dt, const VAR_MATRIXd &F) {
	
	VAR_MATRIXd Phi;
	VAR_MATRIXd eye;
	eye.setIdentity(F.rows(), F.cols());
	
	return (eye + dt*F + 0.5*dt*dt*F*F);
}



