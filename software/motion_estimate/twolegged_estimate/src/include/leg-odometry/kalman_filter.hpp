#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <leg-odometry/KalmanFilter_Types.hpp>
#include <leg-odometry/KF_Models.hpp>

class KalmanFilter {
private:
	
	int state_size;
	KalmanFilter_Types::Priori priori;
	KalmanFilter_Types::Posterior posterior;
	
	KalmanFilter_Models::BaseModel* _model;
	
	KalmanFilter_Models::MatricesUnit lti_disc(const double &dt, const KalmanFilter_Models::MatricesUnit &cont);
	
	VAR_MATRIXd expm(const double &dt, const VAR_MATRIXd &F);
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	KalmanFilter();
	KalmanFilter(KalmanFilter_Models::BaseModel &def_model);
	~KalmanFilter();
	
	void Initialize();
	
	KalmanFilter_Types::Priori propagatePriori(const unsigned long &ut_now, const KalmanFilter_Types::Posterior &post);
	
	KalmanFilter_Types::Posterior propagatePosterior();
	
	
	void define_model();
	
	
	// we will add the callback method later
	//void define_model(/* callback for continuous f, */ /* callback for continuous h */);
	
};




#endif /*KALMAN_FILTER_HPP_*/
