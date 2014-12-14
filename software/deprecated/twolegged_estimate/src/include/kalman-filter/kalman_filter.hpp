#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <kalman-filter/KalmanFilter_Types.hpp>
#include <kalman-filter/KF_Models.hpp>

#define PRIORI_UPDATE 0
#define POSTERIOR_UPDATE 1


class KalmanFilter {
private:
	
	int state_size;
	
	KalmanFilter_Types::Priori priori;
	KalmanFilter_Types::Posterior posterior;
	unsigned long ut_last_priori_update;
	int last_update_type;
	
	KalmanFilter_Models::BaseModel* _model;
	
	KalmanFilter_Models::MatricesUnit lti_disc(const double &dt, const KalmanFilter_Models::MatricesUnit &cont);
	
	VAR_MATRIXd expm(const VAR_MATRIXd &Ft);
	
	//Does this require mu and cov to be passed from outside?
	KalmanFilter_Models::MatricesUnit propagatePriori(const unsigned long &ut_now, const VAR_VECTORd &variables, const VAR_VECTORd &mu, const VAR_MATRIXd &cov);
	
	void propagatePosterior(const unsigned long &utime_now, const VAR_VECTORd &variables, const VAR_VECTORd &measurements, const KalmanFilter_Models::MatricesUnit &cont_matrices);
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	KalmanFilter();
	KalmanFilter(KalmanFilter_Models::BaseModel &def_model);
	~KalmanFilter();
	
	void Initialize();
	
	// Various filter stepping functions
	void step(const unsigned long &ut_now, const VAR_VECTORd &variables);
	void step(const unsigned long &ut_now, const VAR_VECTORd &variables, const VAR_VECTORd &measurements);
	
	KalmanFilter_Types::State getState();
	
	// we will add the callback method later
	//void define_model(/* callback for continuous f, */ /* callback for continuous h */);
	
};




#endif /*KALMAN_FILTER_HPP_*/
