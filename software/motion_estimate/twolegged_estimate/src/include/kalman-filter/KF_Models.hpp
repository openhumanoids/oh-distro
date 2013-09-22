#ifndef KF_MODELS_HPP_
#define KF_MODELS_HPP_

#include <iostream>
#include <Eigen/Dense>

#include <kalman-filter/kf_conversion_definitions.hpp>
#include <kalman-filter/KalmanFilter_Types.hpp>

#define JOINT_MODEL 0
#define DATAFUSION_MODEL 1

#define NOT_IMPL "THIS HAS NOT BEEN IMPLEMENTED YET"

namespace KalmanFilter_Models {


struct ModelSettings {
	bool propagate_with_linearized;
	bool analytical_jacobian_available;
	bool use_linearized_measurement;
	
	int state_size;
};

class MatricesUnit {
	
public:
	VAR_MATRIXd A;
	VAR_MATRIXd B;
	VAR_MATRIXd C;
	VAR_MATRIXd D;
	VAR_MATRIXd V; // noise shaping matrix. Defined separately, as we may want to separately introduce input u and noise shaping to the state space form
	VAR_MATRIXd Q;
	VAR_MATRIXd R;
	
	MatricesUnit();
};


// models should implement this base abstract class
class BaseModel {
protected:
	ModelSettings settings;
	MatricesUnit continuous_matrices;
	MatricesUnit discrete_matrices;
	
public:

	// definition and parameter functions
	virtual VAR_MATRIXd anaylitical_jacobian(const VAR_MATRIXd &state) = 0;
	virtual VAR_MATRIXd continuous_process_noise(const VAR_MATRIXd &state) = 0;
	virtual VAR_MATRIXd measurement_noise_cov(const VAR_VECTORd &state) = 0;
	virtual VAR_MATRIXd noise_shaping_matrix(const VAR_VECTORd &state) = 0;
	
	// abstracts for f(x) and h(x)
	// These must be implmented, but do not have to be used. This is set thorugh the settings booleans.
	// must be virtual for overriden function calls
	virtual VAR_VECTORd propagation_model(const VAR_VECTORd &post) = 0;
	virtual VAR_VECTORd measurement_model(VAR_VECTORd Param) = 0;
	
	virtual void identify() = 0;
	
	ModelSettings getSettings() {return settings;}
	MatricesUnit getContinuousMatrices(const VAR_VECTORd &state);
	void setup(KalmanFilter_Types::Priori &priori, KalmanFilter_Types::Posterior &posterior);
	
	void getKFDebugData();
};



class Joint_Model : public BaseModel {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Joint_Model();
	
	// Q and R noise contribution functions
	VAR_MATRIXd continuous_process_noise(const VAR_MATRIXd &state);
	VAR_MATRIXd measurement_noise_cov(const VAR_VECTORd &state);
	
	// Analytical J used rather than numerical
	VAR_MATRIXd anaylitical_jacobian(const VAR_MATRIXd &state);

	// f(x) and h(x) functional definition
	VAR_VECTORd propagation_model(const VAR_VECTORd &post);	
	VAR_VECTORd measurement_model(VAR_VECTORd Param);
	
	VAR_MATRIXd noise_shaping_matrix(const VAR_VECTORd &state);
	
	void identify();
	
};


class DataFusion_Model : public BaseModel {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	DataFusion_Model();
	
	// Q and R noise definition functions
	VAR_MATRIXd continuous_process_noise(const VAR_MATRIXd &state);
	VAR_MATRIXd measurement_noise_cov(const VAR_VECTORd &state);
	
	// Analytical J used rather than numerical
	VAR_MATRIXd anaylitical_jacobian(const VAR_MATRIXd &state);
	
	// f(x) and h(x) functional definition
	VAR_VECTORd propagation_model(const VAR_VECTORd &post);
	VAR_VECTORd measurement_model(VAR_VECTORd Param);
	
	VAR_MATRIXd noise_shaping_matrix(const VAR_VECTORd &state);
	
	void identify();
	
};


}

#endif /*KF_MODELS_HPP_*/
