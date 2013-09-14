#ifndef KF_MODELS_HPP_
#define KF_MODELS_HPP_

#include <iostream>
#include <Eigen/Dense>

#include <leg-odometry/kf_conversion_definitions.hpp>
#include <leg-odometry/KalmanFilter_Types.hpp>

#define JOINT_MODEL 0
#define DATAFUSION_MODEL 1

namespace KalmanFilter_Models {


struct ModelSettings {
	bool propagate_with_linearized;
	bool analytical_jacobian_available;
	
	int state_size;
};

class MatricesUnit {
	
public:
	VAR_MATRIXd A;
	VAR_MATRIXd B;
	VAR_MATRIXd C;
	VAR_MATRIXd D;
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

	virtual VAR_MATRIXd anaylitical_jacobian(const VAR_MATRIXd &state) = 0;
	virtual VAR_VECTORd propagation_model(const VAR_VECTORd &post) = 0;
	virtual VAR_VECTORd measurement_model(VAR_VECTORd Param) = 0;
	
	virtual void identify() = 0;
	
	const ModelSettings &getSettings() const {return settings;}
	
};



class Joint_Model : public BaseModel {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Joint_Model();
	
	VAR_MATRIXd anaylitical_jacobian(const VAR_MATRIXd &state);
	VAR_VECTORd propagation_model(const VAR_VECTORd &post);
		
	VAR_VECTORd measurement_model(VAR_VECTORd Param);
	
	void identify();
	
};


class DataFusion_Model : public BaseModel {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	DataFusion_Model();
	
	VAR_MATRIXd anaylitical_jacobian(const VAR_MATRIXd &state);
	VAR_VECTORd propagation_model(const VAR_VECTORd &post);
	VAR_VECTORd measurement_model(VAR_VECTORd Param);
	
	void identify();
	
};


}

#endif /*KF_MODELS_HPP_*/
