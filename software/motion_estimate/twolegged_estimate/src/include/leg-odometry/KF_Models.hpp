#ifndef KF_MODELS_HPP_
#define KF_MODELS_HPP_

namespace KalmanFilter_Models {

// models should implement this base abstract class
class BaseModel {
public:
	virtual void propagation_model(int Param) = 0;
	virtual void measurement_model(int Param) = 0;
	
};



class Joint_Model : public BaseModel {
private:
	
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	void propagation_model(int Param) { ;}
	void measurement_model(int Param) { ;}
	
};


class DataFusion_Model : public BaseModel {
private:
	
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	void propagation_model(int Param) { ;}
	void measurement_model(int Param) { ;}
	
};


}

#endif /*KF_MODELS_HPP_*/
