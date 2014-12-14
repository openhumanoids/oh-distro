/*
 * AdjustableVector.h
 *
 *  Created on: Feb 24, 2012
 *      Author: mfleder
 */

#ifndef ADJUSTABLEVECTOR_H_
#define ADJUSTABLEVECTOR_H_


#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

namespace surrogate_gui
{

	/**Class for modify and drawing a vector in R3 that starts at some point (possibly non-origin).
	 * Supports drawing the vector, rotating it, and draw the circles for rotation*/
	class AdjustableVector
	{
		//------fields-----
		private:
			pcl::PointXYZ _unitOriginDir; 		  //this as unit vector, starting at origin
			pcl::PointXYZ _origin;			  	  //origin from which translation's are made (in R3)
												  //For instance, this could be the centroid of an object
												  //being tracked.  Or it could always be (0,0,0)

			pcl::PointXYZ _offset;				  //offset from _origin at which the vector starts

			double _length; 					  //vector length
			bool _updateOrientationCalled; 		  //user adjusted the vector's orientation?
			bool _translateCalled;				  //translateAlongAxis called?
			bool _valuesInitialized;			  //has the mutator set been called
			int8_t _basis_multipliers[3];		  //multiplying each basis vector by the signs in here
												  //we do this to try and return basis vectors that change continuously,
												  //and avoid a basis vector from changing sign.
												  //these values are all {+-1}
			pcl::PointXYZ _lastBasis[3]; //last basis vectors computed

			bool _displaySelected;

		public:
			const double ANGLE_INC; //amount by which these angles change w/ keyboard/mouse clicks
			const double LENGTH_INC; //amount by which the vector length increases / decreases
			const double TRANSLATE_INC; //increment by which we translate along an axis
			bool _displayAxes;

		//-------constructor/destructor
		public:
			AdjustableVector();
			virtual ~AdjustableVector();

		//------observers
		public:
			pcl::PointXYZ getVectorUnitDir(void) const;

			void getVectorStartEndNonUnit(pcl::PointXYZ &startPt,
										  pcl::PointXYZ &endPt) const;

		private:
			void getOrthonormBasisVectorCircles(pcl::PointXYZ &v1, //circles lie in planes (v1,v2) and (v1,v3)
												pcl::PointXYZ &v2,
												pcl::PointXYZ &v3,
												bool rotatingPhi);

		public:
			bool updateOrientationCalled() const;
			bool translateCalled() const;
			bool basisInitialized() const;
			bool valuesInitialized() const;
			bool displaySelected() const;

		//---------mutators
		public:
			void displaySelected(bool faint); //if false, will display faint (not selected by user)
			void updateLength(double lengthChange);
			void updateOrientation(double thetaPlane1_inc, double phiPlane2_inc);
			void updateOrigin(const pcl::PointXYZ &origin);
			void set(const pcl::PointXYZ &originDir, const pcl::PointXYZ &origin, double lengthInc,
					 const pcl::PointXYZ &offsetFromOrigin);
			void translateAlongAxis(double v1Inc, double v2Inc, double v3Inc); //increments along v1,v2,v3 axes
			void reset(); //reset to initial state

		//----draw
		public:
			void draw() const;

		//-- "imported from PclSurrogateUtils
		private:
			static Eigen::Vector3f toVec(const pcl::PointXYZ &p);
			static pcl::PointXYZ toPt(const Eigen::Vector3f &p);
	}; //class AdjustableVector


} //namespace surrogate_gui
#endif /* ADJUSTABLEVECTOR_H_ */
