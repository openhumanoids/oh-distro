/*
 * AdjustableVector.cpp
 *
 *  Created on: Feb 24, 2012
 *      Author: mfleder
 */

#include "AdjustableVector.h"
#include "LinearAlgebra.h"
#include "SurrogateException.h"
#include <perception/PclSurrogateUtils.h>

#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

#include <GL/glu.h>

using namespace pcl;
using namespace std;

namespace surrogate_gui
{

	//==============constructor/destructor
	AdjustableVector::AdjustableVector()
	: _length(0),
	  _updateOrientationCalled(false), _valuesInitialized(false),
	  _displaySelected(false),
	  ANGLE_INC(0.08), LENGTH_INC(0.08),
	  TRANSLATE_INC(0.02),
	  _displayAxes(false)
	{
		reset();
	}

	AdjustableVector::~AdjustableVector()
	{}

	//================observers
	bool AdjustableVector::valuesInitialized() const
	{
		return _valuesInitialized;
	}

	bool AdjustableVector::basisInitialized() const
	{
		for (int i = 0; i < 3; i++)
		{
			if (_lastBasis[i].x == 0 ||
				_lastBasis[i].y == 0 ||
				_lastBasis[i].z == 0)
				return false; //0 vector
		}

		//none of the vectors are the 0 vector
		return true;
	}

	bool AdjustableVector::updateOrientationCalled() const
	{
		return _updateOrientationCalled;
	}

	bool AdjustableVector::translateCalled() const
	{
		return _translateCalled;
	}


	/**@return getVectorUnitOrigin translated to (_offset + _origin)
	 * with length adjusted
	 *
	 * startPt is the start point in R3 of this vector
	 * endPt is the end of this vector*/
	void AdjustableVector::getVectorStartEndNonUnit(PointXYZ &startPt,
													PointXYZ &endPt) const
	{
		if (!_valuesInitialized)
				throw SurrogateException("getVectorStartEndNonUnit: Vector not yet initialized");

		//--start pt
		Eigen::Vector3f startPt3f = toVec(_origin) + toVec(_offset);

		//---direction to translate from startPt
		Eigen::Vector3f unitVector = toVec(getVectorUnitDir());

		//length is amt to translate
		Eigen::Vector3f endPt3f = startPt3f
								+ (_length * unitVector);

		//-----output
		startPt = toPt(startPt3f);
		endPt = toPt(endPt3f);
	}


	/**@return the user-adjusted force vector or the initial default (plane-normal)
	 * The return value is a unit-vector with start point at the origin (0,0,0)*/
	PointXYZ AdjustableVector::getVectorUnitDir(void) const
	{
		if (!_valuesInitialized)
			throw SurrogateException("getVectorUnitOrigin: Vector not yet initialized");

		return _unitOriginDir;
	}

	bool AdjustableVector::displaySelected() const
	{
		return _displaySelected;
	}

	//====================draw
	void AdjustableVector::draw() const
	{
		if (!_valuesInitialized)
			throw SurrogateException("can't draw vector w/o first initializing");

		if (!basisInitialized())
			return; //nothing to draw yet: haven't computed basis yet
			//getOrthonormBasisVectorCircles(v1u, v2u, v3u, true); //use last v2


		//---------------do blending if not selected
		if (!_displaySelected) //if not selected, blend w/ black to fade
		{
			glEnable(GL_BLEND); //for dimming contrast
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		}
		const double ALPHA_CLOUD_UB = 0.1; //add as alpha

		//--------------

		PointXYZ vecStart, vecEnd;
		getVectorStartEndNonUnit(vecStart, vecEnd);

		//========draw vector line
		glColor4ub(0,0,0, 255); //black
		glColor4f(1,0,0, ALPHA_CLOUD_UB); //red
		glPointSize(5.0f);
		glBegin(GL_POINTS);
		for (double a = 0; a <= 1; a+=0.01)
		{
			glVertex3f((1-a)*vecStart.x + a*vecEnd.x,
					   (1-a)*vecStart.y + a*vecEnd.y,
					   (1-a)*vecStart.z + a*vecEnd.z);
		}
		glEnd(); //GL_POINTS


		//======draw arrow on end of vector
		PointXYZ v1u,v2u,v3u; //circles lie in planes {v1u, v2u} and {v1u, v3u}

		v1u = _lastBasis[0];
		v2u = _lastBasis[1];
		v3u = _lastBasis[2];


		//the force vector lies along v1u



		//-------draw the basis vectors---

		if (_displayAxes)
		{
			//determine end pts
			PointXYZ v1uTrans = LinearAlgebra::add(v1u, vecStart);
			PointXYZ v2uTrans = LinearAlgebra::add(v2u, vecStart); //translated origin for axis is at the vector start
			PointXYZ v3uTrans = LinearAlgebra::add(v3u, vecStart); // ' '

			glBegin(GL_POINTS);
			for (double a = -.5; a <= 0.5; a+=0.01)
			{
				glColor4ub(0,0,0, 255); //black
				glColor4f(1,1,0, ALPHA_CLOUD_UB);
				glVertex3f((1-a)*vecStart.x + a*v1uTrans.x,
						   (1-a)*vecStart.y + a*v1uTrans.y,
						   (1-a)*vecStart.z + a*v1uTrans.z);

				glColor4ub(0,0,0, 255); //black
				glColor4f(1,0.5,0.5, ALPHA_CLOUD_UB);
				glVertex3f((1-a)*vecStart.x + a*v2uTrans.x,
						   (1-a)*vecStart.y + a*v2uTrans.y,
						   (1-a)*vecStart.z + a*v2uTrans.z);

				glColor4ub(0,0,0, 255); //black
				glColor4f(1,0,1, ALPHA_CLOUD_UB);
				glVertex3f((1-a)*vecStart.x + a*v3uTrans.x,
						   (1-a)*vecStart.y + a*v3uTrans.y,
						   (1-a)*vecStart.z + a*v3uTrans.z);
			}
			glEnd(); //GL_POINTS
		}
		//------

		glColor4ub(0,0,0, 255); //black
		glColor4f(1,0,0, ALPHA_CLOUD_UB); //red
		glBegin(GL_TRIANGLE_FAN);

		glNormal3f(v1u.x, v1u.y, v1u.z); //vector is along this axis
		glVertex3f(vecEnd.x, vecEnd.y, vecEnd.z); //tip of arrow is the end pt
		//half way between start and end points
		PointXYZ coneBottom = LinearAlgebra::mult(0.5, LinearAlgebra::add(vecStart, vecEnd)); //average
		for (double angle = 0; angle <= 2*M_PI; angle += M_PI/12.0)
		{
			PointXYZ nextPt = LinearAlgebra::getCirclePt(v2u, v3u,
														 0.02, //radius
														 angle,
														 coneBottom);
			glVertex3f(nextPt.x, nextPt.y, nextPt.z);
		}

		glEnd();  //GL_TRIANGLE_FAN


		//========draw circles to indicate rotation axes
		//planes in which the circles lie: {v1u, v2u} ; {v1u, v3u}
		const double CIRCLE_DEGREES_TO_DRAW = M_PI_2;

		//compute the max distance from the vector point to draw a circle
		//by the law of cosines, this is:  r*sqrt[2-2*cos(CIRCLE_DEGREES_TO_DRAW/2)]
		double max_dist = _length*sqrt(2-2*cos(CIRCLE_DEGREES_TO_DRAW/2.0));

		glPointSize(3.0);
		glBegin(GL_POINTS);

		for (double a = 0; a < 2*M_PI; a+=0.01)
		{
			glColor4ub(0,0,0, 255); //black
			glColor4f(0,0,1, ALPHA_CLOUD_UB); //blue
			//compute distance from circle pt to vector end
			PointXYZ nextCircPt = LinearAlgebra::getCirclePt(v1u, v2u, _length, a, vecStart); //order of vecs is important
			if (euclideanDistance(nextCircPt, vecEnd) < max_dist) //not too far from end?
				glVertex3f(nextCircPt.x, nextCircPt.y, nextCircPt.z);

			glColor4ub(0,0,0, 255); //black
			glColor4f(0,1,1, ALPHA_CLOUD_UB); //other circle
			nextCircPt = LinearAlgebra::getCirclePt(v1u, v3u, _length, a, vecStart); //order of vecs is important
			if (euclideanDistance(nextCircPt, vecEnd) < max_dist) //not too far from end?
				glVertex3f(nextCircPt.x, nextCircPt.y, nextCircPt.z);
		}

		glEnd(); //GL_POINTS

		//------
		if (!_displaySelected)
			glDisable(GL_BLEND); //was turned on at the top
	}


	/**Unit vectors starting from origin
	 * circles lie in planes (v1,v2) and (v1,v3),
	 * have center _offset
	 * */
	void AdjustableVector::getOrthonormBasisVectorCircles(PointXYZ &v1u,
														  PointXYZ &v2u,
														  PointXYZ &v3u,
														  bool rotatingPhi)
	{
		if (!_valuesInitialized)
				throw SurrogateException("getOrthonormBasisVectorCircles: Vector not yet initialized");

		if (LinearAlgebra::length(_unitOriginDir) < 0.00001)
			throw SurrogateException("getOrthoBasisVectorCircles: 0 vector right now");

		try //try using as an input basis {forceVec, forceVec rot 90 degrees}
		{
			//cout << "\nBasis A" << endl;
			double theta = -acos(_unitOriginDir.z);
			double phi = atan(_unitOriginDir.y/_unitOriginDir.x);
			PointXYZ rotatedF = PointXYZ(cos(phi)*sin(theta + M_PI/2.0), //rotate f 90 degrees
					                     sin(phi)*sin(theta + M_PI/2.0),
					                     cos(theta + M_PI/2.0));

			if (basisInitialized() && rotatingPhi)
				LinearAlgebra::getOrthornomalBasis3D_2inputs(_unitOriginDir,
						                                     _lastBasis[1], //use last v2
															 v1u, v2u, v3u);
			else if (basisInitialized() && !rotatingPhi)
				LinearAlgebra::getOrthornomalBasis3D_2inputs(_unitOriginDir,
						                                     _lastBasis[2], //use last v2
															 v1u, v3u, v2u);
			else //basis not initialized yet
				LinearAlgebra::getOrthornomalBasis3D_2inputs(_unitOriginDir,
															 rotatedF,
															 v1u, v2u, v3u);
		}
		catch(...)
		{

			cout << "\n\nBasis B" << endl;
			//try using just forceVec
			LinearAlgebra::getOrthornomalBasis3D(_unitOriginDir,
												 v1u, v2u, v3u);
		}

		v1u = LinearAlgebra::mult(_basis_multipliers[0], v1u); //this should never change sign
		v2u = LinearAlgebra::mult(_basis_multipliers[1], v2u);  //in case v2u suddenly flipped to v2u
		v3u = LinearAlgebra::mult(_basis_multipliers[2], v3u);

		//---mutation happening here: modifying _lastBasis
		//record basis if first time getting a basis
		//or if rotating theta
		_lastBasis[0] = v1u; //force vector
		_lastBasis[1] = (rotatingPhi && basisInitialized())  ? _lastBasis[1] : v2u; //keep v2 fixed if rotating using phi
		_lastBasis[2] = (!rotatingPhi && basisInitialized()) ? _lastBasis[2] : v3u; //keep v3 fixed if rotating using theta

		//cout << "\n basis: \n" <<  v1u << endl << v2u << endl << v3u << "\n" << endl;
	}


	//==========mutators
	void AdjustableVector::set(const PointXYZ &originDir, const PointXYZ &origin, double length,
						  	  const pcl::PointXYZ &offsetFromOrigin)
	{
		if (length <= 0)
			throw SurrogateException("AdjustableVector::set radius <= 0");

		_unitOriginDir = LinearAlgebra::normalize(originDir);
		_origin = origin;
		_valuesInitialized = true;
		_length = length;
		_offset = offsetFromOrigin;

		//--let's call update to display
		updateLength(0.00001);
	}

	void AdjustableVector::updateOrigin(const PointXYZ &origin)
	{
		_origin = origin;
	}


	void AdjustableVector::updateLength(double lengthInc)
	{
		if (_length + lengthInc <= max(0.0, abs(lengthInc)))
			_length = abs(lengthInc);
		else
			_length += lengthInc;

		if (!_valuesInitialized)
		{
			cout << "\n\nupdateForceVector: Vector not yet initialized.  Ignoring keystroke input until initialization"
				  << "\n\n\n" << endl;
			return;
		}
		_updateOrientationCalled = true;
	}

	void AdjustableVector::displaySelected(bool displaySelected)
	{
		_displaySelected = displaySelected;
	}


	//rotates given these angles
	void AdjustableVector::updateOrientation(double thetaPlane1_inc, double phiPlane2_inc)
	{
		if (thetaPlane1_inc != 0 && phiPlane2_inc != 0)
			throw SurrogateException("Expect only 1 non-zero angle argument");


		if (!_valuesInitialized)
		{
			cout << "\n\nupdateForceVector: Vector not yet initialized.  Ignoring keystroke input until initialization"
				  << "\n\n\n" << endl;
			return;
		}
		_updateOrientationCalled = true;

		PointXYZ v1u, v2u, v3u;
		PointXYZ origin(0,0,0);
		const double radius = 1;
		getOrthonormBasisVectorCircles(v1u, v2u, v3u,
									   phiPlane2_inc != 0); //rotating around phi

		PointXYZ proposedNewForceVec;
		if (thetaPlane1_inc != 0)
			_unitOriginDir = LinearAlgebra::getCirclePt(v1u, v2u,
														radius,
													    thetaPlane1_inc,
		      										    origin); //force vector is unit at origin
		else if (phiPlane2_inc != 0)
			_unitOriginDir = LinearAlgebra::getCirclePt(v1u,v3u,
														radius,
														phiPlane2_inc,
														origin); //force vector is unit at origin
		//make sure basis didn't change too much:
		PointXYZ new_v1, new_v2, new_v3;
		getOrthonormBasisVectorCircles(new_v1, new_v2, new_v3,
									   phiPlane2_inc != 0);
		//hack: we know v1u is force vector and changes continuously
		//just looking for discontinuous changes in v2u and v3u
		if (euclideanDistance(v2u, new_v2) > 0.5)
			_basis_multipliers[1] *= -1;
		if (euclideanDistance(v3u, new_v3) > 0.5)
			_basis_multipliers[2] *= -1;

			//cout << "\n\n******discontinuity detected*****\n" << endl;
	}

	/**Translates by the given amounts along the orthonormal basis vectors */
	void AdjustableVector::translateAlongAxis(double v1Inc, double v2Inc, double v3Inc)
	{
		//-get basis
		PointXYZ v1u, v2u, v3u;
		getOrthonormBasisVectorCircles(v1u, v2u, v3u,
								      true); //rotating around phi? just set to something

		//get increments as vectors
		Eigen::Vector3f v1Scaled = v1Inc*PclSurrogateUtils::toVec(v1u);
		Eigen::Vector3f v2Scaled = v2Inc*PclSurrogateUtils::toVec(v2u);
		Eigen::Vector3f v3Scaled = v3Inc*PclSurrogateUtils::toVec(v3u);

		//Translate
		Eigen::Vector3f newOffset = v1Scaled + v2Scaled + v3Scaled
									+ PclSurrogateUtils::toVec(_offset);
		_offset = PclSurrogateUtils::toPt(newOffset);
		_translateCalled = true;
	}

	void AdjustableVector::reset()
	{
		_length = 0;
		_origin = PointXYZ(0,0,0);
		_offset = PointXYZ(0,0,0);
		_unitOriginDir = PointXYZ(0,0,0);
		_updateOrientationCalled = false;
		_valuesInitialized = false;
		_translateCalled = false;
		_displayAxes = false;

		_basis_multipliers[0] = 1; //v1u
		_basis_multipliers[1] = 1; //v2u
		_basis_multipliers[2] = 1; //v3u

		for (int i = 0; i < 3; i++)
		{
			//set each vector to the zero vector
			_lastBasis[i].x = 0;
			_lastBasis[i].y = 0;
			_lastBasis[i].z = 0;
		}
	}

	//----------"imported"
	Eigen::Vector3f AdjustableVector::toVec(const pcl::PointXYZ &p)
	{
		return PclSurrogateUtils::toVec(p);
	}

	PointXYZ AdjustableVector::toPt(const Eigen::Vector3f &p)
	{
		return PclSurrogateUtils::toPt(p);
	}

} //namespace surrogate_gui
