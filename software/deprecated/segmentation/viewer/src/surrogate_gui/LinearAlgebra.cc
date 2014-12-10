/*
 * LinearAlgebra.cpp
 *
 *  Created on: Feb 22, 2012
 *      Author: mfleder
 */

#include "LinearAlgebra.h"
#include "SurrogateException.h"

//--for eigen
//#include <Eigen/Core>
//#include "eigen3/Eigen/src/Eigen2Support/LeastSquares.h"


using namespace std;
using namespace pcl;

namespace surrogate_gui
{
//----------some linear algebra stuff involving PCL maybe
	/**Get an orthonormal basis for R3 with the only constrant that 1 basis
	 * vector be v1 (normalized).
	 * @param v1 the only input vector
	 * @param v1_out v1 / ||v1||
	 * @param v2_out some vector orthogonal to v1
	 * @param v3_out some vector orthogonal to v1 & v2*/
	void LinearAlgebra::getOrthornomalBasis3D(const PointXYZ &v1, //the only input
											  PointXYZ &v1u,  //v1/||v1||
											  PointXYZ &v2u,  //some unit vector orthogonal to v1
											  PointXYZ &v3u) //unit vector orthogonal to v2
	{
		const double MIN_LENGTH = 0.00001; //minimum vector length
		if (length(v1) < MIN_LENGTH)
		{
			cout << "\nInput to getOrthonormalBasis3D: " << v1 << endl;
			throw SurrogateException("getOrthoBasis3D: vector input too small");
		}

		v1u = normalize(v1);

		//--------------v2u
		//try Gram-Schmidt with v1u and e1 = (1,0,0)
		//v2u = e1 - <v1u, e1>*v1u (not equation changes if not using unit vectors)
		v2u = runGramSchmidt(v1u, PointXYZ(1,0,0));
		if (length(v2u) < MIN_LENGTH) //too small? run Gram-Schmidt with v1u and e2 = (0,1,0)
			v2u = runGramSchmidt(v1u, PointXYZ(0,1,0));

		if (length(v2u) < MIN_LENGTH)
			throw SurrogateException("How were the projections onto both e1 AND e2 both too small?");
		v2u = normalize(v2u);

		//----------------v3u
		//v3u = v1u x v2u
		v3u = crossProduct(v1u, v2u);
		v3u = normalize(v3u);
	}

	/**Get an orthonormal basis for R3 with the only constrant that 1 basis
		 * vector be v1 (normalized) and the other be derived from {v1,v2}.
		 * @param v1 1st input vector
		 * @param v2 2nd input vector
		 * @param v1_out v1 / ||v1||
		 * @param v2_out some vector orthogonal to v1
		 * @param v3_out some vector orthogonal to v1 & v2*/
	void LinearAlgebra::getOrthornomalBasis3D_2inputs(const PointXYZ &v1, //first input
													  const PointXYZ &v2,  //2nd input
													  PointXYZ &v1u,   //v1 / ||v1||
													  pcl::PointXYZ &v2u,  //gram-scmidt on v1,v2
													  pcl::PointXYZ &v3u) //v1_out x v2_out
	{
		const double MIN_LENGTH = 0.00001; //minimum vector length
		if (length(v1) < MIN_LENGTH || length(v2) < MIN_LENGTH)
		{
			cout << "\nInput to getOrthornomalBasis3D_2inputs {v1,v2}: " << v1 << endl << v2 << endl;
			throw SurrogateException("getOrthornomalBasis3D_2inputs {v1,v2}: vector input too small");
		}

		v1u = normalize(v1);

		//--------------v2u
		//Gram-Schmidt with v1u and v2
		v2u = runGramSchmidt(v1u, v2);
		if (length(v2u) < MIN_LENGTH)
				throw SurrogateException ("getOrthornomalBasis3D_2inputs: "
						                   " Grahm-Schmidt vector too small: {v1, v2 - <v1,v2>*v1");
		v2u = normalize(v2u);

		//----------------v3u
		//v3u = v1u x v2u
		v3u = crossProduct(v1u, v2u);
		v3u = normalize(v3u);

		//-------defensive checks
		vector<PointXYZ> basis;
		basis.push_back(v1u);
		basis.push_back(v2u);
		basis.push_back(v3u);

		if (!isOrthoNormBasis(basis))
			throw SurrogateException("Was going to return non-unit vectors: getOrthoBasis3d_2inputs");
	}

	/*returns a vector that is orthogonal to a. might be the 0 vector  **/
	PointXYZ LinearAlgebra::runGramSchmidt(const PointXYZ &a, //first input : will be 1st basis vec
										   const PointXYZ &b)
	{
		PointXYZ bProjOntoA_unit = projectOntoUnitVec(normalize(a), b);
		return sub(b, bProjOntoA_unit); //b - <b, a_unit> * a_unit
	}

	/**Projects vecToProj onto the space spanned by spanVector
	 * @require that ||spanVector|| == 1*/
	PointXYZ LinearAlgebra::projectOntoUnitVec(const PointXYZ &unitVec, const PointXYZ &vecToProj)
	{
		if (!isUnit(unitVec))
			throw SurrogateException("unitVector does not have length 1");

		double innerProduct = dot(unitVec, vecToProj);

		return PointXYZ(unitVec.x*innerProduct,
						unitVec.y*innerProduct,
						unitVec.z*innerProduct);
	}

	double LinearAlgebra::length(const pcl::PointXYZ &p)
	{
		return sqrt(dot(p,p));
	}

	/**@return a - b*/
	PointXYZ LinearAlgebra::sub(const PointXYZ &a, const PointXYZ &b)
	{
		return PointXYZ(a.x-b.x,
				        a.y-b.y,
				        a.z-b.z);
	}

	/**@return a + b*/
	PointXYZ LinearAlgebra::add(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
	{
		return PointXYZ(a.x + b.x,
						a.y + b.y,
						a.z + b.z);
	}

	/**@return lambda*b */
	PointXYZ LinearAlgebra::mult(const double lambda, const pcl::PointXYZ &b)
	{
		return PointXYZ(b.x*lambda,
						b.y*lambda,
						b.z*lambda);
	}

	PointXYZ LinearAlgebra::crossProduct(const PointXYZ &p1, const PointXYZ &p2)
	{
		return PointXYZ( p1.y * p2.z  -  p1.z * p2.y,
						-p1.x * p2.z  +  p1.z * p2.x,
						 p1.x * p2.y  -  p1.y * p2.x);
	}

	/**@return v / ||v||*/
	PointXYZ LinearAlgebra::normalize(const PointXYZ &v)
	{
		double lengthV = length(v);
		if (lengthV < 0.0000001)
			throw SurrogateException("Warning: very small vector");

		return PointXYZ(v.x/lengthV, v.y/lengthV, v.z/lengthV);
	}

	/**@return ||v|| == 1*/
	bool LinearAlgebra::isUnit(const pcl::PointXYZ &p)
	{
		return abs(1 - length(p) < 0.0001);
	}

	/**@return true if all the vectors in basis are orthonormal */
	bool LinearAlgebra::isOrthoNormBasis(const vector<PointXYZ> &basis)
	{
		for (uint i = 0; i < basis.size(); i++)
		{
			if (!isUnit(basis[i]))
				return false;
			for (uint j = 0; j < basis.size(); j++)
			{
				if (i == j)
					continue;
				if (abs(dot(basis[i], basis[j])) > 0.0001)
					return false;
			}
		}
		return true;
	}

	double LinearAlgebra::dot(const PointXYZ &a, const PointXYZ &b)
	{
		return a.x*b.x + a.y*b.y + a.z*b.z;
	}


	/**Returns the point on the circle at angle theta.
	 * The circle lies in the plane of the orthonormal vectors {planeVec1, planeVec2},
	 * has radius r, and theta == 0 corresponds to the point on the circle that lies on the
	 * line planeVec1 * constant
	 * @param planeVec1 unit vector in plane of circle
	 * @param planeVec2 unit vector in plane of circle orthogonal to planeVec1
	 * @param radius radius of circle
	 * @param theta : angle between planeVec1 and the target point*/
	PointXYZ LinearAlgebra::getCirclePt(const PointXYZ &planeVec1, //corresponds to theta == 0
										const PointXYZ &planeVec2,
										const double radius,
										const double theta,
										const PointXYZ &center)
	{
		//=====defensive checks
		//---make sure planeVec1 and planeVec2 are orthonormal
		if (!isUnit(planeVec1) || !isUnit(planeVec2))
		{
			cout << "\ngetCirclePt Error: vec1 = \n" << planeVec1 << "\nvec2 = \n" << planeVec2 << endl;
			throw SurrogateException("Non-unit vectors passed to getCirclePt");
		}
		if (abs(dot(planeVec1, planeVec2)) > 0.0001)
		{
			cout << "\n\n\n\nplaneVec1 = " << planeVec1 << endl;
			cout << "\nplaneVec2 = " << planeVec2 << "\n\n\n" << endl;

			throw SurrogateException("non-orthogonal vectors passed to getCirclePt");
		}
		//should make sure center is in the plane:
		/*PointXYZ normal = crossProduct(planeVec1, planeVec2);
		PointXYZ unitTowardCenter = normalize(center);
		PointXYZ shouldBeInPlaneVec = sub(planeVec1, unitTowardCenter);
		if (abs(dot(normal, shouldBeInPlaneVec)) > 0.001)
		{
			cout << "\n\ndot product of normal + plane vec = "
					<< dot(normal, shouldBeInPlaneVec) << endl << endl;
			throw SurrogateException("Center of circle not in the plane");
		}*/
		//
		if (radius <= 0)
		{
			cout << "\n radius = " << radius << endl;
			throw SurrogateException("non-positive radius passed to getCirclePt");
		}
		//---finished defensive checks


		//equation of circle is:
		//pointXYZ(theta) = center + r*cos(theta)*planeVec1 + r*sin(theta)*planeVec2
		PointXYZ v1Part = mult(radius*cos(theta), planeVec1);
		PointXYZ v2Part = mult(radius*sin(theta), planeVec2);
		return add(center, add(v1Part, v2Part));
	}


	//----------
	/**@return true if this is a good line fit. false otherwise
		 * @param lineStart start pt of line that is fit (output)
		 * @param lineEnd end pt of line that is fit (output)*/
	/*
		bool LinearAlgebra::planeFit(const list<PointXYZ> pointsToFit,
									     PointXYZ &coeffs)
		{
			if (pointsToFit.size() < 2)
				return false;  //can't fit a line w/ < 2 points

			//create a matrix [x's y's 1's]
			//and a vector of [z's]
			MatrixXd data;
			data.setZero(pointsToFit.size(),3); //rows, columns
			MatrixXd zs;
			zs.setZero(pointsToFit.size(), 1); //rows, columns

			int nextRowInd = 0;
			for(list<PointXYZ>::const_iterator iter = pointsToFit.begin();
			    iter!= pointsToFit.end(); iter++)
			{
				PointXYZ nextPt = *iter;
				data(nextRowInd,0) = nextPt.x; //(row, col)
				data(nextRowInd,1) = nextPt.y;
				data(nextRowInd,2) = 1;

				zs(nextRowInd,0) = nextPt.z;

				nextRowInd++;
			}

			//======solve
			MatrixXd coeffs = (data.transpose()*data).inverse() * data.transpose()*zs;

			if (coeffs.rows() != 3 && coeffs.cols() != 1)
				throw SurrogateException("How did this coeff matrix get the wrong size?");

			//coeffs = (a b c)' of the form: ax + by + c = z
			coeffs.x = a;
			coeffs.y = b;
			coeffs.z = c;

			return true;
		}
		*/

	/**@param line1 first point on line
	 * @param line2 2nd point on line
	 * @param p point to examine distance from the line defined by line1,line2
	 * @return distance form p to line(t) = line1 + t*(line2 - line1)*/
	double LinearAlgebra::pointToLineDist(const Eigen::Vector3f &line1, const Eigen::Vector3f &line2,
										  const Eigen::Vector3f &p)
	{
		if (line1 == line2)
			throw SurrogateException("Invalid line: both points are equal");

		//http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
		Eigen::Vector3f minus21 = line2 - line1;
		return (minus21.cross(line1 - p)).norm() / minus21.norm();
	}

	/**@param line1 first point on line
	 * @param line2 2nd point on line
	 * @param p point to project onto Line(t) = line1 + t*(line2-line1)
	 * @param projection project of p onto Line(t) as above
	 * @return the t value as in the equation of the line above*/
	double LinearAlgebra::projectPointToLine(const Eigen::Vector3f &line1, const Eigen::Vector3f &line2,
											 const Eigen::Vector3f &p, Eigen::Vector3f &projection)
	{
		//http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
		Eigen::Vector3f min21 = line2-line1;
		double t = (line1 - p).dot(min21)/min21.squaredNorm();
		projection = line1 + t*min21;
		return t;
	}


	//------------run tests
	void LinearAlgebra::runTests()
	{
		PointXYZ inputVec(1,0,0);
		PointXYZ inputV2(1,1,1);
		PointXYZ v1u, v2u, v3u;
		//getOrthornomalBasis3D(inputVec, v1u, v2u, v3u);
		getOrthornomalBasis3D_2inputs(inputVec, inputV2,
									  v1u, v2u, v3u);


		cout << "\n Got basis: \n"
			  << v1u << endl
			  << v2u << endl
			  << v3u << endl
			  << endl;

		cout << "dot products: \n"
			 << dot(v1u, v2u) << endl
			 << dot(v1u, v3u) << endl
			 << dot(v2u, v3u) << endl;

		cout << "lengths: \n"
			 << length(v1u) << endl
			 << length(v2u) << endl
			 << length(v3u) << endl;

	}

	//-----------
	LinearAlgebra::LinearAlgebra()
	{
		throw SurrogateException("not implemented");
	}

	LinearAlgebra::~LinearAlgebra()
	{
		throw SurrogateException("not implemented");
	}
}
