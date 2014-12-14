/*
 * LinearAlgebra.h
 *
 *  Created on: Feb 22, 2012
 *      Author: mfleder
 */

#ifndef LINEARALGEBRA_H_
#define LINEARALGEBRA_H_

#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

namespace surrogate_gui
{

/**non-instantiable*/
	class LinearAlgebra
	{
		public:
		static void getOrthornomalBasis3D(const pcl::PointXYZ &v1, //the only input
										  pcl::PointXYZ &v1_out,  //v1/||v1||
										  pcl::PointXYZ &v2_out,  //some vector orthogonal to v1
										  pcl::PointXYZ &v3_out); //vector orthogonal to v2

		static void getOrthornomalBasis3D_2inputs(const pcl::PointXYZ &v1, //first input
												  const pcl::PointXYZ &v2,  //2nd input
												  pcl::PointXYZ &v1_out,   //v1 / ||v1||
												  pcl::PointXYZ &v2_out,  //gram-scmidt on v1,v2
												  pcl::PointXYZ &v3_out); //v1_out x v2_out

		static pcl::PointXYZ runGramSchmidt(const pcl::PointXYZ &v1, //1st basis vec
										    const pcl::PointXYZ &v2); //will be made orthogonal to v1

		static pcl::PointXYZ normalize(const pcl::PointXYZ &v); //v / ||v||
		static double dot(const pcl::PointXYZ &v1, const pcl::PointXYZ &v2);  //dot product
		static double length(const pcl::PointXYZ &p);

		static bool isUnit(const pcl::PointXYZ &p);
		static bool isOrthoNormBasis(const std::vector<pcl::PointXYZ> &basis);

		static pcl::PointXYZ sub(const pcl::PointXYZ &a, const pcl::PointXYZ &b); //a - b
		static pcl::PointXYZ add(const pcl::PointXYZ &a, const pcl::PointXYZ &b); //a + b
		static pcl::PointXYZ mult(const double lamda, const pcl::PointXYZ &b); //lambda*b


		static pcl::PointXYZ projectOntoUnitVec(const pcl::PointXYZ &unitVec,
												const pcl::PointXYZ &vecToProj);
		static pcl::PointXYZ crossProduct(const pcl::PointXYZ &p1,
										  const pcl::PointXYZ &p2);

		static pcl::PointXYZ getCirclePt(const pcl::PointXYZ &planeVec1, //corresponds to theta == 0
										 const pcl::PointXYZ &planeVec2,
										 const double radius,
										 const double theta,
										 const pcl::PointXYZ &center);

		//---------fitting
		/*
		static bool lineFit(const std::list<pcl::PointXYZ> pointsToFit,
				           pcl::PointXYZ &lineStart,
				           pcl::PointXYZ &lineEnd);
		 */
		static double pointToLineDist(const Eigen::Vector3f &a_line, const Eigen::Vector3f &b_line,
									  const Eigen::Vector3f &p);

		static double projectPointToLine(const Eigen::Vector3f &a_line, const Eigen::Vector3f &b_line,
										 const Eigen::Vector3f &p, Eigen::Vector3f &projection);



		//--------run tests
		static void runTests();

		//--non-instantiable
		private:
			LinearAlgebra();
			virtual ~LinearAlgebra();
	};
} //namespace
#endif /* LINEARALGEBRA_H_ */
