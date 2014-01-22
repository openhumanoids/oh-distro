#include <iostream>
#include <eigen_utils/eigen_utils.hpp>
#include <stdlib.h>
#include <time.h>
#include "../poly.hpp"

using namespace std;
using namespace Eigen;
using namespace eigen_utils;

int main(int argc, char ** argv)
{
  srand(time(NULL));

  if (argc < 2) {
    printf("usage: %s N\n", argv[0]);
    exit(0);
  }

  int N = atoi(argv[1]);
  int N_poly = N + 1;
  Polynomial poly(N);
  poly.coeffs.setRandom();

  eigen_dump(poly);
  eigen_dump(poly.getDerivative());
  eigen_dump(poly.eval(2));
  eigen_dump(poly.eval(2,1));
  eigen_dump(poly.getDerivative().eval(2));

  VectorXd coeffs(6);
  coeffs << 1, 1, 1, 1, 1, 1;
  Polynomial poly_coeffs(coeffs);
  eigen_dump(poly_coeffs);

  MatrixXd A_derivative(N, N_poly);
  polyGetDerivativeMatrix(2, A_derivative, 1);
  eigen_dump(A_derivative);

  MatrixXd Q(N_poly, N_poly);
  VectorXd der_costs(N_poly);
  der_costs.setOnes();
  polyGetCostMatrix(2, Q, der_costs);
  eigen_dump(Q);

  VectorXd der_0(2);
  VectorXd der_final(2);
  der_costs.resize(5);

  der_0 << 0, 0;
  der_final << 1, 0;
  der_costs << 1, 0, 0, 0, 0;
  double tau = 2;
//  Polynomial poly_opt(4);
  eigen_matlab_dump(der_0);
  eigen_matlab_dump(der_final);
  eigen_matlab_dump(der_costs);
  eigen_matlab_dump(tau);
  poly = polyQuadDerOpt(2, der_0, der_final, der_costs);
  eigen_matlab_dump(poly);

  der_0 << 0, 0;
  der_final << 1, 0;
  der_costs << 1, 0, 0, 0, 0;
  VectorXd taus(2);
  VectorXd der_offsets(2);
  der_offsets << 0, 0;
  taus << .75, 0.5;
  Polynomial p0, p1;
  Polynomial * polys[2];
  polys[0] = &p0;
  polys[1] = &p1;
  int D = 2;
  eigen_matlab_dump(der_offsets);
  eigen_matlab_dump(der_0);
  eigen_matlab_dump(der_final);
  eigen_matlab_dump(der_costs);
  eigen_matlab_dump(taus);
  polyQuadDerOptPiecewise(taus, der_0, der_final, der_costs, der_offsets, polys);
  eigen_matlab_dump(p0);
  eigen_matlab_dump(p1);

}

