__author__ = 'manuelli'
import drake


# Utilities for encoding piecewise polynomials
def encodePolynomialVector(coefficientMatrix):
    msg = drake.lcmt_polynomial_matrix()
    msg.rows = coefficientMatrix.shape[0]
    msg.cols = 1
    numCoefficients = coefficientMatrix.shape[1]

    polynomialMsgList = []

    for rowIdx in xrange(0,msg.rows):
        polynomialMsg = drake.lcmt_polynomial()
        polynomialMsg.num_coefficients = numCoefficients
        polynomialMsg.coefficients = coefficientMatrix[rowIdx,:]
        polynomialMsgList.append([polynomialMsg])

    msg.polynomials = polynomialMsgList

    return msg