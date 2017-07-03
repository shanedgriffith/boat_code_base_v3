/*
 * IndependenceTest.hpp
 *
 *  Created on: Sep 23, 2016
 *      Author: shane
 */

#ifndef SRC_GEOMETRICFLOW_INDEPENDENCETEST_HPP_
#define SRC_GEOMETRICFLOW_INDEPENDENCETEST_HPP_


class IndependenceTest{
private:
	static double igf(double S, double Z);
public:
	static double p_value(double critical_value, int dof);
};


#endif /* SRC_GEOMETRICFLOW_INDEPENDENCETEST_HPP_ */
