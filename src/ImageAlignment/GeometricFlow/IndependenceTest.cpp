/*
 * IndependenceTest.cpp
 *
 *  Created on: Sep 23, 2016
 *      Author: shane
 */

#include <math.h>

#include "IndependenceTest.hpp"

double IndependenceTest::igf(double S, double Z)
{
    if(Z < 0.0)
    {
    	return 0.0;
    }

    double Sc = (1.0 / S);
    Sc *= pow(Z, S);
    Sc *= exp(-Z);

    double Sum = 1.0;
    double Nom = 1.0;
    double Denom = 1.0;

    for(int I = 0; I < 200; I++)
    {
		Nom *= Z;
		S++;
		Denom *= S;
		Sum += (Nom / Denom);
    }

    return Sum * Sc;
}

double IndependenceTest::p_value(double critical_value, int dof){
	if(critical_value < 0 || dof < 1)
	{
		return 0.0;
	}
	double K = ((double)dof) * 0.5;
	double X = critical_value * 0.5;
	if(dof == 2)
	{
		return exp(-1.0 * X);
	}

	double PValue = igf(K, X);
	if(isnan(PValue) || isinf(PValue) || PValue <= 1e-8)
	{
		return 1e-14;
	}

	PValue /= tgamma(K);

	return (1.0 - PValue);
}


