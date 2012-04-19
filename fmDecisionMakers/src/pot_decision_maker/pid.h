/*
 * pid.h
 *
 *  Created on: Mar 19, 2012
 *      Author: Morten Ege Jensen
 */

#ifndef PID_H_
#define PID_H_

struct PID{
	double P, I, D;
	double old_value, ivalue;
	void operator()(double np, double ni, double nd){
		P = np;
		I = ni;
		D = nd;
		old_value = 0;
		ivalue = 0;
	}

	double run(double value, double dt){
		double dvalue = (value - old_value)/dt;
		old_value = value;
		ivalue += value*dt;
		return P*value + I*ivalue + D*dvalue;
	}
};


#endif /* PID_H_ */
