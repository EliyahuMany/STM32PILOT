#ifndef __PIDCONTROLLER_H__
#define __PIDCONTROLLER_H__

#include "stm32f10x.h"

class PIDController
{
	public:
		PIDController(double dt, double Kp, double Ki, double Kd);
		void calculate(double pval, double setpoint);
		double getError();
		double getPrevError();
		double getPidResult();
		double constrain(int min, int max, int minSaturate, int maxSaturate);
		void setKp(float val);
		void setKi(float val);
		void setKd(float val);
		double getKp();
		double getKi();
		double getKd();
	
	private:
		double dt;
		double Kp;
		double Ki;
		double Kd;
	
		double pout;
		double iout;
		double dout;
		double pidResult;
	
		double integral;
		double derivative;
		double error; // for debugging use
		double previousError;
	
		
};

#endif
