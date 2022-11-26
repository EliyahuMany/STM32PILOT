#include "PIDController.h"

PIDController::PIDController(double dt, double Kp, double Ki, double Kd)
{
	this->dt = dt;
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	this->previousError = 0;
	this->previousValue = 0;
}

void PIDController::calculate(double pval, double setpoint)
{
	double error;
	// calculate error
	error = pval - setpoint;
	this->error = error;
	
	// proportinal calc
	double pout;
	pout = this->Kp * error;
	this->pout = pout;
	
	// integral calc
	double iout;
	iout = this->Ki * error * this->dt;
	this->iout = iout;
	
	// derivative calc
	double dout;
	dout = this->Kd * (error - this->previousError) / dt;
	this->dout = dout;
	
	this->previousError = error;
	
	this->pidResult = pout + iout + dout;
}


void PIDController::calculateIdeal(double pval, double setpoint)
{
	double error;
	// calculate error
	error = pval - setpoint;
	this->error = error;
	
	// proportinal calc
	double pout;
	pout = this->Kp * error;
	this->pout = pout;
	
	// integral calc
	double iout;
	iout = this->Ki * error * this->dt;
	this->iout = iout;
	
	// derivative calc
	double dout;
	dout = this->Kd * (pval - this->previousValue) / dt;
	this->dout = dout;
	
	this->previousError = error;
	this->previousValue = pval;
	
	this->pidResult = pout + iout + dout;
}


void PIDController::constrain(double min, double max, double minSaturate, double maxSaturate)
{
	if (this->pidResult < min)
		this->pidResult = min;
	else if (this->pidResult > max)
		this->pidResult = max;
	if(this->pidResult < minSaturate && this->pidResult > maxSaturate)
		this->pidResult = 0;
}

double PIDController::getError()
{
	return this->error;
}

double PIDController::getPrevError()
{
	return this->previousError;
}

double PIDController::getPidResult()
{
	return this->pidResult;
}

double PIDController::getKp()
{
	return this->Kp;
}
void PIDController::setKp(float val)
{
	this->Kp = val;
}

double PIDController::getKi()
{
	return this->Ki;
}
void PIDController::setKi(float val)
{
	this->Ki = val;
}

double PIDController::getKd()
{
	return this->Kd;
}

void PIDController::setKd(float val)
{
	this->Kd = val;
}

void PIDController::resetError()
{
	this->previousError = 0;
	this->previousValue = 0;
}
