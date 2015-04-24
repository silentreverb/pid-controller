/* 
 * File:   PIDController.cpp
 * Author: User
 * 
 * Created on April 22, 2014, 6:28 PM
 */

#include "PIDController.h"

PIDController::PIDController(double kp, double ki, double kd) {
    this->setGains(kp, ki, kd);
    this->setConstraints(-1, -1);
    this->init();
}

PIDController::PIDController(double newKp, double newKi, double newKd, double lowerConstraint, double upperConstraint) {
    this->setGains(kp, ki, kd);
    this->setConstraints(lowerConstraint, upperConstraint);
    this->init();
}

PIDController::PIDController() {
    this->setGains(0, 0, 0);
    this->setConstraints(-100,100);
    this->init();
}

PIDController::PIDController(const PIDController& orig) {
    this->setGains(orig.kp, orig.ki, orig.kd);
    this->setConstraints(orig.lowerConstraint, orig.upperConstraint);
    integrator = orig.integrator;
    lastError = orig.lastError;
}

PIDController::~PIDController() {
}
        
void PIDController::targetSetpoint(double setpoint) {
    this->setpoint = setpoint;
}

void PIDController::setGains(double kp, double ki, double kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PIDController::setConstraints(double lowerConstraint, double upperConstraint) {
    this->lowerConstraint = lowerConstraint;
    this->upperConstraint = upperConstraint;
}

double PIDController::getSetpoint() {
    return setpoint;
}

double PIDController::getKp() {
    return kp;
}

double PIDController::getKi() {
    return ki;
}

double PIDController::getKd() {
    return kd;
}

void PIDController::init() {
    setpoint = 0;
    integrator = 0;
    lastError = 0;
	lastProcessVariable = -123456;
    sample_timer.stop();
	performance_timer.stop();   
}

bool PIDController::isSettled() {

	double percent = 1-(lastProcessVariable/setpoint);

	std::cout<<"Percent: "<<percent<<std::endl;	
	
	if(abs(percent) < 0.05)
	{
		return true;
	}
	else
	{
		return false;
	}
}

double PIDController::calc(double processVariable) {
    sample_timer.stop();
	double error = setpoint - processVariable;
    double samplingTime = boost::chrono::duration_cast<boost::chrono::seconds>(boost::chrono::nanoseconds(sample_timer.elapsed().wall)).count();
    double differentiator = (error - lastError)/samplingTime;
    integrator += (error * samplingTime);
    double controlVariable = kp * error + ki * integrator + kd * differentiator;
    if(controlVariable < lowerConstraint) {
        controlVariable = lowerConstraint;
    }
    else if(controlVariable > upperConstraint) {
        controlVariable = upperConstraint;
    }
    lastError = error;
	lastProcessVariable = processVariable;

	sample_timer.start();

    return controlVariable;
}

