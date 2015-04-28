/* 
 * File:   PIDController.cpp
 * Author: User
 * 
 * Created on April 22, 2014, 6:28 PM
 */

#include "PIDController.h"

using namespace boost::chrono;

PIDController::PIDController(double kp, double ki, double kd) {
    this->setGains(kp, ki, kd);
    this->setConstraints(-1, -1);
    this->init();
}

PIDController::PIDController(double kp, double ki, double kd, double lowerConstraint, double upperConstraint) {
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
    peakTime = -1;
	settlingTime = -1;
	percentOvershoot = 0;
    sample_timer.start();
    performance_timer.start();
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
	peakTime = -1;
	settlingTime = -1;
	percentOvershoot = 0;
    sample_timer.stop();
	performance_timer.stop();   
}

bool PIDController::hasSettled() {
	if(settlingTime != -1) {
		return true;
	}
	
	else {
		return false;
	}
}

double PIDController::calc(double processVariable) {
    sample_timer.stop();
	
	double percent = (processVariable/setpoint) - 1;
	
	if(percent > percentOvershoot && percent > 0)
	{
		percentOvershoot = processVariable/setpoint;
		peakTime = (performance_timer.elapsed().wall)/1e9;
	}
	if(abs(percent) < 0.05 && settlingTime == -1)
	{
		performance_timer.stop();
		settlingTime = (performance_timer.elapsed().wall)/1e9;
		std::cout << "Peak Time Tp: " << peakTime << std::endl;
		std::cout << "Percent Overshoot %OS: " << percentOvershoot << std::endl;
		std::cout << "Settling Time Ts" << settlingTime << std::endl;
	}
	
	double error = setpoint - processVariable;
    double samplingTime = (sample_timer.elapsed().wall)/1e9;
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

	sample_timer.start();

    return controlVariable;
}

