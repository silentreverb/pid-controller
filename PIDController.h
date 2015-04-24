/* 
 * File:   PIDController.h
 * Author: User
 *
 * Created on April 22, 2014, 6:28 PM
 */

#ifndef PIDCONTROLLER_H
#define	PIDCONTROLLER_H

#include <boost/timer/timer.hpp>
#include <boost/chrono.hpp>
#include <iostream>
#include <cmath>

class PIDController {
    public:
        PIDController();
        PIDController(const PIDController& orig);
        PIDController(double newKp, double newKi, double newKd);
        PIDController(double newKp, double newKi, double newKd, double lowerConstraint, double upperConstraint);
        virtual ~PIDController();
        
        void targetSetpoint(double setpoint);
        void setGains(double kp, double ki, double kd);
        void setConstraints(double lowerConstraint, double upperConstraint);
        
        double getSetpoint();
        double getKp();
        double getKi();
        double getKd();

        void init();
        bool isSettled();
        double calc(double feedback);

        
    private:
        double setpoint; 
        double kp, ki, kd;
        double lowerConstraint, upperConstraint;
        boost::timer::cpu_timer sample_timer;
		boost::timer::cpu_timer performance_timer;
		double lastError;
		double lastProcessVariable;
        double integrator;
};

#endif	/* PIDCONTROLLER_H */

