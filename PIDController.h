/* 
 * File:   PIDController.h
 * Author: User
 *
 * Created on April 22, 2014, 6:28 PM
 */

#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <boost/timer/timer.hpp>
#include <boost/chrono.hpp>
#include <iostream>
#include <cmath>

class PIDController {
    public:
        PIDController();
        PIDController(double kp, double ki, double kd);
        PIDController(double kp, double ki, double kd, double lowerOutputLimit, double upperOutputLimit);
        PIDController(double newKp, double newKi, double newKd, double lowerInputLimit, double upperInputLimit, double lowerOutputLimit, double upperOutputLimit);
        PIDController(const PIDController& orig);
        virtual ~PIDController();
        
        void targetSetpoint(double setpoint);
        void setGains(double kp, double ki, double kd);
        void off();
        void on();
        void setInputLimits(double lowerLimit, double upperLimit);
        void setOutputLimits(double lowerLimit, double upperLimit);
        double getSetpoint();
        double getKp();
        double getKi();
        double getKd();

        void reset();
        bool hasSettled();
        double calc(double feedback);

        
    private:
        bool isEnabled;
        bool hasSettled;
        double setpoint; 
        double lastSetpoint;
        double lastControlVariable;
        double lastProcessVariable;
        double kp, ki, kd;
        double lowerInputLimit, upperInputLimit;
        double lowerOutputLimit, upperOutputLimit;
        boost::timer::cpu_timer sample_timer;
        double integrator;
        
        double limiter(double value, double lowerLimit, double upperLimit);
};

#endif  /* PIDCONTROLLER_H */

