#include <PIDController.h>
#include <complex>
#include <iostream>

using namespace std;

#define K_P 11.5 // Proportional gain
#define K_I 19.0 // Integral gain
#define K_D 0.0  // Derivative gain

// Calculate inverse laplace transform for the closed-loop tranfer function
double invLaplace(double sp, double t) {
    double sigma = 1;
    int T = 300;

    complex<double> integral = 0;

    for(int s_imag = -T; s_imag < T; s_imag++) {
        complex<double> s(sigma,s_imag);

        complex<double> pid = K_P + K_I/s + K_D*s;
        complex<double> plant = complex<double>(1)/(pow(s,2) + complex<double>(5)*s + complex<double>(6));
        complex<double> xferFn = complex<double>(sp)*(complex<double>(1)/s)*(pid * plant)/(complex<double>(1) + pid*plant*complex<double>(1));
    
        integral += exp(s*t)*xferFn*complex<double>(1);
    }

    return double(abs(complex<double>(1)/complex<double>(0,2*M_PI)*integral));
}

int main(int argc, char *argv[])
{
	PIDController* pid = new PIDController(K_P,K_I,K_D);

    pid->on(); // Turn PID controller on
    pid->targetSetpoint(10); // Change desired setpoint to 10
    
    double t = 0; // Init with time t=0
    double T = 0.01; // Period
    double controlVariable = 0; // Init with zero actuation
    double processVariable = 0; // Init with zero position
    
    cout << "Time,Setpoint,Output" << endl;

    while(1) {
        cout << t << "," << pid->getSetpoint() << "," << processVariable << endl;
        controlVariable = pid->calc(processVariable); // Calculate next controlVariable
        processVariable = invLaplace(pid->getSetpoint(), t); // Simulate step response of plant with TF 1/(s^2+5*s+6)

        usleep(T*pow(10,6)); // 100ms delay to simulate actuation time
        t += T; // Increment time variable by 100ms
    }
	
	return 0;
}
