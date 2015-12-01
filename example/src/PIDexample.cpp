#include <PIDController.h>
#include <LaplaceInversion.h>
#include <iostream>
#include <fstream>

using namespace std;

#define SETPOINT 10.0 // Desired setpoint

#define PLANT_A 2.0 // Poles of plant transfer function
#define PLANT_B 3.0 // Poles of plant transfer function

#define K_P 11.5 // Proportional gain
#define K_I 19.0 // Integral gain
#define K_D 0.0  // Derivative gain


// Define closed-loop TF for simulation
cmplex xferFn(const cmplex &s) {
    cmplex pid = K_P + K_I/s + K_D*s;
    cmplex plant = 1.0/((s+PLANT_A)*(s+PLANT_B));
    
    return SETPOINT*(1.0/s)*(pid*plant)/(1.0+pid*plant*1.0);
}

int main(int argc, char *argv[])
{
	PIDController* pid = new PIDController(K_P,K_I,K_D);

    pid->on(); // Turn PID controller on
    pid->targetSetpoint(SETPOINT); // Change desired setpoint to SETPOINT
    
    double t = 0; // Init with time t=0
    double T = 0.01; // Period

    double controlVariable = 0; // Init with zero actuation
    double processVariable = 0; // Init with zero position
   
    cout << "Simulation running..." << endl;
    
    ofstream writeToCsv;
    writeToCsv.open("PIDexample.csv");
    writeToCsv << "Time,Setpoint,Output" << endl;

    while(t < 20) {
        writeToCsv << t << "," << pid->getSetpoint() << "," << processVariable << endl;
        
        controlVariable = pid->calc(processVariable); // Calculate next controlVariable
        processVariable = LaplaceInversion(xferFn, t, 1e-8); // Simulate plant with TF 1/((s+a)(s+b))

        usleep(T*pow(10,6)); // 100ms delay to simulate actuation time
        t += T; // Increment time variable by 100ms
    }

	writeToCsv.close();
    cout << "Simulation complete! Output saved to PIDexample.csv." << endl;
	
    return 0;
}
