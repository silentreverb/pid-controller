#include <PIDController.h>
#include <cstdlib>
#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
	PIDController* pid = new PIDController(5,0.5,0.25,-100,100);

    pid->on(); // Turn PID controller on
    pid->targetSetpoint(10); // Change desired setpoint to 10
    
    double t = 0; // Init with time t=0
    double controlVariable = 0; // Init with zero actuation
    double processVariable = 0; // Init with zero position
    
    while(1) {
        controlVariable = pid->calc(processVariable); // Calculate next controlVariable
        processVariable += controlVariable/(20 + ((rand() % 11) - 5)); // Arbitrary function to simulate a plant
        
        cout << "Time: " << t << ", Setpoint: " << pid->getSetpoint() << ", PV: " << processVariable << endl;
        
        usleep(100000); // 100ms delay to simulate actuation time
        t += 0.1; // Increment time variable by 100ms
    }
	
	return 0;
}
