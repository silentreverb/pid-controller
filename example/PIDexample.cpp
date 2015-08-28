#include <PIDController.h>
#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
	PIDController* pid = new PIDController(1,0,0,-100,100);

    pid->on();
    pid->targetSetpoint(100);
    pid->off();

    cout << "Hello World!" << endl;
	
	return 0;
}
