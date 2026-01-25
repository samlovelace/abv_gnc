#include <fstream>
#include <string>
#include <unistd.h>
#include <jetgpio.h>
#include <iostream>

using namespace std;

int main() {

  	gpioInitialise();
  	gpioSetMode(8, JET_OUTPUT);
  	gpioSetMode(10, JET_OUTPUT);
  	gpioSetMode(12, JET_OUTPUT);
  	gpioSetMode(22, JET_OUTPUT);
  	
  	cout << "Set pin " << 8 << " to HIGH." << endl;
  	gpioWrite(8, 1);
    	sleep(2);
    	cout << "Set pin " << 8 << " to LOW." << endl;
	gpioWrite(8, 0);
	sleep(2);

	cout << "Set pin " << 10 << " to HIGH." << endl;
  	gpioWrite(10, 1);
    	sleep(2);
    	cout << "Set pin " << 10 << " to LOW." << endl;
	gpioWrite(10, 0);
	sleep(2);
	
	cout << "Set pin " << 12 << " to HIGH." << endl;
  	gpioWrite(12, 1);
    	sleep(2);
    	cout << "Set pin " << 12 << " to LOW." << endl;
	gpioWrite(12, 0);
	sleep(2);
	
	cout << "Set pin " << 22 << " to HIGH." << endl;
  	gpioWrite(22, 1);
    	sleep(2);
    	cout << "Set pin " << 22 << " to LOW." << endl;
	gpioWrite(22, 0);
	sleep(2);

	gpioTerminate();
	return 0;
}



