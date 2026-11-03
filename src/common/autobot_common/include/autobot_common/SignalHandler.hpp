
#include "plog/Log.h"

// Signal handler function
void signalHandler(int signal) 
{

    LOGD  << "\n" << "\t\t"
	  R"(________________________
		|                       |
		|   SHUTTING DOWN...    |
		|_______________________|
               __   /
              / o) /
     _.----._/ /
    /         /
 __/ (  | (  |
/__.-'|_|--|_|
)";
    exit(0); // Exit the program
}