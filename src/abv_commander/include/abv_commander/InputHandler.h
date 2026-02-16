#ifndef INPUTHANDLER_H
#define INPUTHANDLER_H

#include "abv_commander/Colors.hpp"
#include <string> 
 
class InputHandler 
{ 
public:
    InputHandler();
    ~InputHandler();

    void handle(const std::string& anInput); 

private:

    std::string mPackagePath; 
   
};
#endif //INPUTHANDLER_H