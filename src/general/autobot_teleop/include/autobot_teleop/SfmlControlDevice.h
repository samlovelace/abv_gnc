#ifndef SFMLCONTROLDEVICE_H
#define SFMLCONTROLDEVICE_H

#include "IControlDevice.h"
#include <SFML/Graphics.hpp>
 
class SfmlControlDevice : public IControlDevice
{ 
public:
    SfmlControlDevice();
    ~SfmlControlDevice();

    bool handleInput() override; 

private:

    sf::RenderWindow mWindow;
    void appendCommand(Eigen::VectorXd anUpdateCmd); 
    sf::Vector3i getThrustDirection(int joystickId); 
    int snapAxis(float value, float threshold = 0.5f);
    bool mFirstConnected; 

};
#endif //SFMLCONTROLDEVICE_H