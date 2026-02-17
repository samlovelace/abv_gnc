
#include "SfmlControlDevice.h"
#include <cstdio>
#include <iostream> 

SfmlControlDevice::SfmlControlDevice() : mFirstConnected(false)
{
    // In constructor
    mWindow.create(sf::VideoMode(50, 50), "teleop");

    // HACKY HACK HACK 
    Eigen::VectorXd cmd(3);
    cmd << 0, 0, 0; 
    setCommand(cmd); 
}

SfmlControlDevice::~SfmlControlDevice()
{

}

bool SfmlControlDevice::handleInput()
{
    while (mWindow.isOpen())
    {
        sf::Event event;
        while (mWindow.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                mWindow.close();
        }

        sf::Vector3i thrustDir{0, 0, 0};

        // Check keyboard state continuously
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
            thrustDir.x += 1;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
            thrustDir.x -= 1;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
            thrustDir.y -= 1;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
            thrustDir.y += 1;
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::E))
            thrustDir.z += 1;
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Q))
            thrustDir.z -= 1; 

        if (sf::Joystick::isConnected(0))
        {
            thrustDir = getThrustDirection(0);
        }

        Eigen::VectorXd cmd(3);
        cmd << thrustDir.x, thrustDir.y, thrustDir.z;
        appendCommand(cmd);

        mWindow.clear();
        mWindow.display();

    }
    return true;
}


void SfmlControlDevice::appendCommand(Eigen::VectorXd anUpdateCmd)
{
    auto current = getCommand(); 
    Eigen::VectorXd result(current.size());

    for (int i = 0; i < current.size(); ++i) 
    {
        result[i] = current[i] == anUpdateCmd[i] ? current[i] : anUpdateCmd[i];    
    }

    setCommand(result); 
}

int SfmlControlDevice::snapAxis(float value, float threshold) 
{
    if (value > threshold) return 1;
    if (value < -threshold) return -1;
    return 0;
}

sf::Vector3i SfmlControlDevice::getThrustDirection(int joystickId) 
{
    if (!sf::Joystick::isConnected(joystickId))
        return {0, 0, 0};

    if (!mFirstConnected)
    {
        mFirstConnected = true; 
        std::cout << "Game Controller connected! Use the left joystick to move the vehicle fwd/back and left/right" << std::endl; 
    }

    float xRaw = sf::Joystick::getAxisPosition(joystickId, sf::Joystick::X); // right = +
    float yRaw = sf::Joystick::getAxisPosition(joystickId, sf::Joystick::Y); // down = +
    yRaw = -yRaw; // Flip Y axis

    float xNorm = xRaw / 100.f;
    float yNorm = yRaw / 100.f;

    int zThrust = 0;

    if (sf::Joystick::isButtonPressed(joystickId, sf::Joystick::PovX)) // L1 / LB    
        zThrust -= 1;
    
    if (sf::Joystick::isButtonPressed(joystickId, sf::Joystick::PovY)) // R1 / RB
        zThrust += 1;

    return {snapAxis(xNorm, 0.2f), snapAxis(yNorm, 0.2f), zThrust};
}
