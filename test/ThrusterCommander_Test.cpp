
#include <gtest/gtest.h> 
#include "abv_controller/ThrusterCommander.h"
 
class ThrusterCommander_Test : public ::testing::Test, public ThrusterCommander
{ 
public:
    ThrusterCommander_Test() {
        mConfig.uOn = 0.2; 
        mConfig.uOff = 0.9; 
    }
    ~ThrusterCommander_Test() = default; 

protected:  
   
};


// TODO: add more cases 
TEST_F(ThrusterCommander_Test, ConvertToThrustVector_Test)
{
    std::vector<Eigen::Vector3d> inputs = {
        {2.5, 0, 0}, 
        {0, 2.5, 0}, 
        {0, 0, 2.5}, 
        {0, 0, -2.5},
        {0, -2.5, 0},
        {-2.5, 0, 0} 
    }; 
    
    std::vector<Eigen::Vector3i> outputs = {
        {1, 0, 0}, 
        {0, 1, 0}, 
        {0, 0, 1}, 
        {0, 0, -1},
        {0, -1, 0},
        {-1, 0, 0} 
    };

    for (int i = 0; i < inputs.size(); i++)
    { 
        auto output = convertToThrustVector(inputs[i]); 

        ASSERT_EQ(output,outputs[i]); 
    }

}

// TODO: add rest of cases to test 
TEST_F(ThrusterCommander_Test, ThrusterCommand_Test)
{
    std::vector<Eigen::Vector3i> outputs = {
        {1, 0, 0}, 
        {0, 1, 0}, 
        {0, 0, 1}, 
        {0, 0, -1},
        {0, -1, 0},
        {-1, 0, 0} 
    };

    std::vector<std::string> commands = {
        "900000011",
        "911000000",
        "901000100",
        "910001000", 
        "900001100", 
        "900110000"
    }; 

    for(int i =0; i < outputs.size(); i++)
    {
        determineThrusterCommand(outputs[i]); 

        ASSERT_EQ(mThrusterCommand, commands[i]); 
    }

}



