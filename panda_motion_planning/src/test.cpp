#include <ros/ros.h>
#include <panda_motion_planning/panda_ik_analytic.h>
#include <iostream>

int main()
{
    std::array<double, 16> O_T_EE_array = {1, 0, 0,
                                           0, -1, 0,
                                           0, 0, -1,
                                           0.2, 0, 0.487};
    // std::array<double, 16> O_T_EE_array = {1, 0, 0, 0.3,
    //                                        0, 1, 0, 0.0,
    //                                        0, 0, 1, 0.2};
    std::array<double, 7> q_actual_array =  {0.0, -0.785, 0, -2.355, 0.0, 1.571, 0.785};
    double q7 = 0.0; 

    auto result = franka_IK_EE(O_T_EE_array, q7, q_actual_array);
    
    for (int j=0; j<4;j++)
    {
        for (int i=0; i<7; i++)
        {
            std::cout << result[j][i] << "\n";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    return 0;
}