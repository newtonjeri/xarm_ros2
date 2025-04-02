#include <cstdint>
#include <iostream>
#include <vector>

bool isFinalPose(const std::vector<double> pose,
                 const std::vector<double> previous_pose)
    {
        // Check if we've reached the final position
        const double position_tolerance = 0.005; // 5mm
        const double orientation_tolerance = 0.01;
    
        return (std::abs(pose[0] - previous_pose[0]) < position_tolerance &&
                std::abs(pose[1] - previous_pose[1]) < position_tolerance &&
                std::abs(pose[2] - previous_pose[2]) < position_tolerance &&
                std::abs(pose[3] - previous_pose[3]) < orientation_tolerance &&
                std::abs(pose[4] - previous_pose[4]) < orientation_tolerance &&
                std::abs(pose[5] - previous_pose[5]) < orientation_tolerance);
    }
int main()
{
    uint8_t myNumber = 70000;
    // std::cout << "The value of myNumber is: " << myNumber << std::endl;
    printf("The value of myNumber is: %d\n", myNumber);
    // Define the pose and previous_pose
    std::vector<double> pose = {0.4912, 0.0661, 0.0650, 1.0, 0.0, 0.0};
    std::vector<double> previous_pose = {0.4911, 0.0661, 0.0650, 1.0, 0.0, 0.0};

    // Call the function
    int is_same = isFinalPose(pose, previous_pose);
    std::cout << "Final pose check completed. " << is_same << std::endl;
    return 0;
}