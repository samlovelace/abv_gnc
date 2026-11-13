
#include <vector> 
#include <iostream>
#include <string> 
#include <vector> 
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "autobot_commander/InputHandler.h"
#include "autobot_common/RosTopicManager.h"
#include "autobot_commander/CommanderComms.h"
#include <sstream>

namespace
{
// Hardcoded sim transforms, pending real tf frames for the camera and manip
// base. Values taken directly from the sim config/world files:
//   - sim/mujoco/worlds/rebot-tabletop.xml: "overhead" camera pos="0 0 1.65",
//     no quat given -> identity rotation (MuJoCo's default camera orientation
//     already looks straight down along world -z).
//   - sim/mujoco/config/rebot_tabletop.yaml: base_link mount pos/quat applied
//     to the arm in sim.py via MjModel.body_pos/body_quat. Quat is MuJoCo's
//     (w, x, y, z) order.

Eigen::Isometry3d worldToCamera()
{
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(0.0, 0.0, 1.65);
    return transform;
}

Eigen::Isometry3d worldToManip()
{
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(0.0, -0.30, 0.775);
    transform.linear() = Eigen::Quaterniond(0.707, 0.0, 0.0, 0.707).normalized().toRotationMatrix();
    return transform;
}

Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose& aPose)
{
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(aPose.position.x, aPose.position.y, aPose.position.z);
    transform.linear() = Eigen::Quaterniond(aPose.orientation.w, aPose.orientation.x,
                                             aPose.orientation.y, aPose.orientation.z)
                              .normalized()
                              .toRotationMatrix();
    return transform;
}

geometry_msgs::msg::Pose isometryToPose(const Eigen::Isometry3d& aTransform)
{
    geometry_msgs::msg::Pose pose;

    const Eigen::Vector3d position = aTransform.translation();
    pose.position.x = position.x();
    pose.position.y = position.y();
    pose.position.z = position.z();

    const Eigen::Quaterniond orientation(aTransform.linear());
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();
    return pose;
}

void printPose(const std::string& aLabel, const std::string& aFrameId, const geometry_msgs::msg::Pose& aPose)
{
    std::cout << GREEN << aLabel << " (" << aFrameId << "): "
              << "position (" << aPose.position.x << ", " << aPose.position.y << ", " << aPose.position.z << "), "
              << "orientation (" << aPose.orientation.x << ", " << aPose.orientation.y << ", "
              << aPose.orientation.z << ", " << aPose.orientation.w << ")" << std::endl;
}

constexpr double kPreGraspStandoffM = 0.15;

// GraspPlannerHandler::computeOrientation builds the grasp orientation with
// its local +X axis as the fixed top-down approach direction (rotation.col(0)
// = xAxis). The pre-grasp pose is the same orientation, offset a fixed
// standoff back along that axis (local -X) - works in whatever frame the
// grasp pose is already expressed in, since it's a local-frame offset.
Eigen::Isometry3d preGraspFromGrasp(const Eigen::Isometry3d& aGraspPose)
{
    return aGraspPose * Eigen::Translation3d(-kPreGraspStandoffM, 0.0, 0.0);
}

// [finger_left, finger_right] positions, from
// autobot_manip_bringup/config/manipulators/abv_rebot/manip.yaml's
// gripper.open/gripper.closed - finger_right mimics finger_left with
// multiplier -1 in the URDF, so the two values are always +/- of each other.
const std::vector<double> kGripperOpenPos{0.05, -0.05};
const std::vector<double> kGripperClosedPos{0.0, 0.0};
}  // namespace

InputHandler::InputHandler()
{
    mPackagePath = ament_index_cpp::get_package_share_directory("autobot_commander");
    std::cout << "Package path: " << mPackagePath << std::endl; 
}

InputHandler::~InputHandler()
{

}

void InputHandler::handle(const std::string& anInput)
{
    if ("pose" == anInput)
    {
        std::array<double, 3> vehPose;
        std::string vehPoseStr;

        std::cout << "Enter goal pose (x, y, yaw): ";
        std::getline(std::cin, vehPoseStr);

        std::istringstream iss(vehPoseStr);
        for (int i = 0; i < 3; ++i)
        {
            if (!(iss >> vehPose[i]))
            {
                std::cerr << "Invalid input. Please enter three space-separated numbers." << std::endl;
                return;
            }
        }

        autobot_msgs::msg::Vec3 data; 
        data.set__x(vehPose[0]); 
        data.set__y(vehPose[1]); 
        data.set__yaw(vehPose[2]); 

        autobot_msgs::msg::ControllerCommand cmd; 
        cmd.set__data(data); 
        cmd.set__type("pose");
        
        RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::ControllerCommand>("abv/controller/command", cmd); 
        std::cout << "Published goal waypoint (x y yaw): " << vehPose[0] << ", " << vehPose[1] << ", " << vehPose[2] << std::endl; 
    }
    else if ("vel" == anInput)
    {
        std::array<double, 3> vehPose;
        std::string vehPoseStr;

        std::cout << "Enter goal velocity (xd, yd, yawd): ";
        std::getline(std::cin, vehPoseStr);

        std::istringstream iss(vehPoseStr);
        for (int i = 0; i < 3; ++i)
        {
            if (!(iss >> vehPose[i]))
            {
                std::cerr << "Invalid input. Please enter three space-separated numbers." << std::endl;
                return;
            }
        }

        autobot_msgs::msg::Vec3 data; 
        data.set__x(vehPose[0]); 
        data.set__y(vehPose[1]); 
        data.set__yaw(vehPose[2]); 

        autobot_msgs::msg::ControllerCommand cmd; 
        cmd.set__data(data); 
        cmd.set__type("velocity");
        
        RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::ControllerCommand>("abv/controller/command", cmd); 
        std::cout << "Published goal velocity waypoint (xd yd yawd): " << vehPose[0] << ", " << vehPose[1] << ", " << vehPose[2] << std::endl;
    }
    else if ("stop" == anInput)
    {
        autobot_msgs::msg::Vec3 data; 
        data.set__x(0); 
        data.set__y(0); 
        data.set__yaw(0); 

        autobot_msgs::msg::ControllerCommand cmd; 
        cmd.set__data(data); 
        cmd.set__type("STOP");
        
        RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::ControllerCommand>("abv/controller/command", cmd); 
    }
    else if ("path" == anInput)
    {
        std::string trajType;
        std::string duration; 

        std::cout << GREEN << "Type: "; 
        std::getline(std::cin, trajType);

        std::cout << GREEN << "Duration (s): "; 
        std::getline(std::cin, duration); 
        double dur = std::stod(duration); 

        autobot_msgs::msg::GuidanceCommand cmd; 
        cmd.set__type(trajType); 
        cmd.set__duration(dur); 

        RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::GuidanceCommand>("abv/guidance/command", cmd); 
    }
    else if ("grasp" == anInput)
    {
        handleGrasp();
    }
    else if ("open" == anInput)
    {
        handleGripper(true);
    }
    else if ("close" == anInput)
    {
        handleGripper(false);
    }
    else if ("execute" == anInput)
    {
        handleExecute();
    }
    else if (anInput == "help" || anInput == "--help" || anInput == "-h")
    {
        std::cout << R"(
        ABV Commander Tool
        -----------------------------
        Interact with the air-bearing vehicle by sending pose or velocity waypoints.

        Available Commands:

        pose:       command the abv to a desired x, y, yaw pose
        vel:        command the abv to a desired x, y, yaw velocity
        stop:       command the abv to zero velocity on x, y, yaw axes
        path:       command the abv to follow a path sent by the guidance node
        grasp:      request a grasp pose for a tracked object id from autobot_grasp_planner
        open:       open the gripper
        close:      close the gripper
        execute:    execute a pick plan for a tracked object id via autobot_manip_executor

        Type 'help' or '--help' to show this message again.
        )" << std::endl;
        std::string test; 
        std::getline(std::cin, test); 
    }       
    else{
        std::cout << RED << "Unsupported command!" << std::endl;
    }
}

void InputHandler::handleGrasp()
{
    if (!mPlanGraspClient)
    {
        // Created here, not in the ctor - RosTopicManager's node doesn't exist
        // until rclcpp::init() has run (CommanderComms's ctor), which happens
        // after InputHandler is constructed in main().
        mPlanGraspClient = RosTopicManager::getInstance()->create_client<autobot_msgs::srv::PlanGrasp>("manip/plan_grasp");
    }

    std::cout << "Enter tracked object id: ";
    std::string idStr;
    std::getline(std::cin, idStr);

    int32_t id = 0;
    std::istringstream iss(idStr);
    if (!(iss >> id))
    {
        std::cerr << "Invalid input. Please enter an integer id." << std::endl;
        return;
    }

    auto request = std::make_shared<autobot_msgs::srv::PlanGrasp::Request>();
    request->id = id;

    auto future = mPlanGraspClient->async_send_request(request);
    auto response = future.get();

    if (!response->success)
    {
        std::cout << RED << "Grasp planning failed: " << response->message << std::endl;
        return;
    }

    printPose("Grasp pose", response->grasp_pose.header.frame_id, response->grasp_pose.pose);

    const auto manipPose = transformWorldPoseToManipFrame(response->grasp_pose.pose);
    printPose("Grasp pose", "manip_base", manipPose);

    const auto preGraspWorld = isometryToPose(preGraspFromGrasp(poseToIsometry(response->grasp_pose.pose)));
    const auto preGraspManip = isometryToPose(preGraspFromGrasp(poseToIsometry(manipPose)));

    printPose("Pre-grasp pose", response->grasp_pose.header.frame_id, preGraspWorld);
    printPose("Pre-grasp pose", "manip_base", preGraspManip);
}

geometry_msgs::msg::Pose InputHandler::transformWorldPoseToManipFrame(const geometry_msgs::msg::Pose& aPoseInWorld) const
{
    const Eigen::Isometry3d T_W_C = worldToCamera();
    const Eigen::Isometry3d T_W_M = worldToManip();
    const Eigen::Isometry3d T_W_O = poseToIsometry(aPoseInWorld);

    const Eigen::Isometry3d T_M_C = T_W_M.inverse() * T_W_C;
    const Eigen::Isometry3d T_C_W = T_W_C.inverse();
    const Eigen::Isometry3d T_M_O = T_M_C * T_C_W * T_W_O;

    return isometryToPose(T_M_O);
}

void InputHandler::handleGripper(bool anOpen)
{
    autobot_msgs::msg::JointPositionWaypoint waypoint;
    waypoint.positions = anOpen ? kGripperOpenPos : kGripperClosedPos;

    RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::JointPositionWaypoint>(
        "arm/gripper_position_waypoint", waypoint);

    std::cout << GREEN << "Published gripper " << (anOpen ? "open" : "close") << " waypoint (finger_left, finger_right): "
              << waypoint.positions[0] << ", " << waypoint.positions[1] << std::endl;
}

void InputHandler::executionStatusCallback(const autobot_msgs::msg::ExecutionStatus::SharedPtr aMsg)
{
    std::lock_guard<std::mutex> lk(mExecutionStatusMutex);
    mLatestExecutionStatus = *aMsg;
    mHasExecutionStatus = true;
    mExecutionStatusCv.notify_all();
}

void InputHandler::handleExecute()
{
    if (!mExecutionStatusSubscribed)
    {
        // Created here, not in the ctor - same reasoning as mPlanGraspClient.
        RosTopicManager::getInstance()->createSubscriber<autobot_msgs::msg::ExecutionStatus>(
            "manip/execution_status", std::bind(&InputHandler::executionStatusCallback, this, std::placeholders::_1));
        mExecutionStatusSubscribed = true;
    }

    std::cout << "Enter tracked object id: ";
    std::string idStr;
    std::getline(std::cin, idStr);

    int32_t id = 0;
    std::istringstream iss(idStr);
    if (!(iss >> id))
    {
        std::cerr << "Invalid input. Please enter an integer id." << std::endl;
        return;
    }

    {
        std::lock_guard<std::mutex> lk(mExecutionStatusMutex);
        mHasExecutionStatus = false;
    }

    autobot_msgs::msg::ExecutePickCommand cmd;
    cmd.id = id;
    RosTopicManager::getInstance()->publishMessage<autobot_msgs::msg::ExecutePickCommand>("manip/execute_pick_command", cmd);
    std::cout << GREEN << "Published execute_pick_command for id=" << id << ", waiting for result..." << std::endl;

    std::unique_lock<std::mutex> lk(mExecutionStatusMutex);
    const bool gotTerminal = mExecutionStatusCv.wait_for(lk, std::chrono::seconds(60), [this] {
        return mHasExecutionStatus &&
               (mLatestExecutionStatus.status == autobot_msgs::msg::ExecutionStatus::SUCCEEDED ||
                mLatestExecutionStatus.status == autobot_msgs::msg::ExecutionStatus::FAILED);
    });

    if (!gotTerminal)
    {
        std::cout << RED << "Timed out waiting for execution result." << std::endl;
        return;
    }

    if (mLatestExecutionStatus.status == autobot_msgs::msg::ExecutionStatus::SUCCEEDED)
    {
        std::cout << GREEN << "Pick execution succeeded: " << mLatestExecutionStatus.message << std::endl;
    }
    else
    {
        std::cout << RED << "Pick execution failed: " << mLatestExecutionStatus.message << std::endl;
    }
}
