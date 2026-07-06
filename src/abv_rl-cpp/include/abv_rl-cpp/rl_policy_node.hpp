#pragma once

#include <array>
#include <string>
#include <vector>
#include <memory>
#include <tuple>

#include <onnxruntime_cxx_api.h>

#include "rclcpp/rclcpp.hpp"
#include "abv_msgs/srv/abv_control_action.hpp"

// ---------------------------------------------------------------------------
// Obstacle represented as (center_x, center_y, radius)
// ---------------------------------------------------------------------------
struct Obstacle {
    float cx, cy, radius;
};

// ---------------------------------------------------------------------------
// Thin wrapper: loads one ONNX model and runs inference
// ---------------------------------------------------------------------------
class OnnxModel {
public:
    OnnxModel(Ort::Env& env, const std::string& path);

    // Run single-batch inference: input shape [1, input_dim]
    // Returns flat output vector (all output tensors concatenated)
    std::vector<float> run(const std::vector<float>& input);

    int input_dim()  const { return input_dim_;  }
    int output_dim() const { return output_dim_; }

private:
    Ort::Session          session_;
    Ort::AllocatorWithDefaultOptions allocator_;

    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;
    std::vector<std::string> input_names_buf_;
    std::vector<std::string> output_names_buf_;

    int input_dim_  = 0;
    int output_dim_ = 0;
};

// ---------------------------------------------------------------------------
// Inference controller: protagonist + adversary + critic ensemble
// ---------------------------------------------------------------------------
class RLController {
public:
    explicit RLController(const std::string& model_dir,
                          int num_obstacles,
                          int num_critics = 3);

    // Returns [fx, fy, fyaw] action
    std::array<float, 3> compute(
        const std::array<float, 3>& pose,       // x, y, yaw
        const std::array<float, 3>& velocity,   // vx, vy, omega
        const std::array<float, 3>& error       // rel_x, rel_y, rel_yaw
    );

    void set_obstacles(std::vector<Obstacle> obs) { obstacles_ = std::move(obs); }

private:
    // Build observation vector from pose/vel/error + obstacle margins
    std::vector<float> build_obs(
        const std::array<float, 3>& pose,
        const std::array<float, 3>& velocity,
        const std::array<float, 3>& error
    ) const;

    // Returns (dir_x, dir_y, margin) for one obstacle.
    // margin > 0 outside, < 0 inside (negativeInside=false convention from Python)
    std::tuple<float,float,float> circle_margin(
        float px, float py, const Obstacle& obs
    ) const;

    Ort::Env env_;

    std::unique_ptr<OnnxModel> protagonist_;
    std::unique_ptr<OnnxModel> adversary_;
    std::vector<std::unique_ptr<OnnxModel>> critics_;

    std::vector<Obstacle> obstacles_;
    int state_dim_;
};

// ---------------------------------------------------------------------------
// ROS 2 node
// ---------------------------------------------------------------------------
class RLPolicyNode : public rclcpp::Node {
public:
    RLPolicyNode();

private:
    void handle_request(
        const std::shared_ptr<abv_msgs::srv::AbvControlAction::Request>  req,
              std::shared_ptr<abv_msgs::srv::AbvControlAction::Response> res
    );

    std::unique_ptr<RLController> controller_;
    rclcpp::Service<abv_msgs::srv::AbvControlAction>::SharedPtr service_;
};
