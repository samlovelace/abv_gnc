#include "abv_rl-cpp/rl_policy_node.hpp"

#include <cmath>
#include <stdexcept>
#include <numeric>

// ==========================================================================
// OnnxModel
// ==========================================================================

OnnxModel::OnnxModel(Ort::Env& env, const std::string& path)
    : session_(env, path.c_str(), Ort::SessionOptions{})
{
    // --- collect input / output names & dims ---
    size_t n_in  = session_.GetInputCount();
    size_t n_out = session_.GetOutputCount();

    for (size_t i = 0; i < n_in; ++i) {
        auto name = session_.GetInputNameAllocated(i, allocator_);
        input_names_buf_.emplace_back(name.get());
    }
    for (size_t i = 0; i < n_out; ++i) {
        auto name = session_.GetOutputNameAllocated(i, allocator_);
        output_names_buf_.emplace_back(name.get());
    }
    for (auto& s : input_names_buf_)  input_names_.push_back(s.c_str());
    for (auto& s : output_names_buf_) output_names_.push_back(s.c_str());

    // Derive input_dim from the first input tensor's last dimension
    auto info = session_.GetInputTypeInfo(0);
    auto shape = info.GetTensorTypeAndShapeInfo().GetShape();
    input_dim_ = static_cast<int>(shape.back());

    // Output dim: sum of last-dims of all output tensors
    output_dim_ = 0;
    for (size_t i = 0; i < n_out; ++i) {
        auto oi    = session_.GetOutputTypeInfo(i);
        auto oshape = oi.GetTensorTypeAndShapeInfo().GetShape();
        output_dim_ += static_cast<int>(oshape.back());
    }
}

std::vector<float> OnnxModel::run(const std::vector<float>& input)
{
    // Build input tensor [1, input_dim]
    std::array<int64_t, 2> in_shape{1, input_dim_};
    auto mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    Ort::Value in_tensor = Ort::Value::CreateTensor<float>(
        mem_info,
        const_cast<float*>(input.data()),
        input.size(),
        in_shape.data(), in_shape.size()
    );

    auto out_tensors = session_.Run(
        Ort::RunOptions{nullptr},
        input_names_.data(),  &in_tensor,      1,
        output_names_.data(), output_names_.size()
    );

    // Flatten all outputs into one vector
    std::vector<float> result;
    for (auto& t : out_tensors) {
        auto* data = t.GetTensorMutableData<float>();
        auto  info = t.GetTensorTypeAndShapeInfo();
        size_t n   = info.GetElementCount();
        result.insert(result.end(), data, data + n);
    }
    return result;
}

// ==========================================================================
// RLController
// ==========================================================================

RLController::RLController(const std::string& model_dir,
                           int num_obstacles,
                           int num_critics)
    : env_(ORT_LOGGING_LEVEL_WARNING, "rl_policy"),
      state_dim_(6 + 3 * num_obstacles)
{
    protagonist_ = std::make_unique<OnnxModel>(env_, model_dir + "/protagonist.onnx");
    adversary_   = std::make_unique<OnnxModel>(env_, model_dir + "/adversary.onnx");

    for (int i = 0; i < num_critics; ++i) {
        critics_.emplace_back(std::make_unique<OnnxModel>(
            env_, model_dir + "/critic_" + std::to_string(i) + ".onnx"
        ));
    }
}

// circle_margin: negative outside obstacle, positive inside.
// Mirrors Python's calculate_margin_circle(negativeInside=False).
std::tuple<float,float,float>
RLController::circle_margin(float px, float py, const Obstacle& obs) const
{
    float dx   = px - obs.cx;
    float dy   = py - obs.cy;
    float dist = std::sqrt(dx*dx + dy*dy) + 1e-10f;
    float dir_x  = dx / dist;
    float dir_y  = dy / dist;
    float margin = (dist - 1e-10f) - obs.radius;   // subtract the eps back out
    return {dir_x, dir_y, -margin};
}

std::vector<float> RLController::build_obs(
    const std::array<float, 3>& pose,
    const std::array<float, 3>& velocity,
    const std::array<float, 3>& error) const
{
    // [rel_x, rel_y, theta, vx, vy, omega,  (dir_x, dir_y, g_x) * N_obs]
    std::vector<float> obs;
    obs.reserve(state_dim_);

    obs.push_back(error[0]);    // rel_x
    obs.push_back(error[1]);    // rel_y
    obs.push_back(pose[2]);     // theta
    obs.push_back(velocity[0]); // vx
    obs.push_back(velocity[1]); // vy
    obs.push_back(velocity[2]); // omega

    for (const auto& ob : obstacles_) {
        auto [dx, dy, margin] = circle_margin(pose[0], pose[1], ob);
        obs.push_back(dx);
        obs.push_back(dy);
        obs.push_back(margin);
    }
    return obs;
}

std::array<float, 3> RLController::compute(
    const std::array<float, 3>& pose,
    const std::array<float, 3>& velocity,
    const std::array<float, 3>& error)
{
    auto obs = build_obs(pose, velocity, error);

    // Protagonist returns [mean_x, mean_y, mean_yaw] (deterministic / tanh-squashed)
    // The ONNX model should be exported to output the deterministic mean directly.
    auto pro_out = protagonist_->run(obs);

    // We only need the action; adversary / critics are available if you want
    // to log Q-values, but they don't affect the returned action.
    // auto adv_out = adversary_->run(obs);
    // (critics not called at runtime — Q-value logging left as an exercise)

    return {pro_out[0], pro_out[1], pro_out[2]};
}

// ==========================================================================
// RLPolicyNode
// ==========================================================================

RLPolicyNode::RLPolicyNode() : Node("abv_rl")
{
    // Declare & read parameters
    this->declare_parameter<std::string>("model_dir",     "/home/optimus/abv_gnc/src/abv_rl-cpp/abv_rl_onnx/");
    this->declare_parameter<int>        ("num_obstacles", 3);
    this->declare_parameter<int>        ("num_critics",   3);

    // Obstacle generation params
    this->declare_parameter<int>   ("random_seed", 7);
    this->declare_parameter<double>("x_min", -0.5);
    this->declare_parameter<double>("x_max",  4.0);
    this->declare_parameter<double>("y_min", -0.5);
    this->declare_parameter<double>("y_max",  2.0);
    this->declare_parameter<double>("obstacle_radius", 0.1);

    auto model_dir    = this->get_parameter("model_dir").as_string();
    int  n_obs        = this->get_parameter("num_obstacles").as_int();
    int  n_critics    = this->get_parameter("num_critics").as_int();

    controller_ = std::make_unique<RLController>(model_dir, n_obs, n_critics);

    // Seed-based random obstacle placement (mirrors Python logic exactly)
    {
        int    seed     = this->get_parameter("random_seed").as_int();
        double x_min    = this->get_parameter("x_min").as_double();
        double x_max    = this->get_parameter("x_max").as_double();
        double y_min    = this->get_parameter("y_min").as_double();
        double y_max    = this->get_parameter("y_max").as_double();
        double r        = this->get_parameter("obstacle_radius").as_double();

        std::srand(static_cast<unsigned>(seed));
        std::vector<Obstacle> obstacles;
        obstacles.reserve(n_obs);
        //for (int i = 0; i < n_obs; ++i) {
        //    float cx = static_cast<float>(x_min + (std::rand() / float(RAND_MAX)) * (x_max - x_min));
        //    float cy = static_cast<float>(y_min + (std::rand() / float(RAND_MAX)) * (y_max - y_min));
        //    obstacles.push_back({cx, cy, static_cast<float>(r)});
        // }
        //
        obstacles.push_back({0.957247, -0.122877, 0.1}); 
        obstacles.push_back({2.429205, -0.318909, 0.1}); 
        obstacles.push_back({1.911469, 0.414222, 0.1}); 

        controller_->set_obstacles(std::move(obstacles));
    }

    service_ = this->create_service<abv_msgs::srv::AbvControlAction>(
        "abv/control_action",
        std::bind(&RLPolicyNode::handle_request, this,
                  std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "RL policy node ready (ONNX runtime)");
}

void RLPolicyNode::handle_request(
    const std::shared_ptr<abv_msgs::srv::AbvControlAction::Request>  req,
          std::shared_ptr<abv_msgs::srv::AbvControlAction::Response> res)
{
    std::array<float, 3> pose     = {static_cast<float>(req->pose.x),
                                     static_cast<float>(req->pose.y),
                                     static_cast<float>(req->pose.yaw)};
    std::array<float, 3> velocity = {static_cast<float>(req->vel.x),
                                     static_cast<float>(req->vel.y),
                                     static_cast<float>(req->vel.yaw)};
    std::array<float, 3> error    = {static_cast<float>(req->error.x),
                                     static_cast<float>(req->error.y),
                                     static_cast<float>(req->error.yaw)};

    auto action = controller_->compute(pose, velocity, error);

    res->action.x   = action[0];
    res->action.y   = action[1];
    res->action.yaw = action[2];
    res->is_global  = false;
}

// ==========================================================================
// main
// ==========================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RLPolicyNode>());
    rclcpp::shutdown();
    return 0;
}
