#include "autobot_obj_detection/DetectorFactory.hpp"
#include "autobot_obj_detection/YoloSegDetector.hpp"
#include <stdexcept>

std::unique_ptr<IDetector> makeDetector(const DetectorConfig& cfg)
{
    if (cfg.type == "yolo11-seg") return std::make_unique<YoloSegDetector>(cfg);
    throw std::invalid_argument("unknown detector type: " + cfg.type);
}
