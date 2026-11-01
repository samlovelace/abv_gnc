#pragma once
#include <memory>
#include "autobot_obj_detection/IDetector.hpp"

std::unique_ptr<IDetector> makeDetector(const DetectorConfig& cfg);
