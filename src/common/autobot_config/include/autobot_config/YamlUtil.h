#pragma once

#include <filesystem>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>

// Shared, validated YAML extraction helpers used by every per-subsystem
// config manager in this package. Hoisted out of the old
// autobot_manip_common::ConfigManager so GNC/common/percep parsing gets the
// same required/optional-with-default semantics manip already had.
namespace autobot_config::YamlUtil
{
    inline YAML::Node require(const YAML::Node& parent,
                               const std::string& key,
                               const std::string& context)
    {
        if (!parent[key] || parent[key].IsNull())
            throw std::runtime_error(
                "[autobot_config] Missing required key '" + key +
                "' in section '" + context + "'");
        return parent[key];
    }

    template<typename T>
    T requireScalar(const YAML::Node& parent,
                     const std::string& key,
                     const std::string& context)
    {
        return require(parent, key, context).as<T>();
    }

    template<typename T>
    T optionalScalar(const YAML::Node& parent,
                      const std::string& key,
                      T defaultVal)
    {
        if (parent[key] && !parent[key].IsNull())
            return parent[key].as<T>();
        return defaultVal;
    }

    template<typename T>
    std::vector<T> requireSeq(const YAML::Node& parent,
                               const std::string& key,
                               const std::string& context,
                               std::size_t expectedSize = 0)
    {
        const YAML::Node& node = require(parent, key, context);
        if (!node.IsSequence())
            throw std::runtime_error(
                "[autobot_config] Key '" + key + "' in '" + context +
                "' must be a sequence");
        auto vec = node.as<std::vector<T>>();
        if (expectedSize > 0 && vec.size() != expectedSize)
            throw std::runtime_error(
                "[autobot_config] Key '" + key + "' in '" + context +
                "' has " + std::to_string(vec.size()) +
                " elements, expected " + std::to_string(expectedSize));
        return vec;
    }

    template<typename T>
    std::optional<std::vector<T>> optionalSeq(const YAML::Node& parent,
                                               const std::string& key,
                                               std::size_t expectedSize = 0)
    {
        if (!parent[key] || parent[key].IsNull())
            return std::nullopt;
        auto vec = parent[key].as<std::vector<T>>();
        if (expectedSize > 0 && vec.size() != expectedSize)
            throw std::runtime_error(
                "[autobot_config] Optional key '" + key +
                "' has " + std::to_string(vec.size()) +
                " elements, expected " + std::to_string(expectedSize));
        return vec;
    }

    template<typename T>
    void append(std::vector<T>& dst, const std::vector<T>& src)
    {
        dst.insert(dst.end(), src.begin(), src.end());
    }

    inline std::string resolvePath(const std::filesystem::path& base,
                                    const std::string& relative)
    {
        return (base / relative).lexically_normal().string();
    }

    inline std::vector<std::string> splitPath(const std::string& path)
    {
        std::vector<std::string> out;
        std::stringstream ss(path);
        std::string item;
        while (std::getline(ss, item, '.'))
            if (!item.empty()) out.push_back(item);
        return out;
    }

    inline Eigen::Vector3d parseVector3d(const YAML::Node& node)
    {
        if (!node || !node.IsSequence() || node.size() != 3)
            throw std::runtime_error("[autobot_config] Invalid Vector3d format in YAML.");

        return Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
    }

    inline std::vector<int> parseIntVector(const YAML::Node& node)
    {
        if (!node || !node.IsSequence())
            throw std::runtime_error("[autobot_config] Invalid int vector format in YAML.");

        std::vector<int> vec;
        vec.reserve(node.size());
        for (const auto& element : node)
            vec.push_back(element.as<int>());
        return vec;
    }
}
