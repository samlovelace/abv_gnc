#ifndef UTIL_HPP
#define UTIL_HPP

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace util
{
    namespace fs = std::filesystem;

    inline fs::path getConfigPath(const std::string& filename)
    {
        const char* configDir = std::getenv("AUTOBOT_CONFIG_DIR");

        if (!configDir) {
            throw std::runtime_error(
                "AUTOBOT_CONFIG_DIR environment variable not set"
            );
        }

        return fs::path(configDir) / filename;
    }
} // namespace util 
#endif // UTIL_HPP