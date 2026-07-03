#ifndef CONFIG_READER_H_
#define CONFIG_READER_H_

#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace Planing
{
    class ConfigReader // 配置文件读取器
    {
    public:
        ConfigReader();

    private:

    };
}  // namespace Planing
#endif  // CONFIG_READER_H_