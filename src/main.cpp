#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

//#include "telemetry_streamer_odom/config.hpp"

// 你的节点类头（保持与你的实现一致）
//#include "telemetry_streamer_odom/protocol.hpp"          // 仅为确保链接库加载无误，可选
// 如果节点类在单独头里：#include "telemetry_streamer_odom/telemetry_streamer_node.hpp"
// 本示例里节点类在 src/telemetry_streamer_node.cpp 内部声明并编译到库中，
// 这里直接在 main 里 forward-declare（若需要的话）。
//class TelemetryStreamerNode;  // 如果你把类放到了独立头里，就删掉这行并改用那个头
#include "telemetry_streamer_odom/telemetry_streamer_node.hpp"  // <-- 添加这个头文件


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // 1) 计算默认配置路径：<share>/config/streams.xml
  std::string xml_path_default;
  try {
    const auto share_dir = ament_index_cpp::get_package_share_directory("telemetry_streamer_odom");
    xml_path_default = share_dir + "/config/streams.xml";
  } catch (const std::exception &e) {
    // 如果包未安装或找不到，给出可读错误
    std::cerr << "[telemetry_streamer_odom] Failed to locate package share directory: "
              << e.what() << std::endl;
    rclcpp::shutdown();
    return 2;
  }

  // 2) 如果命令行传入自定义路径，就用 argv[1]
  std::string xml_path = xml_path_default;
  if (argc > 1 && argv[1] && std::string(argv[1]).size() > 0) {
    xml_path = argv[1];
  }

  // 3) 读取配置
  FullConfig cfg;
  if (!load_config_from_xml(xml_path, cfg)) {
    std::cerr << "[telemetry_streamer_odom] Failed to load config: " << xml_path << std::endl;
    std::cerr << "  Hint: ensure the file exists. Default expected at: "
              << xml_path_default << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  // 4) 创建并运行节点
  // 注意：TelemetryStreamerNode 的构造函数签名需与之前实现一致 (const FullConfig&).
  auto node = std::make_shared<TelemetryStreamerNode>(cfg);

  RCLCPP_INFO(node->get_logger(),
              "telemetry_streamer_odom running. Using config: %s",
              xml_path.c_str());

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
