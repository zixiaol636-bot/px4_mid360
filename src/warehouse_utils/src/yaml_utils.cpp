// yaml_utils.cpp
//
// 这个文件是 warehouse_utils 包里的 YAML 工具实现文件，主要给其他模块提供两类公共能力：
// 1. 航点文件读写：
//    - save_waypoints() 把程序里的 Waypoint 航点列表保存成 YAML 文件；
//    - load_waypoints() 从 YAML 文件读取航点，并转换回程序中的 Waypoint 结构体。
// 2. ROS2 参数读取：
//    - load_param_double() 读取 double 类型参数；
//    - load_param_string() 读取 string 类型参数；
//    - 如果参数没有配置，就使用调用者传入的默认值。
//
// 它本身不负责控制无人机，也不执行任务；它更像一个“数据转换和配置读取工具”，
// 帮助航点规划、任务执行等模块方便地保存/加载任务数据。

// 引入本模块的头文件，里面声明了 Waypoint 结构体和本文件实现的函数接口。
#include "warehouse_utils/yaml_utils.h"

// 引入 yaml-cpp 库，用来读取 YAML 文件、访问 YAML 节点、生成 YAML 文本。
#include <yaml-cpp/yaml.h>

// 引入文件输出流 std::ofstream，用于把生成好的 YAML 内容写入磁盘文件。
#include <fstream>
// 引入标准异常类 std::runtime_error，用于在文件或 YAML 格式错误时抛出异常。
#include <stdexcept>
// 引入 std::string 和 std::to_string，用于字符串字段、错误信息拼接等。
#include <string>

namespace warehouse_utils
{

namespace
{

// 把 YAML 字段名和默认值集中放在这里。
// 这样以后如果字段名要改，只需要改一处，避免保存和读取两边不一致。
constexpr char kFrameIdField[] = "frame_id";
constexpr char kPositionField[] = "position";
constexpr char kOrientationField[] = "orientation";
constexpr char kHoverTimeField[] = "hover_time";
constexpr char kAcceptanceRadiusField[] = "acceptance_radius";

constexpr char kDefaultFrameId[] = "map";
constexpr double kDefaultHoverTime = 0.0;
constexpr double kDefaultAcceptanceRadius = 1.0;

// 匿名命名空间里的函数只在当前 cpp 文件中可见。
// 这个校验函数只服务于本文件的 YAML 读取逻辑，不需要暴露到头文件里。
//
// 检查 YAML 中某个数组字段是否符合预期格式。
// 这里主要用于航点的 position 和 orientation：
// - position 必须是 [x, y, z]，长度为 3；
// - orientation 必须是 [x, y, z, w]，长度为 4。
// 如果字段缺失、不是数组，或数组长度不对，就抛出异常交给外层处理。
void validate_sequence_size(const YAML::Node & node, std::size_t expected, const char * field_name)
{
  // 初学者容易忽略的一点：YAML 字段不一定存在，也不一定是数组。
  // 所以读取 node[0]、node[1] 之前，必须先确认它真的是指定长度的数组。
  if (!node || !node.IsSequence() || node.size() != expected) {
    throw std::runtime_error(std::string("Invalid waypoint field: ") + field_name);
  }
}

// 把一个 Waypoint 写入 YAML::Emitter。
// save_waypoints() 负责整体文件流程；这个函数只负责“单个航点怎么写”。
void emit_waypoint(YAML::Emitter & out, const Waypoint & wp)
{
  // 每个航点写成一个 map，也就是一组 key-value。
  out << YAML::BeginMap;

  // frame_id 来自 PoseStamped 的 header，表示这些坐标属于哪个坐标系。
  out << YAML::Key << kFrameIdField << YAML::Value << wp.pose.header.frame_id;

  // position 使用 Flow 风格写成一行，便于人工查看和编辑。
  // wp.pose.pose 看起来有两个 pose，是因为：
  // - wp.pose 是 geometry_msgs::msg::PoseStamped；
  // - wp.pose.pose 才是里面真正的 Pose，包含 position 和 orientation。
  out << YAML::Key << kPositionField << YAML::Value
      << YAML::Flow << YAML::BeginSeq
      << wp.pose.pose.position.x
      << wp.pose.pose.position.y
      << wp.pose.pose.position.z
      << YAML::EndSeq;

  // orientation 存储 ROS 中常用的四元数，顺序为 x/y/z/w。
  // 注意这里不是欧拉角 roll/pitch/yaw，而是四元数；保存和读取时顺序必须一致。
  out << YAML::Key << kOrientationField << YAML::Value
      << YAML::Flow << YAML::BeginSeq
      << wp.pose.pose.orientation.x
      << wp.pose.pose.orientation.y
      << wp.pose.pose.orientation.z
      << wp.pose.pose.orientation.w
      << YAML::EndSeq;

  // hover_time 表示到达航点后停留多久；acceptance_radius 表示离目标多近算到达。
  // 这两个字段不是 ROS Pose 自带的，是本项目 Waypoint 结构体额外定义的任务属性。
  out << YAML::Key << kHoverTimeField << YAML::Value << wp.hover_time;
  out << YAML::Key << kAcceptanceRadiusField << YAML::Value << wp.acceptance_radius;
  out << YAML::EndMap;
}

// 从一个 YAML 节点解析出一个 Waypoint。
// load_waypoints() 负责读取文件和遍历数组；这个函数只负责“单个航点怎么读”。
Waypoint parse_waypoint(const YAML::Node & node, std::size_t index)
{
  if (!node || !node.IsMap()) {
    throw std::runtime_error("Waypoint item must be a map at index " + std::to_string(index));
  }

  // 先校验长度，再按下标读取，避免 position[2] 或 orientation[3] 越界/无效。
  validate_sequence_size(node[kPositionField], 3, kPositionField);
  validate_sequence_size(node[kOrientationField], 4, kOrientationField);

  Waypoint wp;

  // as<std::string>("map") 表示：
  // - 如果 frame_id 存在，就把它转成 string；
  // - 如果 frame_id 缺失，就使用默认值 "map"。
  wp.pose.header.frame_id = node[kFrameIdField].as<std::string>(kDefaultFrameId);

  // position 已经在上面确认是长度为 3 的数组，所以这里可以安全读取 0/1/2。
  // as<double>() 会把 YAML 数值转换成 C++ double；如果内容不是数字，会抛异常。
  wp.pose.pose.position.x = node[kPositionField][0].as<double>();
  wp.pose.pose.position.y = node[kPositionField][1].as<double>();
  wp.pose.pose.position.z = node[kPositionField][2].as<double>();

  // orientation 同理，按保存时约定的 x/y/z/w 顺序恢复四元数。
  wp.pose.pose.orientation.x = node[kOrientationField][0].as<double>();
  wp.pose.pose.orientation.y = node[kOrientationField][1].as<double>();
  wp.pose.pose.orientation.z = node[kOrientationField][2].as<double>();
  wp.pose.pose.orientation.w = node[kOrientationField][3].as<double>();

  // 这两个字段允许旧配置文件不写；缺失时分别使用括号里的默认值。
  wp.hover_time = node[kHoverTimeField].as<double>(kDefaultHoverTime);
  wp.acceptance_radius = node[kAcceptanceRadiusField].as<double>(kDefaultAcceptanceRadius);

  return wp;
}

// 读取 ROS2 参数的通用小工具。
// public 函数仍然保留 double/string 两个明确接口；内部用模板减少重复代码。
template<typename T>
T load_param(
  const rclcpp::Node::SharedPtr & node,
  const std::string & param_name,
  const T & default_val)
{
  // 先放入默认值，保证即使参数不存在，val 也有一个合理结果。
  T val = default_val;

  // get_parameter_or 的含义是“如果参数存在就读取，否则使用默认值”。
  // 第二个参数 val 是输出变量，函数会把最终结果写到这里。
  node->get_parameter_or(param_name, val, default_val);
  return val;
}

}  // namespace

// 将航点列表保存成 YAML 文件。
// 保存后的文件顶层是一个数组，每个元素代表一个航点，格式大致如下：
// - frame_id: map
//   position: [x, y, z]
//   orientation: [x, y, z, w]
//   hover_time: 0.0
//   acceptance_radius: 1.0
//
// 函数内部会捕获文件写入、YAML 生成等异常：
// - 成功返回 true；
// - 失败打印 ROS2 错误日志并返回 false。
bool save_waypoints(const std::string & filepath, const std::vector<Waypoint> & waypoints)
{
  try {
    // YAML::Emitter 可以理解成“YAML 字符串拼装器”。
    // 后面的 out << ... 不是打印到终端，而是在逐步生成最终要写入文件的 YAML 文本。
    YAML::Emitter out;

    // 顶层用 BeginSeq/EndSeq 包起来，表示整个文件是一个列表。
    // 因为一个任务通常包含多个航点，所以这里不是单个 map，而是航点数组。
    out << YAML::BeginSeq;

    for (const auto & wp : waypoints) {
      emit_waypoint(out, wp);
    }

    out << YAML::EndSeq;

    // Emitter 只是在内存中生成了 YAML 文本；真正落盘还需要 ofstream 写文件。
    std::ofstream fout(filepath);
    if (!fout.is_open()) {
      throw std::runtime_error("Unable to open waypoint file for writing");
    }

    // out.c_str() 取出 Emitter 里已经拼好的 YAML 文本，然后写入文件。
    fout << out.c_str();
    return true;
  } catch (const std::exception & e) {
    // 这里统一捕获异常，调用者只需要看 true/false，不用处理 yaml-cpp 或文件流的各种异常类型。
    RCLCPP_ERROR(rclcpp::get_logger("yaml_utils"), "Failed to save waypoints: %s", e.what());
    return false;
  }
}

// 从 YAML 文件加载航点列表。
// 读取时要求顶层必须是数组；每个航点必须至少包含合法的 position 和 orientation。
// frame_id、hover_time、acceptance_radius 允许缺省：
// - frame_id 默认是 "map"；
// - hover_time 默认是 0.0；
// - acceptance_radius 默认是 1.0。
//
// 如果文件不存在、YAML 格式错误、字段类型不匹配，函数会打印错误日志并返回已解析结果。
// 通常加载失败时返回空数组；如果中途某个航点出错，前面已解析成功的航点会保留。
std::vector<Waypoint> load_waypoints(const std::string & filepath)
{
  std::vector<Waypoint> waypoints;

  try {
    // LoadFile 会读取文件并解析 YAML。
    // 如果文件不存在、语法不合法，yaml-cpp 会抛异常，下面的 catch 会接住。
    const YAML::Node config = YAML::LoadFile(filepath);

    // 本项目的航点文件顶层必须是数组：
    // 正确：- frame_id: map
    // 错误：frame_id: map
    // 如果顶层不是数组，后面的 for (const auto & node : config) 就不是按航点遍历了。
    if (!config.IsSequence()) {
      throw std::runtime_error("Waypoint file must contain a sequence");
    }

    // 提前按航点数量分配容量，避免 push_back 时反复扩容；航点越多越有意义。
    waypoints.reserve(config.size());

    std::size_t index = 0;
    for (const auto & node : config) {
      // 当前航点所有字段都解析成功后，才加入结果数组。
      // 如果某一行转换失败，会跳到 catch，不会把半成品 wp 加进去。
      waypoints.push_back(parse_waypoint(node, index));
      ++index;
    }
  } catch (const std::exception & e) {
    // 读取失败时不抛给上层，而是记录日志并返回当前结果。
    // 这种写法适合工具函数：调用者可以通过返回的数组是否为空来做后续判断。
    RCLCPP_ERROR(rclcpp::get_logger("yaml_utils"), "Failed to load waypoints: %s", e.what());
  }

  return waypoints;
}

// 读取 double 类型 ROS2 参数的便捷封装。
// 如果节点里没有声明或设置该参数，就返回 default_val。
double load_param_double(
  const rclcpp::Node::SharedPtr & node,
  const std::string & param_name,
  double default_val)
{
  return load_param<double>(node, param_name, default_val);
}

// 读取 string 类型 ROS2 参数的便捷封装。
// 用法和 load_param_double 相同，只是返回值类型换成字符串。
std::string load_param_string(
  const rclcpp::Node::SharedPtr & node,
  const std::string & param_name,
  const std::string & default_val)
{
  // 常用于读取 frame 名称、配置文件路径、话题名等文本参数。
  return load_param<std::string>(node, param_name, default_val);
}

}  // namespace warehouse_utils
