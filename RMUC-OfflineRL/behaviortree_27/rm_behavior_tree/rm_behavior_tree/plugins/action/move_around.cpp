#include "rm_behavior_tree/plugins/action/move_around.hpp"

#include "rm_behavior_tree/map_accessor.hpp"          // MapAccessor：世界↔栅格、占用查询
#include "rm_behavior_tree/visibility_checker.hpp"    // VisibilityChecker + KeyPoint

#include <iomanip>      // std::fixed / std::setprecision
#include <sstream>      // std::ostringstream

namespace rm_behavior_tree
{

// ─────────────────────────── 构造函数 ───────────────────────────
MoveAroundAction::MoveAroundAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::SyncActionNode(name, config),   // ① 先构造父类（必须与 hpp 里的基类一致）
  node_(params.nh)                    // ② 接住外部给的 ROS 节点（已被 executor spin 的那个）
{
  // ③ 出厂默认值：三层默认值的最底层（端口没配、说明书也没默认值时用它）
  map_topic_ = "/map";
  expected_dis_ = 0.3;
  goal_clearance_ = 0.0;
  max_resample_ = 20;
  allow_unknown_ = false;

  // ④ 从端口覆盖（拿到就用，拿不到就保持上面的值）
  getInput("map_topic", map_topic_);
  getInput("expected_dis", expected_dis_);
  getInput("goal_clearance", goal_clearance_);
  getInput("max_resample", max_resample_);
  getInput("allow_unknown", allow_unknown_);

  if (max_resample_ < 1) {
    max_resample_ = 1;                // 防止配成 0/负数 → 循环一次都不进 → 必然失败
  }
  if (expected_dis_ < 0.0) {
    expected_dis_ = 0.0;              // 防止负数半径
  }

  rng_.seed(std::random_device{}());  // ⑤ 随机引擎只播一次种
  initMapSubscription();              // ⑥ 构造期就订阅（不能等到 tick，见下方说明）
}

// ─────────────────────── 地图订阅（只做一次）───────────────────────
void MoveAroundAction::initMapSubscription()
{
  if (map_sub_) {
    return;                           // 幂等：已经订过就不再订
  }

  // /map 一般是 latched（只发一次），订阅者必须用 transient_local + reliable，
  // 否则"订阅得晚"就永远收不到（对比：PursueEnemy 用了 best_effort 踩过这个坑）
  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.transient_local().reliable();

  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, map_qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(map_mutex_);   // 上锁；离开 {} 自动解锁
      latest_map_ = msg;                              // 只存最新一帧（共享指针，不拷贝）
    });

  RCLCPP_INFO(node_->get_logger(),
              "MoveAround: subscribing map topic '%s'", map_topic_.c_str());
}

// ─────────────────── 主逻辑：一次 tick 算出一个点 ───────────────────
BT::NodeStatus MoveAroundAction::tick()
{
  // ① 读机器人当前位置（每个 tick 都可能是新的）
  auto loc = getInput<geometry_msgs::msg::TransformStamped>("message");
  if (!loc) {
    RCLCPP_ERROR(node_->get_logger(),
                 "MoveAround: missing required input [message]: %s", loc.error().c_str());
    return BT::NodeStatus::FAILURE;     // 绝不能返回 RUNNING！
  }
  const double cx = loc.value().transform.translation.x;
  const double cy = loc.value().transform.translation.y;
  const double cz = loc.value().transform.translation.z;

  // ② 拿地图（整段 tick 都持有锁：保证这一轮用的地图不会被换掉）
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (!latest_map_) {
    RCLCPP_WARN(node_->get_logger(),
                "MoveAround: no map on '%s' yet, cannot validate points",
                map_topic_.c_str());
    return BT::NodeStatus::FAILURE;     // 保守策略：校验不了就不发点
  }
  MapAccessor map_accessor(*latest_map_);   // 每 tick 只拷贝一次地图

  // ③ 抽一个合法点
  double gx = 0.0, gy = 0.0;
  if (!sampleUsablePoint(map_accessor, cx, cy, gx, gy)) {
    RCLCPP_WARN(node_->get_logger(),
                "MoveAround: no usable point after %d tries (center=(%.3f,%.3f) r=%.2f)",
                max_resample_, cx, cy, expected_dis_);
    return BT::NodeStatus::FAILURE;     // → RetryUntilSuccessful 再试一轮
  }

  // ④ 写黑板，交给 SendGoal
  setOutput("goal_pose", formatGoalPose(gx, gy, cz));

  RCLCPP_INFO(node_->get_logger(),
              "MoveAround: goal=(%.3f,%.3f) center=(%.3f,%.3f) r=%.2f",
              gx, gy, cx, cy, expected_dis_);
  return BT::NodeStatus::SUCCESS;       // 规划完成（不等于"走到了"，走路是 SendGoal 的事）
}

// ──────────────────── 判点：三级校验 ────────────────────
bool MoveAroundAction::isPointUsable(const MapAccessor & map, double cx, double cy,
                                     double x, double y)
{
  const auto & info = latest_map_->info;

  // ── 第 1 关：必须在栅格图范围内 ──
  // 注意：MapAccessor::worldToMap 内部是 static_cast<int> 截断（不是向下取整），
  //       坐标略小于 origin 时（如 -0.3）会被截断成 0 → 误判成"在图内"，
  //       所以这里额外用 origin 做一次显式边界检查。
  if (x < info.origin.position.x || y < info.origin.position.y) {
    return false;
  }
  int mx = 0, my = 0;
  if (!map.worldToMap(x, y, mx, my)) {
    return false;                     // 超出图的上/右边界
  }

  // ── 第 2 关：不能是障碍、也不能是未知区域 ──
  const int idx = my * static_cast<int>(info.width) + mx;
  const int value = latest_map_->data[idx];
  if (value > 50) {
    return false;                     // 占用值 >50 = 障碍（与 MapAccessor 的阈值保持一致）
  }
  if (value < 0 && !allow_unknown_) {
    return false;                     // -1 = 未知区域（MapAccessor 把 -1 当可通行，这里自己挡）
  }

  // ── 第 3 关：从当前位置到落点必须直线可达 ──
  double resolution = map.getResolution();
  if (resolution <= 0.0) {
    resolution = 0.05;                // 兜底，避免除 0
  }
  const int clearance_cells = static_cast<int>(goal_clearance_ / resolution);

  KeyPoint from;  from.x = cx;  from.y = cy;
  KeyPoint to;    to.x = x;     to.y = y;

  return VisibilityChecker::isVisible(from, to, map, clearance_cells);
}

// ──────────────── 抽点：随机 + 重抽 ────────────────
bool MoveAroundAction::sampleUsablePoint(const MapAccessor & map, double cx, double cy,
                                         double & out_x, double & out_y)
{
  for (int attempt = 0; attempt < max_resample_; ++attempt) {
    const double angle = angle_dist_(rng_);                        // [0, 2π) 上的均匀角度
    const double x = cx + expected_dis_ * std::cos(angle);         // 圆周上取点
    const double y = cy + expected_dis_ * std::sin(angle);

    if (isPointUsable(map, cx, cy, x, y)) {
      out_x = x;                      // 通过引用参数把结果交出去
      out_y = y;
      return true;
    }
    // 不合格就继续下一轮（这里不打印日志，避免刷屏；失败次数在 tick 里统一报）
  }
  return false;                       // 抽满次数还没有 → 失败
}

// ──────────────── 拼字符串：给 SendGoal 的接口 ────────────────
std::string MoveAroundAction::formatGoalPose(double x, double y, double z)
{
  std::ostringstream oss;
  // fixed + setprecision(4)：固定小数 4 位，
  //   - 避免默认格式在某些数值下输出科学计数法（如 1e-05）
  //   - 4 位小数 = 0.1 毫米精度，足够
  oss << std::fixed << std::setprecision(4)
      << x << ";" << y << ";" << z << ";0;0;0;1";   // 姿态用单位四元数 (0,0,0,1)
  return oss.str();
}

}  // namespace rm_behavior_tree

// ──────────────── 注册为"需要 RosNodeParams 的插件" ────────────────
#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::MoveAroundAction, "MoveAround");
