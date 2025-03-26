#ifndef ROSBAG_RECORDER_SERVICE_HPP_
#define ROSBAG_RECORDER_SERVICE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_transport/recorder.hpp"

#include "rosbag_recorder_service/srv/record_start.hpp"
#include "rosbag_recorder_service/srv/record_stop.hpp"
#include "rosbag_recorder_service/srv/get_status.hpp"

namespace rosbag_recorder_service
{

class RosbagRecorderService : public rclcpp::Node
{
public:
  explicit RosbagRecorderService(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~RosbagRecorderService();

private:
  // サービスコールバック
  void handle_record_start(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<srv::RecordStart::Request> request,
    std::shared_ptr<srv::RecordStart::Response> response);

  void handle_record_stop(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<srv::RecordStop::Request> request,
    std::shared_ptr<srv::RecordStop::Response> response);

  void handle_get_status(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<srv::GetStatus::Request> request,
    std::shared_ptr<srv::GetStatus::Response> response);

  // ヘルパー関数
  bool start_recording(
    const std::string & bag_name,
    const std::vector<std::string> & topics,
    const std::string & compression_mode,
    const std::string & compression_format,
    const std::string & storage_id,
    const std::string & uri,
    double max_bagfile_size,
    double max_cache_size);

  bool stop_recording();
  void start_node();
  void reset_node();

  // サービスサーバー
  rclcpp::Service<srv::RecordStart>::SharedPtr record_start_service_;
  rclcpp::Service<srv::RecordStop>::SharedPtr record_stop_service_;
  rclcpp::Service<srv::GetStatus>::SharedPtr get_status_service_;

  // Executorとスピンスレッド関連
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::thread spin_thread_;
  
  // レコーダー関連
  std::shared_ptr<rosbag2_transport::Recorder> recorder_;
  bool is_recording_;
  std::string current_bag_path_;
  uint64_t recorded_messages_;
  std::chrono::time_point<std::chrono::steady_clock> start_time_;
};

}  // namespace rosbag_recorder_service

#endif  // ROSBAG_RECORDER_SERVICE_HPP_