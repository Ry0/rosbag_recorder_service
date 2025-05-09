#include "rosbag_recorder_service/rosbag_recorder_service.hpp"

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <filesystem>

#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_transport/recorder.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"

namespace rosbag_recorder_service
{

RosbagRecorderService::RosbagRecorderService(const rclcpp::NodeOptions & options)
: Node("rosbag_recorder_service", options),
  is_recording_(false),
  recorded_messages_(0)
{
  // サービスの初期化
  record_start_service_ = create_service<srv::RecordStart>(
    "~/record_start",
    std::bind(
      &RosbagRecorderService::handle_record_start, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  record_stop_service_ = create_service<srv::RecordStop>(
    "~/record_stop",
    std::bind(
      &RosbagRecorderService::handle_record_stop, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  get_status_service_ = create_service<srv::GetStatus>(
    "~/get_status",
    std::bind(
      &RosbagRecorderService::handle_get_status, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  RCLCPP_INFO(get_logger(), "Rosbag recorder service has been initialized.");
}

RosbagRecorderService::~RosbagRecorderService()
{
  if (is_recording_) {
    stop_recording();
  }
}

void RosbagRecorderService::handle_record_start(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<srv::RecordStart::Request> request,
  std::shared_ptr<srv::RecordStart::Response> response)
{
  (void)request_header;  // 未使用パラメータ警告を抑制

  if (is_recording_) {
    response->success = false;
    response->message = "Already recording to " + current_bag_path_;
    return;
  }

  bool result = start_recording(
    request->bag_name,
    request->topics,
    request->compression_mode,
    request->compression_format,
    request->storage_id,
    request->uri,
    request->max_bagfile_size,
    request->max_cache_size);

  response->success = result;
  if (result) {
    response->message = "Recording started to " + current_bag_path_;
  } else {
    response->message = "Failed to start recording";
  }
}

void RosbagRecorderService::handle_record_stop(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<srv::RecordStop::Request> request,
  std::shared_ptr<srv::RecordStop::Response> response)
{
  (void)request_header;  // 未使用パラメータ警告を抑制
  (void)request;         // 未使用パラメータ警告を抑制

  if (!is_recording_) {
    response->success = false;
    response->message = "Not currently recording";
    return;
  }

  bool result = stop_recording();
  response->success = result;
  if (result) {
    response->message = "Recording stopped";
    response->bag_path = current_bag_path_;
  } else {
    response->message = "Failed to stop recording";
  }
}

void RosbagRecorderService::handle_get_status(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<srv::GetStatus::Request> request,
  std::shared_ptr<srv::GetStatus::Response> response)
{
  (void)request_header;  // 未使用パラメータ警告を抑制
  (void)request;         // 未使用パラメータ警告を抑制

  response->is_recording = is_recording_;
  response->current_bag_path = current_bag_path_;
  response->recorded_messages = recorded_messages_;

  if (is_recording_) {
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - start_time_;
    response->elapsed_time = elapsed.count();
  } else {
    response->elapsed_time = 0.0;
  }
}

bool RosbagRecorderService::start_recording(
  const std::string & bag_name,
  const std::vector<std::string> & topics,
  const std::string & compression_mode,
  const std::string & compression_format,
  const std::string & storage_id,
  const std::string & uri,
  double max_bagfile_size,
  double max_cache_size)
{
  try {
    // レコーディングオプションの設定
    rosbag2_transport::RecordOptions record_options;

    if(topics.empty() || topics.size() == 0){
      RCLCPP_WARN(
        get_logger(), "topics is empty. Record all topics.");
      record_options.all_topics = true;
    }
    else{
      record_options.all_topics = false;
      record_options.topics = topics;
    }

    record_options.is_discovery_disabled = false;
    record_options.rmw_serialization_format = "cdr";
    record_options.topic_polling_interval = std::chrono::milliseconds(100);

    // ストレージオプションの設定
    rosbag2_storage::StorageOptions storage_options;
    
    std::string bag_file_name;
    if (bag_name.empty()){
      // 現在の時刻を取得
      auto now = std::chrono::system_clock::now();
      std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    
      // 時刻をフォーマットする
      std::tm tm_struct;
      localtime_r(&now_time, &tm_struct);

      std::ostringstream oss;
      oss << "rosbag2_"
          << std::put_time(&tm_struct, "%Y_%m_%d-%H_%M_%S");
      bag_file_name = oss.str();
      RCLCPP_WARN(
        get_logger(), "bag_name is empty. bag_name is automatically generated. %s", bag_file_name.c_str());
    }
    else{
      bag_file_name = bag_name;
    }

    if (uri.empty()){
      RCLCPP_WARN(
        get_logger(), "uri is empty. the bag file will be saved in current directory. %s", bag_file_name.c_str());
      storage_options.uri = bag_file_name;
    }
    else{
      auto file_path = std::filesystem::path(uri) / std::filesystem::path(bag_file_name);
      storage_options.uri = file_path.string(); 
    }

    if (std::filesystem::exists(storage_options.uri) && std::filesystem::is_directory(storage_options.uri)) {
        try {
            std::uintmax_t count = std::filesystem::remove_all(storage_options.uri);
            RCLCPP_INFO(get_logger(),
                        "Directory '%s' and its %ju contents deleted.",
                        storage_options.uri.c_str(), count);
        } catch (const std::filesystem::filesystem_error& e) {
            RCLCPP_ERROR(get_logger(),
                         "Failed to delete directory '%s': %s",
                         storage_options.uri.c_str(), e.what());
        }
    } 

    storage_options.storage_id = storage_id.empty() ? "mcap" : storage_id;
    storage_options.max_bagfile_size = static_cast<uint64_t>(max_bagfile_size * 1024 * 1024);;
    storage_options.max_cache_size = static_cast<uint64_t>(max_cache_size * 1024 * 1024);;

    // 圧縮オプションの設定
    if (compression_mode.empty()) {
      RCLCPP_WARN(
        get_logger(), "compression_mode is empty. compression_mode is set as none.");
      record_options.compression_mode = "none";
    }
    else{
      record_options.compression_mode = compression_mode;
      if (compression_mode != "none"){
        record_options.compression_format = compression_format.empty() ? "zstd" : compression_format;
      }
    }
    
    // レコーダーの作成
    auto writer = rosbag2_transport::ReaderWriterFactory::make_writer(record_options);
    recorder_ = std::make_shared<rosbag2_transport::Recorder>(
        std::move(writer), storage_options, record_options);

    // ノード実行
    start_node();

    // レコード開始
    recorder_->record();

    // 状態の更新
    is_recording_ = true;
    current_bag_path_ = storage_options.uri;
    recorded_messages_ = 0;
    start_time_ = std::chrono::steady_clock::now();

    if (record_options.all_topics){
      RCLCPP_INFO(
        get_logger(), "Started recording to %s with all topics",
        current_bag_path_.c_str());
    }
    else{
      RCLCPP_INFO(
        get_logger(), "Started recording to %s with topics: %zu",
        current_bag_path_.c_str(), topics.size());
    }

    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to start recording: %s", e.what());
    // ノードの停止
    reset_node();

    if (is_recording_ && recorder_) {
      recorder_.reset();
    }
    
    return false;
  }
}

bool RosbagRecorderService::stop_recording()
{
  if (!is_recording_ || !recorder_) {
    return false;
  }

  try {
    // ノードのリセット
    reset_node();

    // レコーダーのリセット
    recorder_.reset();
    is_recording_ = false;
    
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - start_time_;
    
    RCLCPP_INFO(
      get_logger(), "Stopped recording to %s. Duration: %.2f seconds",
      current_bag_path_.c_str(), elapsed.count());
    
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to stop recording: %s", e.what());
    return false;
  }
}

void RosbagRecorderService::start_node()
{
    // Executorの作成と設定
    exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(recorder_);

    // 別スレッドでスピン
    spin_thread_ = std::thread([this]() {
      exec_->spin();
    });
}

void RosbagRecorderService::reset_node()
{
  if (exec_) {
    exec_->cancel();
    
    // スレッドが終了するまで待機
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    
    // ノードの削除
    exec_->remove_node(recorder_);
    exec_.reset();
  }
}

}  // namespace rosbag_recorder_service
