#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {

rclcpp::QoS make_qos(const std::string & reliability, int depth) {
  auto qos = rclcpp::QoS(rclcpp::KeepLast(std::max(1, depth)));
  if (reliability == "best_effort") {
    qos.best_effort();
  } else if (reliability == "reliable") {
    qos.reliable();
  } else {
    throw std::invalid_argument("unsupported QoS reliability: " + reliability);
  }
  return qos;
}

}  // namespace

class CompressedImageEncoder : public rclcpp::Node {
 public:
  CompressedImageEncoder() : Node("compressed_image_encoder") {
    const auto input_topics = declare_parameter<std::vector<std::string>>(
        "input_topics", std::vector<std::string>{});
    const auto output_topics = declare_parameter<std::vector<std::string>>(
        "output_topics", std::vector<std::string>{});
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 95);
    png_compression_ = declare_parameter<int>("png_compression", 1);
    codec_ = declare_parameter<std::string>("codec", "jpeg");
    log_every_ = declare_parameter<int>("log_every", 512);
    const auto input_reliability =
        declare_parameter<std::string>("input_reliability", "best_effort");
    const auto output_reliability =
        declare_parameter<std::string>("output_reliability", "reliable");
    const auto input_depth = declare_parameter<int>("input_depth", 1);
    const auto output_depth = declare_parameter<int>("output_depth", 10);

    if (input_topics.size() != output_topics.size()) {
      throw std::invalid_argument("input_topics and output_topics sizes differ");
    }
    if (input_topics.empty()) {
      throw std::invalid_argument("input_topics is empty");
    }
    jpeg_quality_ = std::clamp(jpeg_quality_, 1, 100);
    png_compression_ = std::clamp(png_compression_, 0, 9);
    if (codec_ != "jpeg" && codec_ != "png") {
      throw std::invalid_argument("codec must be jpeg or png");
    }

    auto callback_group =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group;

    const auto input_qos = make_qos(input_reliability, input_depth);
    const auto output_qos = make_qos(output_reliability, output_depth);

    counters_.resize(input_topics.size(), 0);
    skipped_counters_.resize(input_topics.size(), 0);
    slots_.reserve(input_topics.size());
    publishers_.resize(input_topics.size());
    subscriptions_.resize(input_topics.size());
    workers_.reserve(input_topics.size());

    for (std::size_t i = 0; i < input_topics.size(); ++i) {
      slots_.push_back(std::make_unique<Slot>());
    }

    for (std::size_t i = 0; i < input_topics.size(); ++i) {
      publishers_[i] =
          create_publisher<sensor_msgs::msg::CompressedImage>(
              output_topics[i], output_qos);
      subscriptions_[i] = create_subscription<sensor_msgs::msg::Image>(
          input_topics[i], input_qos,
          [this, i](sensor_msgs::msg::Image::ConstSharedPtr msg) {
            cache_latest(i, std::move(msg));
          },
          sub_options);
      RCLCPP_INFO(
          get_logger(), "Encoding %s -> %s", input_topics[i].c_str(),
          output_topics[i].c_str());
    }

    for (std::size_t i = 0; i < input_topics.size(); ++i) {
      workers_.emplace_back([this, i, input_topic = input_topics[i]] {
        worker_loop(i, input_topic);
      });
    }
  }

  ~CompressedImageEncoder() override {
    stopping_.store(true);
    for (auto & slot : slots_) {
      slot->cv.notify_all();
    }
    for (auto & worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

 private:
  struct Slot {
    std::mutex mutex;
    std::condition_variable cv;
    sensor_msgs::msg::Image::ConstSharedPtr latest;
    std::uint64_t seq{0};
  };

  void cache_latest(
      std::size_t index, sensor_msgs::msg::Image::ConstSharedPtr msg) {
    auto & slot = slots_[index];
    {
      std::lock_guard<std::mutex> lock(slot->mutex);
      slot->latest = std::move(msg);
      ++slot->seq;
    }
    slot->cv.notify_one();
  }

  void worker_loop(std::size_t index, const std::string & input_topic) {
    std::uint64_t processed_seq = 0;
    while (rclcpp::ok() && !stopping_.load()) {
      sensor_msgs::msg::Image::ConstSharedPtr msg;
      std::uint64_t current_seq = 0;
      {
        auto & slot = slots_[index];
        std::unique_lock<std::mutex> lock(slot->mutex);
        slot->cv.wait(lock, [&] {
          return stopping_.load() || slot->seq != processed_seq;
        });
        if (stopping_.load()) {
          break;
        }
        msg = slot->latest;
        current_seq = slot->seq;
      }

      if (processed_seq != 0 && current_seq > processed_seq + 1) {
        skipped_counters_[index] += current_seq - processed_seq - 1;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Encoder skipped %zu frames on %s because encoding fell behind",
            skipped_counters_[index], input_topic.c_str());
      }
      processed_seq = current_seq;

      if (msg) {
        encode_and_publish(index, input_topic, *msg);
      }
    }
  }

  void encode_and_publish(
      std::size_t index, const std::string & input_topic,
      const sensor_msgs::msg::Image & msg) {
    cv::Mat encoded_input;
    cv::Mat source;

    if (msg.encoding == "rgb8" || msg.encoding == "bgr8") {
      source = cv::Mat(
          static_cast<int>(msg.height), static_cast<int>(msg.width), CV_8UC3,
          const_cast<unsigned char *>(msg.data.data()), msg.step);
      if (msg.encoding == "rgb8" && codec_ == "jpeg") {
        cv::cvtColor(source, encoded_input, cv::COLOR_RGB2BGR);
      } else {
        encoded_input = source;
      }
    } else if (msg.encoding == "mono8" || msg.encoding == "8UC1") {
      encoded_input = cv::Mat(
          static_cast<int>(msg.height), static_cast<int>(msg.width), CV_8UC1,
          const_cast<unsigned char *>(msg.data.data()), msg.step);
    } else if (
        msg.encoding == "16UC1" || msg.encoding == "mono16" ||
        msg.encoding == "z16") {
      encoded_input = cv::Mat(
          static_cast<int>(msg.height), static_cast<int>(msg.width), CV_16UC1,
          const_cast<unsigned char *>(msg.data.data()), msg.step);
    } else {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Unsupported encoding on %s: %s",
          input_topic.c_str(), msg.encoding.c_str());
      return;
    }

    std::vector<unsigned char> encoded;
    std::vector<int> params;
    std::string extension;
    if (codec_ == "jpeg") {
      extension = ".jpg";
      params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    } else {
      extension = ".png";
      params = {cv::IMWRITE_PNG_COMPRESSION, png_compression_};
    }

    if (!cv::imencode(extension, encoded_input, encoded, params)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "%s encoding failed for %s",
          codec_.c_str(), input_topic.c_str());
      return;
    }

    sensor_msgs::msg::CompressedImage out;
    out.header = msg.header;
    out.format = codec_;
    out.data = std::move(encoded);
    publishers_[index]->publish(out);

    const auto count = ++counters_[index];
    if (log_every_ > 0 && count % static_cast<std::size_t>(log_every_) == 0) {
      RCLCPP_INFO(
          get_logger(), "Encoded %zu frames from %s", count,
          input_topic.c_str());
    }
  }

  int jpeg_quality_{95};
  int png_compression_{1};
  int log_every_{512};
  std::string codec_{"jpeg"};
  std::atomic<bool> stopping_{false};
  std::vector<std::size_t> counters_;
  std::vector<std::size_t> skipped_counters_;
  std::vector<std::unique_ptr<Slot>> slots_;
  std::vector<std::thread> workers_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr>
      publishers_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
      subscriptions_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CompressedImageEncoder>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
