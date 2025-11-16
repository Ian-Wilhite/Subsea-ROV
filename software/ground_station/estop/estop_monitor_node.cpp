#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

namespace
{

constexpr auto kDefaultGpioPath = "/sys/class/gpio/gpio23/value";
constexpr std::chrono::microseconds kDefaultPollPeriod{500};

timespec ToTimespec(std::chrono::nanoseconds period)
{
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(period);
  const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
    period - seconds);
  timespec ts{};
  ts.tv_sec = seconds.count();
  ts.tv_nsec = nanoseconds.count();
  return ts;
}

class RealtimeGpioPoller
{
public:
  using Callback = std::function<void(bool)>;

  RealtimeGpioPoller(std::string path, std::chrono::nanoseconds period, Callback cb)
  : path_(std::move(path)),
    period_(period),
    callback_(std::move(cb)),
    running_(true),
    worker_(&RealtimeGpioPoller::run, this)
  {
  }

  ~RealtimeGpioPoller()
  {
    stop();
  }

  void stop()
  {
    bool expected = true;
    if (running_.compare_exchange_strong(expected, false, std::memory_order_release)) {
      if (worker_.joinable()) {
        worker_.join();
      }
    }
  }

private:
  void run()
  {
    const int fd = ::open(path_.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
      throw std::runtime_error(
              "Failed to open GPIO value file at " + path_ + ": " + std::strerror(errno));
    }

    const timespec sleep_ts = ToTimespec(period_);
    while (running_.load(std::memory_order_acquire)) {
      char value = '0';
      const auto read_bytes = ::pread(fd, &value, 1, 0);
      if (read_bytes == 1) {
        callback_(value == '1');
      }
      clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_ts, nullptr);
    }

    ::close(fd);
  }

  std::string path_;
  std::chrono::nanoseconds period_;
  Callback callback_;
  std::atomic<bool> running_;
  std::thread worker_;
};

class EstopMonitorNode : public rclcpp::Node
{
public:
  explicit EstopMonitorNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("estop_monitor", options)
  {
    const auto gpio_path = declare_parameter<std::string>("gpio_value_path", kDefaultGpioPath);
    const auto poll_period_us =
      declare_parameter<int>("poll_period_us", static_cast<int>(kDefaultPollPeriod.count()));

    poller_ = std::make_unique<RealtimeGpioPoller>(
      gpio_path, std::chrono::microseconds(poll_period_us),
      [this](bool active) { last_sample_.store(active, std::memory_order_release); });
  }

  ~EstopMonitorNode() override
  {
    if (poller_) {
      poller_->stop();
    }
  }

  bool latest_state() const
  {
    return last_sample_.load(std::memory_order_acquire);
  }

private:
  std::atomic<bool> last_sample_{false};
  std::unique_ptr<RealtimeGpioPoller> poller_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::InitOptions init_options;
  init_options.use_default_signal_handlers(false);
  rclcpp::init(argc, argv, init_options);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  auto node = std::make_shared<EstopMonitorNode>(rclcpp::NodeOptions().use_intra_process_comms(true));
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
