#pragma once
#include <memory>
#include <string>
#include <vector>
namespace rclcpp {
inline bool running = true;
inline bool ok() { return running; }
inline void shutdown() { running=false; }
inline void init(int, char**) { running=true; }
struct TimerBase { using SharedPtr = std::shared_ptr<TimerBase>; };
template<class T> struct Publisher {
  using SharedPtr = std::shared_ptr<Publisher>;
  inline static std::vector<T> published;
  void publish(const T& value) { published.push_back(value); }
};
struct Node {
  explicit Node(const char*) {}
  virtual ~Node() = default;
  template<class T> auto create_publisher(const std::string&, int) {
    return std::make_shared<Publisher<T>>();
  }
  template<class Duration, class Callback> auto create_wall_timer(Duration, Callback) {
    return std::make_shared<TimerBase>();
  }
};
template<class T> void spin(T) {}
}
