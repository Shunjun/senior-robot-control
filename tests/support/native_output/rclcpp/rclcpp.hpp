#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
namespace rclcpp {
struct MessageInfo {
  struct Rmw {struct Gid {uint8_t data[24]{};} publisher_gid;} info;
  const Rmw & get_rmw_message_info() const {return info;}
};
inline bool running = true;
inline bool ok() { return running; }
inline void shutdown() { running=false; }
inline void init(int, char**) { running=true; }
struct TimerBase { using SharedPtr = std::shared_ptr<TimerBase>; };
template<class T> struct Publisher {
  using SharedPtr = std::shared_ptr<Publisher>;
  inline static std::vector<T> published;
  inline static std::vector<std::string> topics;
  void publish(const T& value) { published.push_back(value); }
};
struct Node {
  explicit Node(const char*) {}
  virtual ~Node() = default;
  template<class T> auto create_publisher(const std::string& topic, int) {
    Publisher<T>::topics.push_back(topic);
    return std::make_shared<Publisher<T>>();
  }
  template<class Duration, class Callback> auto create_wall_timer(Duration, Callback) {
    return std::make_shared<TimerBase>();
  }
};
template<class T> void spin(T) {}
}
