#pragma once
#include <mutex>
#include <cpp_redis/cpp_redis>
#include <nlohmann/json.hpp>
#include <unordered_map>



using json = nlohmann::json;

typedef void (*re_callback_t)(json&);
typedef void (*ev_callback_t)(void);
void sigint_handler(int);

class Subscriber
{
  public:
    Subscriber();
    void run();
    void subscribe_to_channel(const std::string &channel, re_callback_t callback);
    void block(std::mutex&);

  private:
    const std::string DEFAULT_IP = "127.0.0.1";
    const int DEFAULT_PORT = 6379;
    const int TIMEOUT_MS = 1;
    const int MAX_RECONNECTS = -1;
    const int RECONNECT_INTERVAL_MS = 1;
    cpp_redis::subscriber subscriber_;
    std::unordered_map<std::string, re_callback_t> subscriptions_;
    std::unordered_map<std::string, ev_callback_t> event_subscriptions_;
};