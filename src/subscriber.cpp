#include "subscriber.hpp"


Subscriber::Subscriber() {}

void Subscriber::run()
{
    subscriber_.connect(DEFAULT_IP, DEFAULT_PORT, [](const std::string &host, std::size_t port,
                        cpp_redis::subscriber::connect_state status) {
        if (status == cpp_redis::subscriber::connect_state::dropped)
        {
            std::cout << "client disconnected from " << host << ":" << port << std::endl
                        << "trying to reconnect" << std::endl;
        }
    }, TIMEOUT_MS, MAX_RECONNECTS, RECONNECT_INTERVAL_MS);
}

void Subscriber::subscribe_to_channel(const std::string &channel, re_callback_t callback)
{
    subscriptions_[channel] = callback;
    subscriber_.subscribe(channel, [&](const std::string &channel, const std::string msg) {
        json data;
        try
        {
            data = json::parse(msg);
        }
        catch (json::parse_error& e)
        {
            // output exception information
            // don't reset callback function if json is not valid
            std::cerr << "message: " << e.what() << '\n'
                      << "exception id: " << e.id << '\n'
                      << "byte position of error: " << e.byte << std::endl;
            return;
        }
        (subscriptions_[channel])(data);
    });

    subscriber_.commit();
}

void Subscriber::block(std::mutex &mtx){
    mtx.lock();
}

