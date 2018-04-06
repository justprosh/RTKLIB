#pragma once
#include <queue>
#include <mutex>

template <class T>
class RWQueue {
public:
    RWQueue();
    void push(T& value);
    T& front();
    T pop();
    bool isempty();
    int size();

 private:
    std::queue<T> queue_;
    std::mutex mutex_;
};

template <class T>
RWQueue<T>::RWQueue() {}
template <class T>
void RWQueue<T>::push(T& value) {
    {
        std::unique_lock<std::mutex> lock(mutex_);
        queue_.push(value);
    }
}

template <class T>
T& RWQueue<T>::front() {
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return queue_.front();
    }
}

template <class T>
T RWQueue<T>::pop() {
    {
        std::unique_lock<std::mutex> lock(mutex_);
        T elem = queue_.front();
        queue_.pop();
        return elem;
    }
}

template <class T>
bool RWQueue<T>::isempty() {
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return queue_.empty();
    }
}

template <class T>
int RWQueue<T>::size() {
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return queue_.size();
    }
}