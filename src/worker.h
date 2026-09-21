#pragma once
#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>

namespace m2m {
// A persistent owner for sequential jobs; destruction drains and joins it.
class Worker {
    std::mutex mutex;
    std::condition_variable ready;
    std::queue<std::function<void()>> jobs;
    bool stopping=false;
    std::thread thread;
public:
    Worker():thread([this] {
        for(;;) {
            std::function<void()> job;
            {
                std::unique_lock<std::mutex> lock(mutex);
                ready.wait(lock,[this]{return stopping || !jobs.empty();});
                if(jobs.empty()) return;
                job=std::move(jobs.front()); jobs.pop();
            }
            job();
        }
    }) {}
    ~Worker() {
        { std::lock_guard<std::mutex> lock(mutex); stopping=true; }
        ready.notify_one(); thread.join();
    }
    template<class F> auto submit(F fn) {
        using R=decltype(fn());
        auto task=std::make_shared<std::packaged_task<R()>>(std::move(fn));
        auto result=task->get_future();
        { std::lock_guard<std::mutex> lock(mutex); jobs.push([task]{(*task)();}); }
        ready.notify_one(); return result;
    }
};
}
