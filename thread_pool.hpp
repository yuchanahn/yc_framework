#pragma once

#include <iostream>
#include <functional>
#include <vector>
#include <queue>

#include <thread>
#include <mutex>
#include <condition_variable>

class test_thread_pool {
private:
    size_t size;
    std::vector<std::thread> pool;

    bool end_threads;
    std::queue<std::function<void()>> jobs;

    std::mutex mutex_for_queue;
    std::condition_variable cv_for_queue;

    std::condition_variable cv_finished;

private:
    void execute_job() {
        while (true) {
            std::unique_lock<std::mutex> lock(mutex_for_queue);
            cv_for_queue.wait(lock, [this]() { return !this->jobs.empty() || this->end_threads; });

            // end thread
            if (this->end_threads && this->jobs.empty()) {
                return;
            }

            // fetch job from the queue
            auto job = std::move(jobs.front());
            jobs.pop();
            lock.unlock();

            // execute the job
            job();

            cv_finished.notify_one();
        }
    }

public:
    test_thread_pool(size_t thread_count)
      : size(thread_count), end_threads(false) {
        // create threads
        new(&pool) std::vector<std::thread>(thread_count); // placement new
        for (auto &t : pool) {
            t = std::thread([this]() {
              this->execute_job();
            });
        }
    }
    ~test_thread_pool() {
        this->end_threads = true;
        cv_for_queue.notify_all();
        for (auto &t : pool) {
            t.join();
        }
    }

    void add_task(std::function<void()> job) {
        if (this->end_threads) {
            throw std::runtime_error("ThreadPool is ended");
        }
        {
            std::lock_guard<std::mutex> lock(mutex_for_queue);
            jobs.push(std::move(job));
        }
        cv_for_queue.notify_one();
    }

    bool is_busy() {
        bool poolbusy;
        {
            std::unique_lock<std::mutex> lock(mutex_for_queue);
            poolbusy = jobs.empty();
        }
        return poolbusy;
    }

    void wait_all() {
        std::unique_lock<std::mutex> lock(mutex_for_queue);
        cv_finished.wait(lock, [this](){ return this->jobs.empty(); });
    }
};