//
// Created by orange on 02.03.2026.
//
#pragma once
#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace rose::core
{
    // Minimal persistent thread pool.
    // Workers block on a condition variable between tasks; no busy-waiting.
    // Usage: call submit(callable) to enqueue work; await via the returned future.
    class ThreadPool
    {
    public:
        explicit ThreadPool(int num_threads = static_cast<int>(std::thread::hardware_concurrency()))
        {
            m_workers.reserve(num_threads);
            for (int i = 0; i < num_threads; ++i)
                m_workers.emplace_back([this] { worker_loop(); });
        }

        ~ThreadPool()
        {
            {
                std::scoped_lock lock(m_mutex);
                m_stop = true;
            }
            m_cv.notify_all();
            for (auto& t : m_workers)
                t.join();
        }

        // Non-copyable, non-movable — worker threads hold a pointer to *this.
        ThreadPool(const ThreadPool&)            = delete;
        ThreadPool& operator=(const ThreadPool&) = delete;
        ThreadPool(ThreadPool&&)                 = delete;
        ThreadPool& operator=(ThreadPool&&)      = delete;

        // Enqueue a callable and return a future for its result.
        template<class F>
        [[nodiscard]] std::future<std::invoke_result_t<F>> submit(F&& f)
        {
            using R = std::invoke_result_t<F>;
            auto task = std::make_shared<std::packaged_task<R()>>(std::forward<F>(f));
            auto fut  = task->get_future();
            {
                std::scoped_lock lock(m_mutex);
                m_tasks.emplace([t = std::move(task)] { (*t)(); });
            }
            m_cv.notify_one();
            return fut;
        }

        [[nodiscard]] int size() const { return static_cast<int>(m_workers.size()); }

    private:
        void worker_loop()
        {
            while (true)
            {
                std::function<void()> task;
                {
                    std::unique_lock lock(m_mutex);
                    m_cv.wait(lock, [this] { return m_stop || !m_tasks.empty(); });
                    if (m_stop && m_tasks.empty())
                        return;
                    task = std::move(m_tasks.front());
                    m_tasks.pop();
                }
                task();
            }
        }

        std::vector<std::thread>          m_workers;
        std::queue<std::function<void()>> m_tasks;
        std::mutex                        m_mutex;
        std::condition_variable           m_cv;
        bool                              m_stop = false;
    };
} // namespace rose::core
