//
// Created by orange on 02.03.2026.
//
#pragma once
#include <asio/post.hpp>
#include <asio/thread_pool.hpp>
#include <future>
#include <memory>
#include <thread>

namespace rose::core
{
    // Persistent thread pool backed by asio::thread_pool.
    // Workers are created once and reused every frame.
    // submit() posts work via asio::post and returns a std::future for the result.
    class ThreadPool
    {
    public:
        explicit ThreadPool(int num_threads = static_cast<int>(std::thread::hardware_concurrency()))
            : m_pool(static_cast<std::size_t>(num_threads))
            , m_size(num_threads)
        {}

        // join() drains all posted work, then joins worker threads.
        ~ThreadPool() { m_pool.join(); }

        // Non-copyable, non-movable — asio::thread_pool is non-movable.
        ThreadPool(const ThreadPool&)            = delete;
        ThreadPool& operator=(const ThreadPool&) = delete;
        ThreadPool(ThreadPool&&)                 = delete;
        ThreadPool& operator=(ThreadPool&&)      = delete;

        template<class F>
        [[nodiscard]] std::future<std::invoke_result_t<F>> submit(F&& f)
        {
            using R   = std::invoke_result_t<F>;
            auto task = std::make_shared<std::packaged_task<R()>>(std::forward<F>(f));
            auto fut  = task->get_future();
            asio::post(m_pool, [t = std::move(task)] { (*t)(); });
            return fut;
        }

        [[nodiscard]] int size() const { return m_size; }

    private:
        asio::thread_pool m_pool;
        int               m_size;
    };
} // namespace rose::core
