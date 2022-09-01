#ifndef XRSLAM_WORKER_H
#define XRSLAM_WORKER_H

#include <xrslam/common.h>

namespace xrslam {

class Worker {
  public:
    Worker() {}

    virtual ~Worker() { stop(); }

    void start() {
        worker_running = true;
#if defined(XRSLAM_ENABLE_THREADING)
        worker_thread = std::thread(&Worker::worker_loop, this);
#endif
    }

    void stop() {
        if (worker_running) {
            worker_running = false;
#if defined(XRSLAM_ENABLE_THREADING)
            worker_cv.notify_all();
            worker_thread.join();
#endif
        }
    }

    std::unique_lock<std::mutex> lock() const {
        return std::unique_lock(worker_mutex);
    }

    void resume(std::unique_lock<std::mutex> &l) {
        l.unlock();
#if defined(XRSLAM_ENABLE_THREADING)
        worker_cv.notify_all();
#else
        worker_loop();
#endif
    }

    virtual bool empty() const = 0;
    virtual void work(std::unique_lock<std::mutex> &l) = 0;

  protected:
    std::atomic<bool> worker_running;

  private:
    void worker_loop();

#if defined(XRSLAM_ENABLE_THREADING)
    std::thread worker_thread;
    std::condition_variable worker_cv;
#endif
    mutable std::mutex worker_mutex;
};

} // namespace xrslam

#endif // XRSLAM_WORKER_H
