#ifndef XRSLAM_UNIQUE_TIMER_H
#define XRSLAM_UNIQUE_TIMER_H

#include <xrslam/common.h>

#define debug_timer(name, message)                                             \
    auto name##_timer = make_timer([](double t) {                              \
        log_message(XRSLAM_LOG_DEBUG, "T(%s) = %.6fms", message, t * 1e3);     \
    })

namespace xrslam {

template <typename _Out, typename _Dur = std::chrono::duration<double>,
          typename _Clk = std::chrono::steady_clock>
class unique_timer {
  public:
    typedef _Clk clock_type;
    typedef _Dur duration_type;

    unique_timer(_Out out) : out(out) {}
    ~unique_timer() { end(); }

    unique_timer &operator=(unique_timer &&right) {
        if (this != &right) {
            time = right.time;
            right.valid = false;
        }
    }

    unique_timer(unique_timer &&right) : time(right.time), out(right.out) {
        right.valid = false;
    }

    typename duration_type::rep duration() {
        return std::chrono::duration_cast<duration_type>(clock_type::now() -
                                                         time)
            .count();
    }

    void end() {
        if (valid) {
            out(duration());
            valid = false;
        }
    }

    unique_timer &operator=(const unique_timer &right) = delete;
    unique_timer(const unique_timer &right) = delete;

  private:
    bool valid = true;
    typename _Clk::time_point time = _Clk::now();
    _Out out;
};

template <typename _Dur = std::chrono::duration<float>,
          typename _Clk = std::chrono::steady_clock, typename _Out>
unique_timer<_Out, _Dur, _Clk> make_timer(_Out out) {
    return unique_timer<_Out, _Dur, _Clk>(out);
}

} // namespace xrslam

#endif // XRSLAM_UNIQUE_TIMER_H
