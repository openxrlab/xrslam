#ifndef XRSLAM_RANDOM_H
#define XRSLAM_RANDOM_H

#include <xrslam/common.h>

namespace xrslam {

inline unsigned int get_random_seed() {
    static std::random_device rd;
    return rd();
}

class RandomBase {
  public:
    RandomBase() { seed(); }

    void seed() { engine.seed(get_random_seed()); }

    void seed(unsigned int value) { engine.seed(value); }

  protected:
    std::default_random_engine engine;
};

template <typename T> class UniformNoise : public RandomBase {
  public:
    typedef T value_type;

    UniformNoise(value_type left = value_type(0.0),
                 value_type right = value_type(1.0))
        : distribution(left, right) {}

    value_type next() { return distribution(engine); }

  private:
    std::uniform_real_distribution<value_type> distribution;
};

template <typename T> class GaussianNoise : public RandomBase {
  public:
    typedef T value_type;

    GaussianNoise(value_type sigma = value_type(1.0),
                  value_type mean = value_type(0.0))
        : distribution(mean, sigma) {}

    value_type next() { return distribution(engine); }

  private:
    std::normal_distribution<value_type> distribution;
};

template <typename T> class UniformInteger : public RandomBase {
  public:
    typedef T value_type;

    // [left, right]
    UniformInteger(value_type left = value_type(0),
                   value_type right = std::numeric_limits<T>::max())
        : distribution(left, right) {}

    void param(value_type left, value_type right) {
        distribution.param(
            std::uniform_int_distribution<value_type>::param_type(left, right));
    }

    value_type next() { return distribution(engine); }

    value_type next(value_type left, value_type right) {
        return distribution(
            engine,
            typename std::uniform_int_distribution<value_type>::param_type(
                left, right));
    }

  private:
    std::uniform_int_distribution<value_type> distribution;
};

class LotBox {
  public:
    LotBox(size_t size) : cap(0), lots(size) {
        std::iota(lots.begin(), lots.end(), 0);
    }

    size_t draw_with_replacement() {
        size_t result = draw_without_replacement();
        refill_last();
        return result;
    }

    size_t draw_without_replacement() {
        if (remaining() > 1) {
            std::swap(lots[cap], lots[dice.next(cap, lots.size() - 1)]);
            size_t result = lots[cap];
            cap++;
            return result;
        } else if (remaining() == 1) {
            cap++;
            return lots.back();
        } else {
            return size_t(-1); // Hey we have nothing left!
        }
    }

    void refill_last(size_t n = 1) {
        if (cap > n) {
            cap -= n;
        } else {
            cap = 0;
        }
    }

    void refill_all() { cap = 0; }

    size_t remaining() const { return lots.size() - cap; }

    void seed() { dice.seed(get_random_seed()); }

    void seed(unsigned int value) { dice.seed(value); }

  private:
    size_t cap;
    std::vector<size_t> lots;
    UniformInteger<size_t> dice;
};

template <typename T> class WhiteNoise {
  public:
    typedef T value_type;
    WhiteNoise(value_type sigma, value_type freq = 1, value_type mean = 0)
        : n(sigma, mean), isdt(sqrt(freq)) {}

    void seed() { n.seed(); }

    void seed(unsigned int value) { n.seed(value); }

    value_type next() { return n.next() * isdt; }

  private:
    GaussianNoise<value_type> n;
    double isdt;
};

template <typename T> class RandomWalk {
  public:
    typedef T value_type;

    RandomWalk(value_type sigma, value_type freq = 1, value_type init = 0,
               value_type bias = 0)
        : v(init), n(sigma, bias), sdt(sqrt(1 / freq)) {}

    void seed() { n.seed(); }

    void seed(unsigned int value) { n.seed(value); }

    void init(const value_type &v) { this->v = v; }

    value_type next() {
        step();
        return v;
    }

  private:
    void step() { v += n.next() * sdt; }

    double v;
    GaussianNoise<value_type> n;
    double sdt;
};

} // namespace xrslam

#endif // XRSLAM_RANDOM_H
