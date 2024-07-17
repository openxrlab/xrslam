#ifndef __OPTIMIZER_COST_FUNCTION_H__
#define __OPTIMIZER_COST_FUNCTION_H__

#include <xrslam/common.h>

namespace xrslam{

class LossFunction{
public:
    virtual inline double Compute(double squared_residual) const = 0;
    virtual inline double ComputeWeight(double squared_residual) const = 0;
    virtual inline double ComputeDerivative(double squared_residual) const = 0;
    virtual inline double ComputeSecondDerivative(double squared_residual) const = 0;
};

class TrivialLoss : public LossFunction{
public:
    inline double Compute(double squared_residual) const override{
        return squared_residual;
    }

    inline double ComputeWeight(double squared_residual) const override{
        return 1.0;
    }

    inline double ComputeDerivative(double squared_residual) const override{
        return 1.0;
    }

    inline double ComputeSecondDerivative(double squared_residual) const override{
        return 0.0;
    }
};



class HuberLoss: public LossFunction{
public:
    HuberLoss(double c) : c_(c) {}

    inline double Compute(double squared_residual) const override{
        return squared_residual < c_ ? 0.5 * squared_residual : c_ * (sqrt(squared_residual) - 0.5 * c_);
    }

    inline double ComputeWeight(double squared_residual) const override{
        return squared_residual < c_ ? 1.0 : c_ / sqrt(squared_residual);
    }

    inline double ComputeDerivative(double squared_residual) const override{
        return squared_residual < c_ ? 1.0 : 0.5 * c_ / sqrt(squared_residual);
    }

    inline double ComputeSecondDerivative(double squared_residual) const override{
        return squared_residual < c_ ? 0.0 : -0.25 * c_ / (squared_residual * sqrt(squared_residual));
    }

private:
    double c_;
};


class CauchyLoss: public LossFunction{
public:
    CauchyLoss(double c) : c_(c) {}

    inline double Compute(double squared_residual) const override{
        return c_ * c_ * std::log(1.0 + squared_residual / (c_ * c_));
    }

    inline double ComputeWeight(double squared_residual) const override{
        return c_ * c_ / (1.0 + squared_residual / (c_ * c_));
    }

    inline double ComputeDerivative(double squared_residual) const override{
        return 1.0 / (1.0 + squared_residual / (c_ * c_));
    }

    inline double ComputeSecondDerivative(double squared_residual) const override{
        double tmp = squared_residual / (c_ * c_);
        return -1.0 / (c_ * c_ * (1.0 + tmp) * (1.0 + tmp));
    }

private:
    double c_;
};

} // namespace xrslam

#endif