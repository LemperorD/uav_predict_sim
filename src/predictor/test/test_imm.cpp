#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>

#include "predictor/imm.hpp"

void require(bool ok, const char *msg)
{
    if (!ok)
    {
        throw std::runtime_error(msg);
    }
}

int main()
{
    try
    {
        predictor::Imm                   filter;
        std::mt19937                     rng(7);
        std::normal_distribution<double> normal;
        double                           raw = 0, filtered = 0;
        predictor::Estimate              e;
        for (int i = 0; i < 1500; ++i)
        {
            const double           t = i * 0.02;
            predictor::Measurement z, truth;
            truth << t, 0.4 * t, 2.0, 1.0, 0.4, 0.0;
            z = truth;
            for (int k = 0; k < 6; ++k)
            {
                z(k) += (k < 3 ? 0.15 : 0.1) * normal(rng);
            }
            e = filter.update(z, 0.02);
            require(e.mean.allFinite() && e.covariance.allFinite(), "nonfinite output");
            require(std::abs(e.probability.sum() - 1) < 1e-12 && e.probability.minCoeff() > 0,
                    "invalid probabilities");
            require((e.covariance - e.covariance.transpose()).norm() < 1e-10,
                    "asymmetric covariance");
            Eigen::SelfAdjointEigenSolver<predictor::Covariance> eig(e.covariance);
            require(eig.eigenvalues().minCoeff() > -1e-10, "negative covariance");
            if (i > 100)
            {
                raw += (z.head<3>() - truth.head<3>()).squaredNorm();
                filtered += (e.mean.head<3>() - truth.head<3>()).squaredNorm();
            }
        }
        require(filtered < raw * 0.5, "filter did not reduce measurement noise");
        auto before = filter.estimate();
        auto future = filter.forecast(0.05, 20);
        require((filter.estimate().mean - before.mean).norm() == 0, "forecast changed filter");
        require(
            (future.back().mean.head<3>() - (e.mean.head<3>() + e.mean.segment<3>(3))).norm() < 0.3,
            "CV forecast inaccurate");
        predictor::Imm accelerated;
        for (int i = 0; i < 600; ++i)
        {
            double                 t = i * 0.02;
            predictor::Measurement z;
            z << 0.2 * t * t, 0, 1, 0.4 * t, 0, 0;
            accelerated.update(z, 0.02);
        }
        auto a = accelerated.estimate();
        require(a.probability(1) > 0.5, "CA model did not respond to acceleration");
        bool threw = false;
        try
        {
            filter.update(predictor::Measurement::Zero(), 0);
        }
        catch (const std::invalid_argument &)
        {
            threw = true;
        }
        require(threw, "zero dt accepted");
        threw = false;
        try
        {
            predictor::Settings s;
            s.stay_probability = 1;
            predictor::Imm bad(s);
        }
        catch (const std::invalid_argument &)
        {
            threw = true;
        }
        require(threw, "invalid transition accepted");
        threw = false;
        try
        {
            predictor::Imm fresh;
            fresh.forecast(0.1, 10);
        }
        catch (const std::invalid_argument &)
        {
            threw = true;
        }
        require(threw, "uninitialized forecast accepted");
        threw        = false;
        auto invalid = predictor::Measurement::Zero().eval();
        invalid(0)   = std::numeric_limits<double>::quiet_NaN();
        try
        {
            filter.update(invalid, 0.02);
        }
        catch (const std::invalid_argument &)
        {
            threw = true;
        }
        require(threw, "NaN measurement accepted");
        auto outlier = predictor::Measurement::Constant(1e6).eval();
        auto robust  = filter.update(outlier, 0.02);
        require(robust.mean.allFinite() && robust.probability.allFinite(),
                "outlier broke log likelihood normalization");
        std::cout << "IMM checks passed; filtered/raw squared error=" << filtered / raw
                  << " CA probability=" << a.probability(1) << '\n';
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
