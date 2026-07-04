/* Author: Servet Bora Bayraktar */

#include <ompl/base/spaces/FactoredStateSpace.h>
#include <cmath>


void ompl::base::FactoredStateSpace::markAngleDim(unsigned int index)
{
    if (angleDim_.size() <= index)
        angleDim_.resize(index + 1, false);
    angleDim_[index] = true;
}

bool ompl::base::FactoredStateSpace::isAngleDim(unsigned int index) const
{
    return index < angleDim_.size() && angleDim_[index];
}

// Per-dimension distance: shortest arc over 2*pi for angular DOF, |diff| otherwise.
double ompl::base::FactoredStateSpace::dimDistance(unsigned int index, double v1, double v2) const
{
    if (isAngleDim(index))
    {
        double d = std::fmod(std::fabs(v1 - v2), 2.0 * M_PI);
        return (d > M_PI) ? (2.0 * M_PI - d) : d;
    }
    return std::fabs(v1 - v2);
}

int ompl::base::FactoredStateSpace::findIndex(std::vector<double> &distances, double t) const
{
    double sum = 0.0;
    for (size_t i = 0; i < grouped_indices.size(); i++)
    {
        sum += distances.at(i);
        if (sum >= t)
        {
            return i;
        }
    }
    return (grouped_indices.size() - 1);
}

std::vector<double> ompl::base::FactoredStateSpace::getDistances(const FactoredStateSpace::StateType *const rfrom,
                                 const FactoredStateSpace::StateType *const rto) const
{
    std::vector<double> distances_;
    double total_dist = 0.0;
    for (auto const &group : grouped_indices)
    {
        double dist_group = 0.0;
        for (auto const &index : group)
        {
            double dj = dimDistance(index, rfrom->values[index], rto->values[index]);
            total_dist += dj;
            dist_group += dj;
        }
        distances_.push_back(dist_group);
    }

    for (size_t j = 0; j < distances_.size(); j++)
    {
        if (distances_[j] > 1e-10 && !std::isnan(total_dist))
            distances_[j] /= total_dist;
        else
            distances_[j] = 0.0;
    }
    return distances_;
}

double ompl::base::FactoredStateSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const
{
    const auto &s1 = state1->as<StateType>();
    const auto &s2 = state2->as<StateType>();
    double d = 0.0;

    for (size_t i = 0; i < dimension_; i++)
    {
        d += dimDistance(i, s1->values[i], s2->values[i]);
    }

    return d;
}

void ompl::base::FactoredStateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, double t,
                 ompl::base::State *state) const
{
    const auto &rfrom = from->as<FactoredStateSpace::StateType>();
    const auto &rto = to->as<FactoredStateSpace::StateType>();
    auto *rstate = state->as<FactoredStateSpace::StateType>();

    if (t >= 1)
    {
        for (size_t i = 0; i < dimension_; i++)
        {
            rstate->values[i] = rto->values[i];
        }
    }
    else
    {
        std::vector<double> distances =
            getDistances(rfrom, rto);  

        int index = findIndex(distances, t);
        double d_interpolated = 0.0;

        for (int i = 0; i < index; i++)
        {
            for (auto const &index_in_group : grouped_indices.at(i))
            {
                rstate->values[index_in_group] = rto->values[index_in_group];
            }
            d_interpolated += distances.at(i);
        }

        double s = 0.0;
        if (std::fabs(distances.at(index)) > 1e-10)
        {
            s = (t - d_interpolated) / distances.at(index);
        }

        for (auto const &index_in_group : grouped_indices.at(index))
        {
            const double f = rfrom->values[index_in_group];
            const double to_v = rto->values[index_in_group];
            if (isAngleDim(index_in_group))
            {
                // interpolate along the shortest arc (SO(2)), then wrap to [-pi, pi]
                double diff = std::fmod(to_v - f, 2.0 * M_PI);
                if (diff > M_PI)  diff -= 2.0 * M_PI;
                if (diff < -M_PI) diff += 2.0 * M_PI;
                double v = f + s * diff;
                v = std::fmod(v + M_PI, 2.0 * M_PI);
                if (v < 0) v += 2.0 * M_PI;
                rstate->values[index_in_group] = v - M_PI;
            }
            else
            {
                rstate->values[index_in_group] = f + s * (to_v - f);
            }
        }

        for (size_t i = index + 1; i < grouped_indices.size(); i++)
        {
            for (auto const &index_in_group : grouped_indices.at(i))
            {
                rstate->values[index_in_group] = rfrom->values[index_in_group];
            }
        }
    }
}
