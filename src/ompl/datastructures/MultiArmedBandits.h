/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Servet Bora Bayraktar */

#ifndef MULTIARMEDBANDITS_H
#define MULTIARMEDBANDITS_H

#include <cmath>
#include <vector>
#include <deque>
#include <random>
#include <Eigen/Dense>


namespace ompl
{
    class MultiArmedBandits
    {
    public:
        MultiArmedBandits(int numArms, unsigned int seed = std::random_device{}())
            : armCounts(numArms, 0), armRewards(numArms, 0.0)
        {
        }

        virtual int chooseArm() = 0;
        virtual void update(int arm, double reward) = 0;
        ~MultiArmedBandits() = default;

    protected:
        std::vector<int> armCounts;
        std::vector<double> armRewards;
        std::mt19937 generator; // Random number generator shared across derived classes
    };

    class MAB_SlidingWindowUCB : public MultiArmedBandits
    {
    public:
        MAB_SlidingWindowUCB(int numArms_, int windowSize_, unsigned int seed = std::random_device{}()) :
            MultiArmedBandits(numArms_, seed),
            distribution_(0.0, 1.0),
            uniform_dist_(0, numArms_ - 1),
            t(0),
            numArms(numArms_),
            windowSize(windowSize_),
            confidenceMultiplier_(1.0),  // Start with full exploration
            minExplorationProbability_(0.0)  // Default: no minimum (can be set)
        {
        }

        // Set adaptive confidence multiplier for exploration/exploitation balance
        // multiplier = 1.0: full exploration (original UCB)
        // multiplier = 0.1: 10% exploration (mostly exploitation)
        void setConfidenceMultiplier(double multiplier) {
            confidenceMultiplier_ = std::max(0.01, std::min(1.0, multiplier));  // Clamp to [0.01, 1.0]
        }

        double getConfidenceMultiplier() const {
            return confidenceMultiplier_;
        }

        // Set minimum exploration probability per arm (for probabilistic completeness)
        // e.g., 0.05 = each arm gets at least 5% of selections
        void setMinExplorationProbability(double minProb) {
            minExplorationProbability_ = std::max(0.0, std::min(1.0 / numArms, minProb));  // Clamp to [0, 1/numArms]
        }

        double getMinExplorationProbability() const {
            return minExplorationProbability_;
        }

        int chooseArm()
        {
            if (t < numArms)
            {
                //play each arm at the beginning
                return t;
            }

            // Calculate selection probabilities based on UCB
            std::vector<double> upperBounds(numArms);
            double maxUpperBound = -std::numeric_limits<double>::infinity();
            
            for (int i = 0; i < numArms; ++i)
            {
                if (armCounts[i] == 0)
                {
                    return i;
                }

                double meanReward = armRewards[i] / armCounts[i];
                // Apply adaptive confidence multiplier to reduce exploration over time
                double confidence = confidenceMultiplier_ * sqrt((2 * log(std::min(t, windowSize))) / armCounts[i]);
                double upperBound = meanReward + confidence;
                upperBounds[i] = upperBound;
                
                if (upperBound > maxUpperBound)
                {
                    maxUpperBound = upperBound;
                }
            }

            // Apply minimum exploration probability if set
            if (minExplorationProbability_ > 0.0)
            {
                // Convert UCB upper bounds to probabilities with minimum guarantee
                // Use softmax-like approach with minimum floor
                double sumExp = 0.0;
                std::vector<double> expValues(numArms);
                
                // Normalize upper bounds and apply exponential
                for (int i = 0; i < numArms; ++i)
                {
                    double normalized = (upperBounds[i] - maxUpperBound) * 10.0;  // Scale for better distribution
                    expValues[i] = std::exp(normalized);
                    sumExp += expValues[i];
                }
                
                // Convert to probabilities with minimum floor
                std::vector<double> probabilities(numArms);
                double remainingProb = 1.0 - (minExplorationProbability_ * numArms);
                remainingProb = std::max(0.0, remainingProb);  // Ensure non-negative
                
                for (int i = 0; i < numArms; ++i)
                {
                    double baseProb = expValues[i] / sumExp;
                    probabilities[i] = minExplorationProbability_ + (baseProb * remainingProb);
                }
                
                // Sample according to probabilities
                double rand = distribution_(generator);
                double cumProb = 0.0;
                for (int i = 0; i < numArms; ++i)
                {
                    cumProb += probabilities[i];
                    if (rand <= cumProb)
                    {
                        return i;
                    }
                }
                return numArms - 1;  // Fallback (shouldn't happen)
            }
            else
            {
                // Standard UCB selection (no minimum exploration)
                // maxUpperBound already calculated above, just find the arm with max value
                int selectedArm = 0;
                for (int i = 1; i < numArms; ++i)
                {
                    if (upperBounds[i] > upperBounds[selectedArm])
                    {
                        selectedArm = i;
                    }
                }
                return selectedArm;
            }
        }

        void update(int arm, double reward)
        {
            t++;

            // Add new reward to history
            history_.push_back({arm, reward});
            armRewards[arm] += reward;
            armCounts[arm]++;

            // Remove expired reward if window size exceeded
            if (history_.size() > static_cast<size_t>(windowSize))
            {
                std::pair<int, double> expired = history_.front();
                history_.pop_front();
                
                int expiredArm = expired.first;
                double expiredReward = expired.second;
                
                armRewards[expiredArm] -= expiredReward;
                armCounts[expiredArm]--;
                
                // Safety check to prevent negative counts/rewards due to floating point errors
                if (armCounts[expiredArm] < 0) armCounts[expiredArm] = 0;
                if (armCounts[expiredArm] == 0) armRewards[expiredArm] = 0.0;
            }
        }

    private:
        std::uniform_real_distribution<double> distribution_;
        std::uniform_int_distribution<int> uniform_dist_;

        int t;
        int numArms;
        int windowSize;
        double confidenceMultiplier_;  // Adaptive exploration parameter (1.0 = full exploration, 0.05 = 5% exploration)
        double minExplorationProbability_;  // Minimum probability per arm (for probabilistic completeness, e.g., 0.05 = 5%)
        
        // History of (arm, reward) pairs for sliding window
        std::deque<std::pair<int, double>> history_;
    };


    class MAB_UCB1 : public MultiArmedBandits
    {
    public:
        MAB_UCB1(int numArms_, unsigned int seed = std::random_device{}())
            : MultiArmedBandits(numArms_, seed),distribution_(0.0, 1.0),
            uniform_dist_(0, numArms_ - 1),
            t(0),
            numArms(numArms_)
        {
        }

        // Choose an arm based on the Upper Confidence Bound formula
        int chooseArm()
        {
            if (t < numArms)
            {
                // Ensure each arm is played at least once
                return t;
            }

            double maxUpperBound = -std::numeric_limits<double>::infinity();
            int selectedArm = 0;

            for (int i = 0; i < numArms; ++i)
            {
                if (armCounts[i] == 0)
                {
                    return i;
                }

                double meanReward = armRewards[i] / armCounts[i];
                double confidence = sqrt((2 * log(t)) / armCounts[i]);
                double upperBound = meanReward + confidence;

                if (upperBound > maxUpperBound)
                {
                    maxUpperBound = upperBound;
                    selectedArm = i;
                }
            }

            return selectedArm;
        }

        // Update arm statistics
        void update(int arm, double reward)
        {
            armRewards[arm] += reward;
            armCounts[arm]++;
            t++; // Increment time step
        }

    private:
        std::uniform_real_distribution<double> distribution_;
        std::uniform_int_distribution<int> uniform_dist_;

        int t;
        int numArms;
        int windowSize;
    };
};


#endif //MULTIARMEDBANDITS_H
