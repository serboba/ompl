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

#ifndef ADAPTIVESPHERE_H
#define ADAPTIVESPHERE_H

namespace ompl
{
    class AdaptiveSphere
    {
    public:
        AdaptiveSphere(base::SpaceInformationPtr si, double startRadius, double minRadius,
                       double minExpectedValidityRate,
                       double maxExpectedValidityRate, double shrinkStep, double growStep) : startRadius_(startRadius),
            minRadius_(minRadius), minExpectedValidityRate_(minExpectedValidityRate),
            maxExpectedValidityRate_(maxExpectedValidityRate),
            shrinkStep_(shrinkStep), growStep_(growStep), si_(si)
        {
        }

        // the entire logic in a single method or a few smaller steps
        double computeValidityRate(ompl::HammersleySphere* sphere)
        {
            std::vector<double> sample_reals_ = std::vector<double>(6, 0.0);
            std::vector<int> validSampleIndices;

            base::State* sample_state = si_->allocState();
            base::State* origin_state = si_->allocState();
            std::vector<double> origin_reals_ = std::vector<double>(6, 0.0);
            si_->getStateSpace()->copyFromReals(origin_state, origin_reals_);

            for (size_t i = 0; i < sphere->spherePoints.size(); i++)
            {
                auto sample = sphere->getSample();
                si_->getStateSpace()->copyFromReals(sample_state, {
                                                        0.0, 0.0, 0.0, sample.second.x, sample.second.y, sample.second.z
                                                    });

                bool validFlag = si_->checkMotion(origin_state, sample_state);
                // saveSampleInDB(sample_state, validFlag, true);

                sphere->popIndexFromIndices(sample.first, validFlag);
            }
            si_->getStateSpace()->enforceBounds(sample_state);

            si_->freeState(sample_state);
            si_->freeState(origin_state);

            return sphere->getValidSampleRate();
        }


        void setupAdaptiveSphereSampling(HammersleySphere* sphere)
        {
            int selectedSamplerArm = 0;
            double current_radius = startRadius_;
            double validity_rate;
            auto start_time = std::chrono::high_resolution_clock::now();


            int maxSteps = 10; // TOOD limit steps to avoid infinite run in case object is totally free anyway
            int currentStep = 0;
            while (current_radius > minRadius_)
            {
                // Collect Quasirandom samples
                sphere->collectQuasiRandomSamples(current_radius);

                // Compute the validity rate at the current radius
                validity_rate = computeValidityRate(sphere);

                //        std::cout << "Calculated validity rate : " << validity_rate << std::endl;

                // Check the validity rate range
                if ((validity_rate > minExpectedValidityRate_ &&
                    validity_rate <= maxExpectedValidityRate_) || (currentStep >= maxSteps))
                {
                    auto end_time = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double> elapsed = end_time - start_time;

                    //            std::cout << "Finished at step : " << currentStep << std::endl;
                    sphere->appendCachedValidSamples();
                    break; // Found a suitable radius
                }

                // Adjust the radius based on the validity rate
                if (validity_rate < minExpectedValidityRate_)
                {
                    // If validity rate is 0%, decrease the radius
                    current_radius *= std::exp(-shrinkStep_);
                    sphere->appendCachedValidSamples();

                    // std::cout << "Validity rate 0.0 decreasing radius to :" << current_radius << std::endl;
                }
                else if (validity_rate > maxExpectedValidityRate_)
                {
                    // If validity rate is over 50%, increase the radius
                    current_radius *= std::exp(growStep_);
                    sphere->allPoints_cached.clear();

                    // std::cout << "Validity rate over threshold increasing radius to :" << current_radius << std::endl;
                }
                currentStep++;
            }

            sphere->bestRadius = current_radius;
        }

    private:
        // references or pointers to your sphere
        double currentRadius_;
        double startRadius_;
        double minRadius_;
        double minExpectedValidityRate_;
        double maxExpectedValidityRate_;
        double shrinkStep_;
        double growStep_;

        ompl::base::SpaceInformationPtr si_;
    };
}


#endif //ADAPTIVESPHERE_H
