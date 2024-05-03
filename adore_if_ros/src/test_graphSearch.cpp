/********************************************************************************
 * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the 
 *
 * SPDX-License-Identifier: EPL-2.0 
 *
 * Contributors: 
 *   Reza Dariani- initial API and implementation
 ********************************************************************************/
#include <adore_if_ros_scheduling/baseapp.h>
#include <adore/apps/graph_search.h>

namespace adore
{
    namespace if_ROS
    {
    class GraphSearchNode : public adore_if_ros_scheduling::Baseapp {
        public:
            adore::apps::GraphSearch *gs_;
            GraphSearchNode() {}
            void init(int argc, char **argv, double rate, std::string nodename)
            {
                adore_if_ros_scheduling::Baseapp::init(argc, argv, rate, nodename);
                adore_if_ros_scheduling::Baseapp::initSim();
                gs_ = new adore::apps::GraphSearch(adore_if_ros_scheduling::Baseapp::getRosNodeHandle());

                // timer callbacks
                std::function<void()> run_fcn(std::bind(&adore::apps::GraphSearch::update, gs_));
                adore_if_ros_scheduling::Baseapp::addTimerCallback(run_fcn);
            }
        };
    } // namespace if_ROS
} // namespace adore

int main(int argc, char **argv)
{
    auto node = new adore::if_ROS::GraphSearchNode();
    node->init(argc, argv, 10.0, "GraphSearchNode");   
    node->run();
    return 0;
}