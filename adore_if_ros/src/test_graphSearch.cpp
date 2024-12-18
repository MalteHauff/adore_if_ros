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
#include <nav_msgs/OccupancyGrid.h>
#include <stdint.h>
#include <adore/apps/gap_provider.h>
#include <adore/apps/graph_search.h>
#include <adore_if_ros/factorycollection.h>
#include <geometry_msgs/Pose.h>

namespace adore
{
    namespace if_ROS
    {
    class GraphSearchNode : public FactoryCollection, public adore_if_ros_scheduling::Baseapp {
        public:
            adore::apps::GraphSearch* gs_;
            //ros::NodeHandle node;
            ros::Subscriber sub;
            GraphSearchNode() {}
            bool validStart, validEnd;
            void init(int argc, char **argv, double rate, std::string nodename)
            {
                adore_if_ros_scheduling::Baseapp::init(argc, argv, rate, nodename);
                adore_if_ros_scheduling::Baseapp::initSim();
                FactoryCollection::init(getRosNodeHandle());
                //ros::NodeHandle node;
                sub = getRosNodeHandle()->subscribe<nav_msgs::OccupancyGrid>("map",10, &GraphSearchNode::receive_map_data, this);

                first_set = false;
                std::cout<<"init graph search node"<<std::endl;
            }
        private:
            bool first_set;
            void receive_map_data(const nav_msgs::OccupancyGrid::ConstPtr &msg){
                std::cout<<"receive map data"<<std::endl;
                if(!first_set){
                    std::cout<<"receive map data first time"<<std::endl;
                    first_set=true;
                    int test=1;
                    gs_=  new adore::apps::GraphSearch(msg, test, (uint32_t)(msg->info.height), (uint32_t)(msg->info.width), Baseapp::getRosNodeHandle()); //->init_gs(data,1,1);//(new_data, (uint32_t)(msg->info.height), (uint32_t)(msg->info.width));//, msg->data, (uint32_t)msg->info.height, (uint32_t)msg->info.width)
                    // timer callbacks
                    std::function<void()> run_fcn(std::bind(&adore::apps::GraphSearch::update, gs_));
                    adore_if_ros_scheduling::Baseapp::addTimerCallback(run_fcn);

                }
                
            }
             void receiveStartPose(geometry_msgs::Pose msg)
            {
                        double r,p,y;
                        //tf::Matrix3x3(tf::Quaternion(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)).getRPY(r,p,y);
                        validStart = true;//Start.setPosition(msg.position.x,msg.position.y,y,Width,Length,Depth,adore::mad::CoordinateConversion::DegToRad(HeadingResolution), figure3);
                        //Start.print();
            }  
            void receiveEndPose(geometry_msgs::Pose msg)
            {
                        double r,p,y;
                        //tf::Matrix3x3(tf::Quaternion(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)).getRPY(r,p,y);
                        validEnd = true; //End.setPosition(msg.position.x,msg.position.y,y,Width,Length,Depth, adore::mad::CoordinateConversion::DegToRad(HeadingResolution),  figure3);
                        //End.print();
            }     
                
        };
    } // namespace if_ROS
} // namespace adore

int main(int argc, char **argv)
{
    adore::if_ROS::GraphSearchNode node;
    node.init(argc, argv, 10.0, "GraphSearchNode");   
    node.run();
    return 0;
}

