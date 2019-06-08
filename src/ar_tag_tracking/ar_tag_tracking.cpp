/*
 * ar_tag_tracking.cpp
 * this node subscribers to transforms from ar tag tracking and publishes the filtered poses of the target position in manipulator frame
 *
 *  Created on:  Jun 07, 2019
 *
 *      
 *      
*/

#include "ros/ros.h"
#include <vector>
#include <numeric> 
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

namespace ar_tag_tracking
{

enum class ARTagTrackingState
{
    OFF,
    INITIALIZE,
    SAMPLING,
    COMPLETED
};  

class ARTagTracking
{
public:
    ARTagTracking( ros::NodeHandle& nh ):
        nh_( std::make_shared<ros::NodeHandle>( nh ) ),
        calculate_target_pose_( false ),
        ar_tag_tracking_state_( ARTagTrackingState::OFF ),
        start_time_( ros::Time::now() ),
        calculation_duration_( ros::Duration( 5.0) )
    {
        // Get params
        nh_->getParam("robot_tag_link", robot_tag_link_);
        nh_->getParam("manipulator_tag_link", manipulator_tag_link_);

        ROS_INFO("manipulator side frame %s, robot side frame %s ", manipulator_tag_link_.c_str(), robot_tag_link_.c_str() );

        // target publisher
        target_pose_pub_ = nh_->advertise<geometry_msgs::PoseStamped>( "/target_pose", 1 );

        // command subscriber
        command_sub_ = nh_->subscribe( "/command", 10, &ARTagTracking::commandCallback, this );
    }

    ~ARTagTracking(){};

    void commandCallback( const std_msgs::String::ConstPtr & command )
    {
        if( command->data == "calculate_target_pose" )
        {
            calculate_target_pose_ = true;       
        }    
    }

    void spin()
    {

    switch( ar_tag_tracking_state_ )
    {
        case ARTagTrackingState::OFF:   // OFF state
            if( calculate_target_pose_ )
            {
                ROS_INFO( "[AR Tag Tracking] tracking AR tags" );
                ar_tag_tracking_state_ = ARTagTrackingState::INITIALIZE;
                break;
            }
            break;
        case ARTagTrackingState::INITIALIZE:   // OFF state
            ROS_INFO( "[AR Tag Tracking] collecting target poses" );
            // initialize 2D vector to accumulate poses 
            for( int i = 0; i < 7; i++)
            {
                std::vector<double> v;
                v.push_back( 0.0 );
                recorded_poses_.push_back( v );
            }
            start_time_ = ros::Time::now();
            ar_tag_tracking_state_ = ARTagTrackingState::SAMPLING;
            break;
        case ARTagTrackingState::SAMPLING:   // take samples and accumulate poses
            if( ros::Time::now() - start_time_  < calculation_duration_ )   // take samles for partucular amount of time
            {
                geometry_msgs::PoseStamped target_pose = calculateTargetPose();
                if( target_pose.header.frame_id ==  manipulator_tag_link_ )
                {
                    ROS_INFO("[AR Tag Tracking] get target position sample of ( %f, %f, %f )", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z );
                    recorded_poses_[0].push_back( target_pose.pose.position.x );
                    recorded_poses_[1].push_back( target_pose.pose.position.y );
                    recorded_poses_[2].push_back( target_pose.pose.position.z );
                    recorded_poses_[3].push_back( target_pose.pose.orientation.x );
                    recorded_poses_[4].push_back( target_pose.pose.orientation.y );
                    recorded_poses_[5].push_back( target_pose.pose.orientation.z );
                    recorded_poses_[6].push_back( target_pose.pose.orientation.w );
                }
            }
            else
            {
                ar_tag_tracking_state_ = ARTagTrackingState::COMPLETED;
            }
            break;
        case ARTagTrackingState::COMPLETED:   // calculate and publish average target pose
            ROS_INFO( "[AR Tag Tracking] calculate average target pose with %d samples", recorded_poses_[0].size() - 1 );
            geometry_msgs::PoseStamped average_target_pose;
            average_target_pose.header.stamp = ros::Time::now();
            average_target_pose.pose.position.x = accumulate( recorded_poses_[0].begin(), recorded_poses_[0].end(), 0.0 ) / ( recorded_poses_[0].size() - 1 );
            average_target_pose.pose.position.y = accumulate( recorded_poses_[1].begin(), recorded_poses_[1].end(), 0.0 ) / ( recorded_poses_[1].size() - 1 );
            average_target_pose.pose.position.z = accumulate( recorded_poses_[2].begin(), recorded_poses_[2].end(), 0.0 ) / ( recorded_poses_[2].size() - 1 );
            average_target_pose.pose.orientation.x = accumulate( recorded_poses_[3].begin(), recorded_poses_[3].end(), 0.0 ) / ( recorded_poses_[3].size() - 1 );
            average_target_pose.pose.orientation.y = accumulate( recorded_poses_[4].begin(), recorded_poses_[4].end(), 0.0 ) / ( recorded_poses_[4].size() - 1 );
            average_target_pose.pose.orientation.z = accumulate( recorded_poses_[5].begin(), recorded_poses_[5].end(), 0.0 ) / ( recorded_poses_[5].size() - 1 );
            average_target_pose.pose.orientation.w = accumulate( recorded_poses_[6].begin(), recorded_poses_[6].end(), 0.0 ) / ( recorded_poses_[6].size() - 1 );
            ROS_INFO("[AR Tag Tracking] find target position of ( %f, %f, %f ) in manipulator frame", average_target_pose.pose.position.x, average_target_pose.pose.position.y, average_target_pose.pose.position.z );
            target_pose_pub_.publish( average_target_pose );
            calculate_target_pose_ = false;
            recorded_poses_.clear();
            ar_tag_tracking_state_ = ARTagTrackingState::OFF;
            break;
        }
    }

    geometry_msgs::PoseStamped calculateTargetPose()
    {
        static ros::Time last_stamp = ros::Time::now();
        geometry_msgs::PoseStamped source_pose;    // reference pose, just for tf
        geometry_msgs::PoseStamped target_pose;

        source_pose.pose.orientation.w = 1.0;
        source_pose.header.frame_id = robot_tag_link_;
        source_pose.header.stamp = ros::Time( 0 );

        // get target pose from tf when a new transform is available && source_pose.header.stamp != last_stamp
        if( tf_listener_.canTransform( manipulator_tag_link_, robot_tag_link_, source_pose.header.stamp )  )
        {
            // get target pose in manipulator frame
            tf_listener_.transformPose( manipulator_tag_link_, source_pose, target_pose );

            last_stamp = source_pose.header.stamp;

            return target_pose;
        }
    }

private:
    // Node Handle shared pointer
    std::shared_ptr<ros::NodeHandle> nh_;
    // Publishers
    ros::Publisher target_pose_pub_;
    // Subscriber
    ros::Subscriber command_sub_;

    // calculate average pose in a partucular amount of time
    ARTagTrackingState ar_tag_tracking_state_;
    bool calculate_target_pose_;
    ros::Time start_time_;
    ros::Duration calculation_duration_;
    std::vector<std::vector<double>> recorded_poses_;

    // frames
    std::string robot_tag_link_;
    std::string manipulator_tag_link_;

    // tf listener
    tf::TransformListener tf_listener_;
};

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ar_tag_tracking");

    ros::NodeHandle nh( "ar_tag_tracking" );

    ROS_INFO("Starting ar_tag_tracker");

    ar_tag_tracking::ARTagTracking ar_tag_tracker( nh );

    ros::Rate looprate( 10 );
    ros::Time node_start_time = ros::Time::now();

    while(ros::ok())
    {
        ar_tag_tracker.spin();

        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
}