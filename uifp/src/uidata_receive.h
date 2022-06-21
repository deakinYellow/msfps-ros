#ifndef UIDATA_RECEIVE_H
#define UIDATA_RECEIVE_H

#include "../../common/utility-cc/stool.hpp"

#include <sensor_msgs/Imu.h>
#include "nlink_parser/LinktrackNodeframe2.h"
#include "../../mros/parameter.hpp"
#include "uidata_manager.h"

namespace uifp {

class UiDataReceive
{
public:
  explicit UiDataReceive( ros::NodeHandlePtr nh_p,
                          std::string tag_sub_topic,
                          std::string imu_sub_topic,
                          UiDataManager* data_manager )
    : nh_(nh_p),
      tag_sub_topic_( tag_sub_topic ),
      imu_sub_topic_( imu_sub_topic ),
      data_manager_( data_manager ) {

    //init subscribe
    tag_subscriber_ = nh_->subscribe( tag_sub_topic_, 10,
                                        &UiDataReceive::tag_callback, this );
    imu_subscriber_ = nh_->subscribe( imu_sub_topic_, 10,
                                        &UiDataReceive::imu_callback, this );

  }
private:
  ros::NodeHandlePtr nh_;
  std::string  tag_sub_topic_;
  std::string  imu_sub_topic_;
  ros::Subscriber tag_subscriber_;
  ros::Subscriber imu_subscriber_;

  UiDataManager*     data_manager_;

  stool::FPS tag_callback_fps_;
  stool::FPS imu_callback_fps_;

  //callback function
  void tag_callback( const nlink_parser::LinktrackNodeframe2::ConstPtr msg );
  void imu_callback( const sensor_msgs::Imu::ConstPtr msg );

};

}  //end of uifp

#endif // UIDATA_RECEIVE_H

