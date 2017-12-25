#include <opencv2/opencv.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <svo/common/frame.h>
#include <svo/frame_handler_mono.h>
#include <svo/frame_handler_stereo.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <vs_common/vs_viz3d.h>
#include "svo_factory.h"
#include "format_convert.h"

#define SUB_TOPIC_LEFT              "/cam0/image_raw"
#define SUB_TOPIC_RIGHT             "/cam1/image_raw"
svo::FrameHandlerStereo::Ptr        g_svo_ptr;
Viz3dThread                         g_viz;
std::vector<cv::Affine3f>           g_traj;

void callback(const sensor_msgs::ImageConstPtr& msg_image_left, const sensor_msgs::ImageConstPtr& msg_image_right)
{
    auto img_left_ptr = cv_bridge::toCvCopy(msg_image_left, "mono8");
    auto img_right_ptr = cv_bridge::toCvCopy(msg_image_right, "mono8");
    cv::Mat imgl = img_left_ptr->image;
    cv::Mat imgr = img_right_ptr->image;
    double ts = msg_image_left->header.stamp.toSec();

    g_svo_ptr->addImages(imgl, imgr, ts*1e9);
    svo::FrameBundlePtr last_frame = g_svo_ptr->getLastFrames();

    g_traj.push_back(svo_trans_to_cv_affine(last_frame->get_T_W_B()));
    g_viz.updateWidget("traj", cv::viz::WTrajectory(g_traj, 2, 1, cv::viz::Color::green()));

    cv::Mat show_img;
    cv::vconcat(imgl, imgr, show_img);
    cv::imshow("image",show_img);
    char key = cv::waitKey(10);
}

int main(int argc, char **argv)
{
    // init
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InstallFailureSignalHandler();
    ros::init(argc, argv, "svo");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    message_filters::Subscriber<sensor_msgs::Image> sub_img_left(nh, SUB_TOPIC_LEFT, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_img_right(nh, SUB_TOPIC_RIGHT, 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(sub_img_left, sub_img_right, 5);  //queue_size
    sync.registerCallback(boost::bind(&callback, _1, _2));

    g_viz.updateWidget("cood", cv::viz::WCoordinateSystem(1));
    
    // init svo
    g_svo_ptr = svo::factory::makeStereo(pnh);
    g_svo_ptr->start();
    ros::spin();
}
