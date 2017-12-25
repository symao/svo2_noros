#include <opencv2/opencv.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <svo/frame_handler_mono.h>
#include <cv_bridge/cv_bridge.h>
#include <vs_common/vs_viz3d.h>
#include "svo_factory.h"
#include "format_convert.h"

#define SUB_TOPIC_IMAGE             "/camera/image_raw"
svo::FrameHandlerMono::Ptr          g_svo_ptr;
Viz3dThread                         g_viz;
std::vector<cv::Affine3f>           g_traj;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    auto img_ptr = cv_bridge::toCvCopy(img_msg, "mono8");
    cv::Mat img = img_ptr->image;
    double ts = img_msg->header.stamp.toSec();

    g_svo_ptr->addImage(img, ts*1e9);
    svo::FrameBundlePtr last_frame = g_svo_ptr->getLastFrames();

    g_traj.push_back(svo_trans_to_cv_affine(last_frame->get_T_W_B()));
    g_viz.updateWidget("traj", cv::viz::WTrajectory(g_traj, 2, 1, cv::viz::Color::green()));

    cv::imshow("image",img);
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
    ros::Subscriber sub_img = nh.subscribe(SUB_TOPIC_IMAGE, 1, img_callback);

    g_viz.updateWidget("cood", cv::viz::WCoordinateSystem(1));

    // init svo
    g_svo_ptr = svo::factory::makeMono(pnh);
    g_svo_ptr->start();
    ros::spin();
}
