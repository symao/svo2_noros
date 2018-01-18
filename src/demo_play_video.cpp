#include <opencv2/opencv.hpp>
// svo headers
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <svo/common/frame.h>
#include <svo/frame_handler_mono.h>
#include <svo/frame_handler_stereo.h>
// vs_common headers
#include <vs_common/vs_viz3d.h>
#include <vs_common/vs_tictoc.h>
// local headers
#include "svo_factory.h"
#include "format_convert.h"

#define DATASET5

// sf outside 1A 1B 1C, forward
#ifdef DATASET1
std::string video_file = "/home/symao/data/mynteye/20171107vins_outside/1/img.avi";
std::string video_ts_file = "/home/symao/data/mynteye/20171107vins_outside/1/imgts.txt";
#endif

// sf inside 1C4F, forward
#ifdef DATASET2
std::string video_file = "/home/symao/data/mynteye/20171107vins/img.avi";
std::string video_ts_file = "/home/symao/data/mynteye/20171107vins/imgts.txt";
#endif

// shiyan outside, downward
#ifdef DATASET3
std::string video_file = "/home/symao/data/mynteye/20171220/15/img_cut.avi";
std::string video_ts_file = "/home/symao/data/mynteye/20171220/15/img_cutts.txt";
#endif

// shiyan outside police station, downward
#ifdef DATASET4
std::string video_file = "/home/symao/data/mynteye/20171220/17/img_cut.avi";
std::string video_ts_file = "/home/symao/data/mynteye/20171220/17/img_cutts.txt";
#endif

// shiyan outside walk, downward
#ifdef DATASET5
std::string video_file = "/home/symao/data/mynteye/20171220/17/img_cut1.avi";
std::string video_ts_file = "/home/symao/data/mynteye/20171220/17/img_cut1ts.txt";
#endif


int main(int argc, char **argv)
{
    //============ init =============//
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InstallFailureSignalHandler();
    ros::init(argc, argv, "svo");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    bool stereo_flag;
    pnh.param<bool>("pipeline_is_stereo", stereo_flag, false);
    std::shared_ptr<svo::FrameHandlerMono>    vo_mono;
    std::shared_ptr<svo::FrameHandlerStereo>  vo_stereo;

    if(stereo_flag)
    {
        vo_stereo = svo::factory::makeStereo(pnh);
        vo_stereo->start();
    }
    else
    {
        vo_mono = svo::factory::makeMono(pnh);
        vo_mono->start();
    }

    //========= load images =============//
    cv::VideoCapture cap(video_file);
    if(!cap.isOpened())
    {
        printf("[ERROR] read source video failed. file '%s' not exists.\n", video_file.c_str());
        return 0;
    }
    int n_img = cap.get(CV_CAP_PROP_FRAME_COUNT);
    printf("load %d images from video %s\n", n_img, video_file.c_str());
    std::ifstream fin_imgts(video_ts_file);
    if(!fin_imgts.is_open())
    {
        printf("[ERROR] stereo_video_play: cannot open file %s\n", video_ts_file.c_str());
        return -1;
    }

    //========= start svo =============//
    Viz3dThread viz;
    viz.updateWidget("cood", cv::viz::WCoordinateSystem(1));
    std::vector<cv::Affine3f> traj;

    cv::Mat image;
    double tframe;
    for(int i=0; i<n_img; i++)
    {
        cap.read(image);
        fin_imgts>>tframe;
        if(i<100) continue;

        printf("process frame %d\n", i);
        if(image.channels()==3)
            cv::cvtColor(image,image,cv::COLOR_BGR2GRAY);

        int r = image.rows/2;
        cv::Mat imgl = image.rowRange(0,r);
        cv::Mat imgr = image.rowRange(r,r*2);

        tic("process");
        svo::FrameBundlePtr last_frame;
        if(stereo_flag)
        {
            vo_stereo->addImages(imgl, imgr, tframe*1e9);
            last_frame = vo_stereo->getLastFrames();
        }
        else
        {
            vo_mono->addImage(imgl, tframe*1e9);
            last_frame = vo_mono->getLastFrames();
        }
        float cost_ms = toc("process");
#if 0
        std::ofstream fout("/home/symao/cost.txt",std::ios::app);
        fout<<cost_ms<<std::endl;
        fout.close();
#endif
        cv::Mat cv_T_w_b = svo_trans_to_cv_mat(last_frame->get_T_W_B());
        // std::cout<<"T:"<<cv_T_w_b<<std::endl;

        traj.push_back(cv_mat_to_affine3f(cv_T_w_b));

        viz.updateWidget("traj", cv::viz::WTrajectory(traj, 2, 1, cv::viz::Color::green()));
        cv::imshow("image",image);
        char key = cv::waitKey(10);
        if(key == 27) break;
    }

    viz.updateWidget("traj", cv::viz::WTrajectory(traj, 2, 1, cv::viz::Color::green()));
    printf("Press any key to exit.\n");
    getchar();
}
