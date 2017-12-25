#include <chrono>
#include <opencv2/opencv.hpp>
// svo headers
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <svo/common/frame.h>
#include <svo/frame_handler_mono.h>
#include <svo/frame_handler_stereo.h>
// local headers
#include "svo_factory.h"
#include "format_convert.h"

int main(int argc, char **argv)
{
    //============ init =============//
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InstallFailureSignalHandler();

    if(argc != 4)
    {
        printf("Usage: ./demo_mono_euroc param_file imgts_file img_dir\n");
        return -1;
    }

    const char* param_file = argv[1];
    const char* imgts_file = argv[2];
    const char* left_img_dir = argv[3];

    YamlParser params(param_file, "r");
    if(!params.isOpened()) return -1;

    std::shared_ptr<svo::FrameHandlerMono> vo_mono = svo::factory::makeMono(params);
    vo_mono->start();

    FILE* fp_log = fopen("vio.txt","w");
    fprintf(fp_log, "#timestamp x y z qw qx qy qz cost_ms\n");

    std::ifstream fin_imgts(imgts_file);
    if(!fin_imgts.is_open())
    {
        printf("File not exists:%s\n", imgts_file);
        return -1;
    }

    while(!fin_imgts.eof())
    {
        std::string ts_str;
        fin_imgts >> ts_str;
        std::stringstream ss(ts_str);
        double ts;
        ss >> ts;
        ts /= 1e9;
        char fimgl[128] = {0};
        sprintf(fimgl, "%s/%s.png", left_img_dir, ts_str.c_str());
        cv::Mat imgl = cv::imread(fimgl, cv::IMREAD_GRAYSCALE);
        if(imgl.rows <= 0 || imgl.cols <= 0) continue;

        auto t1 = std::chrono::system_clock::now();
        vo_mono->addImage(imgl, ts*1e9);
        auto pose = vo_mono->getLastFrames()->get_T_W_B();
        auto t2 = std::chrono::system_clock::now();
        float cost = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()
                        * (double)std::chrono::microseconds::period::num
                        / std::chrono::microseconds::period::den;

        auto p = pose.getPosition();
        auto q = pose.getRotation();
        fprintf(fp_log, "%f %f %f %f %f %f %f %f %f\n", ts, p.x(), p.y(), p.z(),
                        q.w(), q.x(), q.y(), q.z(), cost);
    }
    fclose(fp_log);
}
