#include "svo_factory.h"
#include <svo/svo.h>
#include <svo/common/imu_calibration.h>
#include <svo/frame_handler_mono.h>
#include <svo/frame_handler_stereo.h>
#include <svo/frame_handler_array.h>
#include <yaml-cpp/yaml.h>
#ifdef SVO_USE_VIN_BACKEND
#include <svo_gtsam/backend_optimizer.h>
#include <svo_gtsam/backend_interface.h>
#include <svo_gtsam/graph_manager.h>
#endif

namespace svo {
namespace factory {

BaseOptions loadBaseOptions(YamlParser& params, bool forward_default)
{
  BaseOptions o;
  o.max_n_kfs = params.read<int>("max_n_kfs", 5);
  o.use_imu = params.read<bool>("use_imu", false);
  o.trace_dir = params.read<std::string>("trace_dir", o.trace_dir);
  o.quality_min_fts = params.read<int>("quality_min_fts", 50);
  o.quality_max_fts_drop = params.read<int>("quality_max_drop_fts", 40);
  o.relocalization_max_trials = params.read<int>("relocalization_max_trials", 50);
  o.poseoptim_prior_lambda = params.read<double>("poseoptim_prior_lambda", 0.0);
  o.poseoptim_using_unit_sphere = params.read<bool>("poseoptim_using_unit_sphere", false);
  o.img_align_prior_lambda_rot = params.read<double>("img_align_prior_lambda_rot", 0.0);
  o.img_align_prior_lambda_trans = params.read<double>("img_align_prior_lambda_trans", 0.0);
  o.structure_optimization_max_pts = params.read<int>("structure_optimization_max_pts", 20);
  o.init_map_scale = params.read<double>("map_scale", 1.0);
  std::string default_kf_criterion = forward_default ? "FORWARD" : "DOWNLOOKING";
  if(params.read<std::string>("kfselect_criterion", default_kf_criterion) == "FORWARD")
    o.kfselect_criterion = KeyframeCriterion::FORWARD;
  else
    o.kfselect_criterion = KeyframeCriterion::DOWNLOOKING;
  o.kfselect_min_dist = params.read<double>("kfselect_min_dist", 0.12);
  o.kfselect_numkfs_upper_thresh = params.read<int>("kfselect_numkfs_upper_thresh", 120);
  o.kfselect_numkfs_lower_thresh = params.read<double>("kfselect_numkfs_lower_thresh", 70);
  o.kfselect_min_dist_metric = params.read<double>("kfselect_min_dist_metric", 0.01);
  o.kfselect_min_angle = params.read<double>("kfselect_min_angle", 20);
  o.kfselect_min_disparity = params.read<double>("kfselect_min_disparity", 40);
  o.kfselect_min_num_frames_between_kfs = params.read<int>("kfselect_min_num_frames_between_kfs", 2);
  o.img_align_max_level = params.read<int>("img_align_max_level", 4);
  o.img_align_min_level = params.read<int>("img_align_min_level", 2);
  o.img_align_robustification = params.read<bool>("img_align_robustification", false);
  o.img_align_use_distortion_jacobian = params.read<bool>("img_align_use_distortion_jacobian", false);
  o.img_align_est_illumination_gain = params.read<bool>("img_align_est_illumination_gain", false);
  o.img_align_est_illumination_offset = params.read<bool>("img_align_est_illumination_offset", false);
  o.poseoptim_thresh = params.read<double>("poseoptim_thresh", 2.0);
  o.update_seeds_with_old_keyframes = params.read<bool>("update_seeds_with_old_keyframes", true);
  o.use_async_reprojectors = params.read<bool>("use_async_reprojectors", false);
  return o;
}

DetectorOptions loadDetectorOptions(YamlParser& params)
{
  DetectorOptions o;
  o.cell_size = params.read<int>("grid_size", 35);
  o.max_level = params.read<int>("n_pyr_levels", 3) - 1;
  o.threshold_primary = params.read<int>("detector_threshold_primary", 10);
  o.threshold_secondary = params.read<int>("detector_threshold_secondary", 200);
  if(params.read<bool>("use_edgelets", true))
    o.detector_type = DetectorType::kFastGrad;
  else
    o.detector_type = DetectorType::kFast;
  return o;
}

DepthFilterOptions loadDepthFilterOptions(YamlParser& params)
{
  DepthFilterOptions o;
  o.max_search_level = params.read<int>("n_pyr_levels", 3) - 1;
  o.use_threaded_depthfilter = params.read<bool>("use_threaded_depthfilter", true);
  o.seed_convergence_sigma2_thresh = params.read<double>("seed_convergence_sigma2_thresh", 200.0);
  o.scan_epi_unit_sphere = params.read<bool>("scan_epi_unit_sphere", false);
  o.affine_est_offset= params.read<bool>("depth_filter_affine_est_offset", true);
  o.affine_est_gain = params.read<bool>("depth_filter_affine_est_gain", false);
  o.max_n_seeds_per_frame = params.read<int>("max_fts", 120) * params.read<double>("max_seeds_ratio", 3.0);
  return o;
}

InitializationOptions loadInitializationOptions(YamlParser& params)
{
  InitializationOptions o;
  o.init_min_features = params.read<int>("init_min_features", 100);
  o.init_min_tracked = params.read<int>("init_min_tracked", 80);
  o.init_min_inliers = params.read<int>("init_min_inliers", 70);
  o.init_min_disparity = params.read<double>("init_min_disparity", 40.0);
  o.init_min_features_factor = params.read<double>("init_min_features_factor", 2.0);
  o.reproj_error_thresh = params.read<double>("reproj_err_thresh", 2.0);
  o.init_disparity_pivot_ratio = params.read<double>("init_disparity_pivot_ratio", 0.5);
  std::string init_method = params.read<std::string>("init_method", "FivePoint");
  if(init_method == "Homography")
    o.init_type = InitializerType::kHomography;
  else if(init_method == "TwoPoint")
    o.init_type = InitializerType::kTwoPoint;
  else if(init_method == "FivePoint")
    o.init_type = InitializerType::kFivePoint;
  else if(init_method == "OneShot")
    o.init_type = InitializerType::kOneShot;
  else
    SVO_ERROR_STREAM("Initialization Method not supported: " << init_method);
  return o;
}

FeatureTrackerOptions loadTrackerOptions(YamlParser& params)
{
  FeatureTrackerOptions o;
  o.klt_max_level = params.read<int>("klt_max_level", 4);
  o.klt_min_level = params.read<int>("klt_min_level", 0);
  return o;
}

ReprojectorOptions loadReprojectorOptions(YamlParser& params)
{
  ReprojectorOptions o;
  o.max_n_kfs = params.read<int>("reprojector_max_n_kfs", 5);
  o.max_n_features_per_frame = params.read<int>("max_fts", 160);
  o.cell_size = params.read<int>("grid_size", 35);
  o.reproject_unconverged_seeds = params.read<bool>("reproject_unconverged_seeds", true);
  o.affine_est_offset = params.read<bool>("reprojector_affine_est_offset", true);
  o.affine_est_gain = params.read<bool>("reprojector_affine_est_gain", false);
  return o;
}

CameraBundle::Ptr loadCameraFromYaml(YamlParser& params)
{
  std::string calib_file = params.read<std::string>("calib_file", "~/cam.yaml");
  CameraBundle::Ptr ncam = CameraBundle::loadFromYaml(calib_file);
  std::cout << "loaded " << ncam->numCameras() << " cameras";
  for(const auto& cam : ncam->getCameraVector())
    cam->printParameters(std::cout, "");
  return ncam;
}

StereoTriangulationOptions loadStereoOptions(YamlParser& params)
{
  StereoTriangulationOptions o;
  o.triangulate_n_features = params.read<int>("max_fts", 120);
  o.max_depth_inv = params.read<double>("max_depth_inv", 1.0/50.0);
  o.min_depth_inv = params.read<double>("min_depth_inv", 1.0/0.5);
  o.mean_depth_inv = params.read<double>("mean_depth_inv", 1.0/2.0);
  return o;
}

ImuHandler::Ptr getImuHandler(YamlParser& params)
{
  std::string calib_file = params.read<std::string>("calib_file", "");
  ImuCalibration imu_calib = ImuHandler::loadCalibrationFromFile(calib_file);
  imu_calib.print("Loaded IMU Calibration");
  ImuInitialization imu_init = ImuHandler::loadInitializationFromFile(calib_file);
  imu_init.print("Loaded IMU Initialization");
  ImuHandler::Ptr imu_handler(new ImuHandler(imu_calib, imu_init));
  return imu_handler;
}

void setInitialPose(YamlParser& params, FrameHandlerBase& vo)
{
  Transformation T_world_imuinit(
        Quaternion(params.read<double>("T_world_imuinit/qw", 1.0),
                   params.read<double>("T_world_imuinit/qx", 0.0),
                   params.read<double>("T_world_imuinit/qy", 0.0),
                   params.read<double>("T_world_imuinit/qz", 0.0)),
        Vector3d(params.read<double>("T_world_imuinit/tx", 0.0),
                 params.read<double>("T_world_imuinit/ty", 0.0),
                 params.read<double>("T_world_imuinit/tz", 0.0)));
  vo.setInitialImuPose(T_world_imuinit);
}


FrameHandlerMono::Ptr makeMono(YamlParser& params, const CameraBundlePtr& cam)
{
  // Create camera
  CameraBundle::Ptr ncam = (cam) ? cam : loadCameraFromYaml(params);

  // Init VO
  FrameHandlerMono::Ptr vo =
      std::make_shared<FrameHandlerMono>(
        loadBaseOptions(params, false),
        loadDepthFilterOptions(params),
        loadDetectorOptions(params),
        loadInitializationOptions(params),
        loadReprojectorOptions(params),
        loadTrackerOptions(params),
        ncam);

  // Get initial position and orientation of IMU
  setInitialPose(params, *vo);

  return vo;
}

FrameHandlerStereo::Ptr makeStereo(YamlParser& params, const CameraBundlePtr& cam)
{
  // Load cameras
  CameraBundle::Ptr ncam = (cam) ? cam : loadCameraFromYaml(params);

  // Init VO
  InitializationOptions init_options = loadInitializationOptions(params);
  init_options.init_type = InitializerType::kStereo;
  FrameHandlerStereo::Ptr vo =
      std::make_shared<FrameHandlerStereo>(
        loadBaseOptions(params, true),
        loadDepthFilterOptions(params),
        loadDetectorOptions(params),
        init_options,
        loadStereoOptions(params),
        loadReprojectorOptions(params),
        loadTrackerOptions(params),
        ncam);

  // Get initial position and orientation of IMU
  setInitialPose(params, *vo);

  return vo;
}

FrameHandlerArray::Ptr makeArray(YamlParser& params, const CameraBundlePtr& cam)
{
  // Load cameras
  CameraBundle::Ptr ncam = (cam) ? cam : loadCameraFromYaml(params);

  // Init VO
  InitializationOptions init_options = loadInitializationOptions(params);
  init_options.init_type = InitializerType::kArrayGeometric;
  init_options.init_min_disparity = 25;
  DepthFilterOptions depth_filter_options = loadDepthFilterOptions(params);
  depth_filter_options.verbose = true;
  FrameHandlerArray::Ptr vo =
      std::make_shared<FrameHandlerArray>(
        loadBaseOptions(params, true),
        depth_filter_options,
        loadDetectorOptions(params),
        init_options,
        loadReprojectorOptions(params),
        loadTrackerOptions(params),
        ncam);

  // Get initial position and orientation of IMU
  setInitialPose(params, *vo);

  return vo;
}

} // namespace factory
} // namespace svo
