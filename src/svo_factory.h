#pragma once

#include "vs_yaml_parser.h"
#include <memory>
#include <svo/common/camera_fwd.h>

namespace svo {

// forward declarations
class ImuHandler;
class FrameHandlerMono;
class FrameHandlerStereo;
class FrameHandlerArray;
class FrameHandlerDenseMono;

namespace factory {

/// Get IMU Handler.
std::shared_ptr<ImuHandler> getImuHandler(YamlParser& params);

/// Factory for Mono-SVO.
std::shared_ptr<FrameHandlerMono> makeMono(YamlParser& params,
    const CameraBundlePtr& cam = nullptr);

/// Factory for Stereo-SVO.
std::shared_ptr<FrameHandlerStereo> makeStereo(YamlParser& params,
    const CameraBundlePtr& cam = nullptr);

/// Factory for Camera-Array-SVO.
std::shared_ptr<FrameHandlerArray> makeArray(YamlParser& params,
    const CameraBundlePtr& cam = nullptr);

/// Factory for Camera-Array-SVO
std::shared_ptr<FrameHandlerDenseMono> makeDenseMono(YamlParser& params);

} // namespace factory
} // namespace mono
