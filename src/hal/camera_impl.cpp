#include "autoaim/hal/hikrobot_camera.hpp"
#include <cstring>
#include <limits>
#ifdef AUTOAIM_HAS_HIKROBOT
#include <MvCameraControl.h>
#endif

namespace autoaim::hal {
struct HikrobotCamera::Impl {
  HikrobotOptions options;
  Clock& clock;
  core::Generation generation;
  core::FrameId next_id = 1;
  void* handle = nullptr;
  bool opened = false;
  bool grabbing = false;
  Impl(HikrobotOptions values, Clock& source, core::Generation epoch)
      : options(std::move(values)), clock(source), generation(epoch) {}
};
HikrobotCamera::HikrobotCamera(HikrobotOptions options, Clock& clock, core::Generation generation)
    : impl_(std::make_unique<Impl>(std::move(options), clock, generation)) {
  const auto& o = impl_->options;
  if (o.serial_number.empty() || o.exposure.value() <= 0 || !std::isfinite(o.gain) || o.gain < 0 ||
      !std::isfinite(o.frame_rate_hz) || o.frame_rate_hz <= 0 || o.transport_delay.value() < 0 ||
      o.timing_uncertainty.value() < 0 || o.maximum_frame_bytes == 0 || o.maximum_frame_bytes > UINT32_MAX ||
      clock.now().domain() != core::ClockDomain::host_monotonic)
    throw std::invalid_argument("Invalid explicit Hikrobot parameters/clock domain");
}
HikrobotCamera::~HikrobotCamera() { close_device(); }
core::Result<bool> HikrobotCamera::open_device() {
  using Result = core::Result<bool>;
  auto& self = *impl_;
  if (self.handle) return Result::failure(core::ErrorCode::invalid_input, "Camera already open");
#ifdef AUTOAIM_HAS_HIKROBOT
  try {
    const auto check = [](int status) {
      if (status != MV_OK) throw std::runtime_error("Hikrobot SDK error " + std::to_string(static_cast<unsigned>(status)));
    };
    MV_CC_DEVICE_INFO_LIST devices{};
    check(MV_CC_EnumDevices(MV_USB_DEVICE, &devices));
    MV_CC_DEVICE_INFO* selected = nullptr;
    for (unsigned i = 0; i < devices.nDeviceNum; ++i) {
      auto* info = devices.pDeviceInfo[i];
      const auto* serial = reinterpret_cast<const char*>(info->SpecialInfo.stUsb3VInfo.chSerialNumber);
      if (self.options.serial_number == std::string(serial, strnlen(serial, sizeof(info->SpecialInfo.stUsb3VInfo.chSerialNumber)))) selected = info;
    }
    if (!selected) throw std::runtime_error("Requested camera serial not found");
    check(MV_CC_CreateHandle(&self.handle, selected));
    check(MV_CC_OpenDevice(self.handle)); self.opened = true;
    check(MV_CC_SetEnumValue(self.handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF));
    check(MV_CC_SetEnumValue(self.handle, "GainAuto", MV_GAIN_MODE_OFF));
    check(MV_CC_SetFloatValue(self.handle, "ExposureTime", static_cast<float>(self.options.exposure.value() * 1e6)));
    check(MV_CC_SetFloatValue(self.handle, "Gain", static_cast<float>(self.options.gain)));
    check(MV_CC_SetFrameRate(self.handle, static_cast<float>(self.options.frame_rate_hz)));
    check(MV_CC_SetImageNodeNum(self.handle, 2));
    check(MV_CC_SetGrabStrategy(self.handle, MV_GrabStrategy_LatestImagesOnly));
    check(MV_CC_StartGrabbing(self.handle)); self.grabbing = true;
    return Result::success(true);
  } catch (const std::exception& error) {
    close_device(); return Result::failure(core::ErrorCode::io, error.what());
  }
#else
  return Result::failure(core::ErrorCode::unavailable, "Hikrobot SDK disabled at build time");
#endif
}
void HikrobotCamera::close_device() noexcept {
  auto& self = *impl_;
#ifdef AUTOAIM_HAS_HIKROBOT
  if (self.grabbing) MV_CC_StopGrabbing(self.handle);
  if (self.opened) MV_CC_CloseDevice(self.handle);
  if (self.handle) MV_CC_DestroyHandle(self.handle);
#endif
  self.handle = nullptr; self.opened = false; self.grabbing = false;
}
void HikrobotCamera::set_generation(core::Generation generation) {
  if (generation < impl_->generation) throw std::invalid_argument("Camera generation cannot rewind");
  impl_->generation = generation;
}
core::Result<FrameRead> HikrobotCamera::read_for(core::Seconds timeout) {
  using Result = core::Result<FrameRead>;
  auto& self = *impl_;
  if (!self.grabbing || timeout.value() <= 0 || timeout.value() > 1)
    return Result::failure(core::ErrorCode::unavailable, "Camera closed or timeout outside (0,1] seconds");
#ifdef AUTOAIM_HAS_HIKROBOT
  MV_FRAME_OUT raw{};
  const int code = MV_CC_GetImageBuffer(self.handle, &raw, static_cast<unsigned>(std::ceil(timeout.value() * 1000)));
  if (static_cast<unsigned>(code) == MV_E_NODATA) return Result::success({InputStatus::timeout, {}});
  if (code != MV_OK) return Result::failure(core::ErrorCode::io, "Camera acquisition failed");
  struct Lease {
    void* handle; MV_FRAME_OUT& raw;
    ~Lease() { MV_CC_FreeImageBuffer(handle, &raw); }
  } lease{self.handle, raw};
  try {
    const auto received = self.clock.now();
    const auto& info = raw.stFrameInfo;
    const auto bytes = static_cast<std::size_t>(info.nWidth) * info.nHeight * 3;
    if (!info.nWidth || !info.nHeight || bytes > self.options.maximum_frame_bytes || self.next_id == UINT64_MAX)
      throw std::runtime_error("Camera frame dimensions/capacity/id invalid");
    auto pixels = std::make_shared<std::vector<std::uint8_t>>(bytes);
    MV_CC_PIXEL_CONVERT_PARAM_EX convert{};
    convert.nWidth = info.nWidth; convert.nHeight = info.nHeight;
    convert.enSrcPixelType = info.enPixelType; convert.pSrcData = raw.pBufAddr; convert.nSrcDataLen = info.nFrameLen;
    convert.enDstPixelType = PixelType_Gvsp_BGR8_Packed; convert.pDstBuffer = pixels->data();
    convert.nDstBufferSize = static_cast<unsigned>(pixels->size());
    if (MV_CC_ConvertPixelTypeEx(self.handle, &convert) != MV_OK || convert.nDstLen != pixels->size())
      throw std::runtime_error("Camera BGR8 conversion failed");
    // 旧配置只能提供接收延迟估计，SDK tick 未建立主机时钟映射，不能标为硬同步。
    const auto exposure = core::advance(received, core::Seconds(-self.options.transport_delay.value() - self.options.exposure.value() * 0.5));
    auto frame = std::make_shared<const core::CapturedFrame>(core::Stamp(self.next_id++, self.generation, exposure),
      core::Image(info.nWidth, info.nHeight, static_cast<std::size_t>(info.nWidth) * 3, pixels), received,
      core::TimeOrigin::receipt_estimated, self.options.timing_uncertainty, core::Evidence::declared());
    return Result::success({InputStatus::frame, frame});
  } catch (const std::exception& error) { return Result::failure(core::ErrorCode::io, error.what()); }
#else
  return Result::failure(core::ErrorCode::unavailable, "Hikrobot SDK disabled");
#endif
}
}  // namespace autoaim::hal
