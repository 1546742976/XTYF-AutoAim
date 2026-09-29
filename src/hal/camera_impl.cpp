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
  std::optional<HikrobotReadback> actual;
  std::optional<std::uint32_t> previous_device_frame;

  Impl(HikrobotOptions values, Clock& source, core::Generation epoch)
      : options(std::move(values)), clock(source), generation(epoch) {
  }
};

HikrobotCamera::HikrobotCamera(HikrobotOptions options, Clock& clock, core::Generation generation)
    : impl_(std::make_unique<Impl>(std::move(options), clock, generation)) {
  validate_hikrobot_options(impl_->options);

  if (clock.now().domain() != core::ClockDomain::host_monotonic)
    throw std::invalid_argument("Invalid explicit Hikrobot parameters/clock domain");
}

HikrobotCamera::~HikrobotCamera() {
  close_device();
}

std::optional<HikrobotReadback> HikrobotCamera::readback() const {
  return impl_->actual;
}

core::Result<bool> HikrobotCamera::open_device() {
  using Result = core::Result<bool>;
  auto& self = *impl_;

  if (self.handle)
    return Result::failure(core::ErrorCode::invalid_input, "Camera already open");
#ifdef AUTOAIM_HAS_HIKROBOT
  try {
    const auto check = [](int status) {
      if (status != MV_OK)
        throw std::runtime_error("Hikrobot SDK error " +
                                 std::to_string(static_cast<unsigned>(status)));
    };

    MV_CC_DEVICE_INFO_LIST devices{};
    check(MV_CC_EnumDevices(MV_USB_DEVICE, &devices));
    MV_CC_DEVICE_INFO* selected = nullptr;

    for (unsigned i = 0; i < devices.nDeviceNum; ++i) {
      auto* info = devices.pDeviceInfo[i];
      const auto* serial =
          reinterpret_cast<const char*>(info->SpecialInfo.stUsb3VInfo.chSerialNumber);

      if (self.options.serial_number ==
          std::string(serial,
                      strnlen(serial, sizeof(info->SpecialInfo.stUsb3VInfo.chSerialNumber))))
        selected = info;
    }

    if (!selected)
      throw std::runtime_error("Requested camera serial not found");

    const auto* model =
        reinterpret_cast<const char*>(selected->SpecialInfo.stUsb3VInfo.chModelName);
    if (std::string(model, strnlen(model, sizeof(selected->SpecialInfo.stUsb3VInfo.chModelName))) !=
        "MV-CS016-10UC")
      throw std::runtime_error("Selected camera is not MV-CS016-10UC");

    check(MV_CC_CreateHandle(&self.handle, selected));
    check(MV_CC_OpenDevice(self.handle));
    self.opened = true;
    const auto set_int = [&](const char* key, std::int64_t value) {
      check(MV_CC_SetIntValueEx(self.handle, key, value));
    };
    const auto get_int = [&](const char* key) {
      MVCC_INTVALUE_EX value{};
      check(MV_CC_GetIntValueEx(self.handle, key, &value));

      return std::int64_t(value.nCurValue);
    };
    const auto get_enum = [&](const char* key) {
      MVCC_ENUMVALUE value{};
      check(MV_CC_GetEnumValue(self.handle, key, &value));

      return value.nCurValue;
    };
    const auto get_float = [&](const char* key) {
      MVCC_FLOATVALUE value{};
      check(MV_CC_GetFloatValue(self.handle, key, &value));

      return double(value.fCurValue);
    };

    // 先归零偏移，再设置范围，避免旧 ROI 使合法的新范围越界。
    set_int("OffsetX", 0);
    set_int("OffsetY", 0);
    set_int("BinningHorizontal", 1);
    set_int("BinningVertical", 1);
    set_int("DecimationHorizontal", 1);
    set_int("DecimationVertical", 1);
    set_int("Width", self.options.geometry.width);
    set_int("Height", self.options.geometry.height);
    set_int("OffsetX", self.options.geometry.offset_x);
    set_int("OffsetY", self.options.geometry.offset_y);
    const auto pixel_type = self.options.pixel_format == CameraPixelFormat::bayer_rg8
                                ? PixelType_Gvsp_BayerRG8 : PixelType_Gvsp_BGR8_Packed;
    check(MV_CC_SetEnumValue(self.handle, "PixelFormat", pixel_type));
    const auto white_balance = self.options.white_balance == WhiteBalancePolicy::manual
                                   ? MV_BALANCEWHITE_AUTO_OFF : MV_BALANCEWHITE_AUTO_CONTINUOUS;
    check(MV_CC_SetEnumValue(self.handle, "BalanceWhiteAuto", white_balance));

    if (self.options.balance_ratio) {
      constexpr const char* colors[] = {"Red", "Green", "Blue"};

      for (int i = 0; i < 3; ++i) {
        check(MV_CC_SetEnumValueByString(self.handle, "BalanceRatioSelector", colors[i]));
        set_int("BalanceRatio", (*self.options.balance_ratio)[i]);
      }
    }
    check(MV_CC_SetEnumValue(self.handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF));
    check(MV_CC_SetEnumValue(self.handle, "GainAuto", MV_GAIN_MODE_OFF));
    check(MV_CC_SetFloatValue(self.handle, "ExposureTime",
                              static_cast<float>(self.options.exposure.value() * 1e6)));

    check(MV_CC_SetFloatValue(self.handle, "Gain", static_cast<float>(self.options.gain)));
    check(MV_CC_SetBoolValue(self.handle, "AcquisitionFrameRateEnable", true));
    check(MV_CC_SetFrameRate(self.handle, static_cast<float>(self.options.frame_rate_hz)));

    if (get_enum("PixelFormat") != static_cast<unsigned>(pixel_type) ||
        get_enum("BalanceWhiteAuto") != static_cast<unsigned>(white_balance) ||
        get_enum("ExposureAuto") != MV_EXPOSURE_AUTO_MODE_OFF ||
        get_enum("GainAuto") != MV_GAIN_MODE_OFF || get_int("BinningHorizontal") != 1 ||
        get_int("BinningVertical") != 1 || get_int("DecimationHorizontal") != 1 ||
        get_int("DecimationVertical") != 1)
      throw std::runtime_error("Camera format/automatic policy/subsampling readback mismatch");

    std::optional<std::array<std::int64_t, 3>> ratios;
    if (self.options.balance_ratio) {
      ratios = std::array<std::int64_t, 3>{};
      constexpr const char* colors[] = {"Red", "Green", "Blue"};

      for (int i = 0; i < 3; ++i) {
        check(MV_CC_SetEnumValueByString(self.handle, "BalanceRatioSelector", colors[i]));
        (*ratios)[i] = get_int("BalanceRatio");
      }
    }

    HikrobotReadback actual{{int(get_int("Width")), int(get_int("Height")),
                             int(get_int("OffsetX")), int(get_int("OffsetY"))},
                            self.options.pixel_format, self.options.white_balance, ratios,
                            core::Seconds(get_float("ExposureTime") * 1e-6), get_float("Gain"),
                            get_float("AcquisitionFrameRate")};
    validate_hikrobot_readback(self.options, actual);
    self.actual = actual;
    check(MV_CC_SetImageNodeNum(self.handle, 2));
    check(MV_CC_SetGrabStrategy(self.handle, MV_GrabStrategy_LatestImagesOnly));
    check(MV_CC_StartGrabbing(self.handle));
    self.grabbing = true;

    return Result::success(true);
  } catch (const std::exception& error) {
    close_device();

    return Result::failure(core::ErrorCode::io, error.what());
  }
#else
  return Result::failure(core::ErrorCode::unavailable, "Hikrobot SDK disabled at build time");
#endif
}

void HikrobotCamera::close_device() noexcept {
  auto& self = *impl_;
#ifdef AUTOAIM_HAS_HIKROBOT
  if (self.grabbing)
    MV_CC_StopGrabbing(self.handle);

  if (self.opened)
    MV_CC_CloseDevice(self.handle);

  if (self.handle)
    MV_CC_DestroyHandle(self.handle);
#endif
  self.handle = nullptr;
  self.opened = false;
  self.grabbing = false;
  self.actual.reset();
  self.previous_device_frame.reset();
}

void HikrobotCamera::set_generation(core::Generation generation) {
  if (generation < impl_->generation)
    throw std::invalid_argument("Camera generation cannot rewind");

  impl_->generation = generation;
}

core::Result<FrameRead> HikrobotCamera::read_for(core::Seconds timeout) {
  using Result = core::Result<FrameRead>;
  auto& self = *impl_;

  if (!self.grabbing || timeout.value() <= 0 || timeout.value() > 1)
    return Result::failure(core::ErrorCode::unavailable,
                           "Camera closed or timeout outside (0,1] seconds");
#ifdef AUTOAIM_HAS_HIKROBOT
  MV_FRAME_OUT raw{};
  const int code = MV_CC_GetImageBuffer(self.handle, &raw,
                                        static_cast<unsigned>(std::ceil(timeout.value() * 1000)));

  if (static_cast<unsigned>(code) == MV_E_NODATA)
    return Result::success({InputStatus::timeout, {}});

  if (code != MV_OK)
    return Result::failure(core::ErrorCode::io, "Camera acquisition failed");

  struct Lease {
    void* handle;
    MV_FRAME_OUT& raw;

    ~Lease() {
      MV_CC_FreeImageBuffer(handle, &raw);
    }
  } lease{self.handle, raw};

  try {
    const auto received = self.clock.now();
    const auto& info = raw.stFrameInfo;
    const auto bytes = static_cast<std::size_t>(info.nWidth) * info.nHeight * 3;

    const auto& actual = *self.actual;
    const auto expected_pixel = actual.pixel_format == CameraPixelFormat::bayer_rg8
                                    ? PixelType_Gvsp_BayerRG8 : PixelType_Gvsp_BGR8_Packed;

    if (info.nWidth != actual.geometry.width || info.nHeight != actual.geometry.height ||
        info.nOffsetX != actual.geometry.offset_x || info.nOffsetY != actual.geometry.offset_y ||
        info.enPixelType != expected_pixel || bytes > self.options.maximum_frame_bytes ||
        self.next_id == UINT64_MAX)
      throw std::runtime_error("Camera frame dimensions/capacity/id invalid");

    auto pixels = std::make_shared<std::vector<std::uint8_t>>(bytes);
    MV_CC_PIXEL_CONVERT_PARAM_EX convert{};
    convert.nWidth = info.nWidth;
    convert.nHeight = info.nHeight;
    convert.enSrcPixelType = info.enPixelType;
    convert.pSrcData = raw.pBufAddr;
    convert.nSrcDataLen = info.nFrameLen;
    convert.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    convert.pDstBuffer = pixels->data();
    convert.nDstBufferSize = static_cast<unsigned>(pixels->size());

    if (MV_CC_ConvertPixelTypeEx(self.handle, &convert) != MV_OK ||
        convert.nDstLen != pixels->size())
      throw std::runtime_error("Camera BGR8 conversion failed");

    // 旧配置只能提供接收延迟估计，SDK tick 未建立主机时钟映射，不能标为硬同步。
    const auto exposure =
        core::advance(received, core::Seconds(-self.options.transport_delay.value() -
                                              actual.exposure.value() * 0.5));

    const auto ticks = (std::uint64_t(info.nDevTimeStampHigh) << 32) | info.nDevTimeStampLow;
    const auto metadata = hikrobot_metadata(self.previous_device_frame, info.nFrameNum, ticks,
                                            actual);
    self.previous_device_frame = info.nFrameNum;
    auto frame = std::make_shared<const core::CapturedFrame>(
        core::Stamp(self.next_id++, self.generation, exposure),
        core::Image(info.nWidth, info.nHeight, static_cast<std::size_t>(info.nWidth) * 3, pixels),
        received, core::TimeOrigin::receipt_estimated, self.options.timing_uncertainty,
        core::Evidence::declared(), metadata);

    return Result::success({InputStatus::frame, frame});
  } catch (const std::exception& error) {
    return Result::failure(core::ErrorCode::io, error.what());
  }
#else
  return Result::failure(core::ErrorCode::unavailable, "Hikrobot SDK disabled");
#endif
}
} // namespace autoaim::hal
