#include "autoaim/control/checked_command.hpp"
#include "test_support.hpp"
#include <limits>
#include <type_traits>

int main() {
  using namespace autoaim;
  static_assert(std::is_aggregate_v<control::Command>);
  static_assert(std::is_standard_layout_v<control::Command>);
  static_assert(std::is_copy_assignable_v<control::Command>);
  static_assert(!std::is_default_constructible_v<control::CheckedCommand>);
  static_assert(!std::is_copy_assignable_v<control::CheckedCommand>);

  return test::run([] {
    // 五项结构化绑定锁定字段数量；检查名称、类型和顺序与参考 Command 一致。
    control::Command payload{true, false, 0.123456789012345, -0.234567890123456};
    auto& [control, shoot, yaw, pitch, horizon_distance] = payload;
    static_assert(std::is_same_v<decltype(control), bool>);
    static_assert(std::is_same_v<decltype(shoot), bool>);
    static_assert(std::is_same_v<decltype(yaw), double>);
    static_assert(std::is_same_v<decltype(pitch), double>);
    static_assert(std::is_same_v<decltype(horizon_distance), double>);
    CHECK(&control == &payload.control && &shoot == &payload.shoot);
    CHECK(&yaw == &payload.yaw && &pitch == &payload.pitch);
    CHECK(&horizon_distance == &payload.horizon_distance);
    CHECK(control && !shoot && horizon_distance == 0);
    CHECK(yaw == 0.123456789012345 && pitch == -0.234567890123456);

    const control::Command zero{};
    CHECK(!zero.control && !zero.shoot);
    CHECK(zero.yaw == 0 && zero.pitch == 0 && zero.horizon_distance == 0);

    const core::Stamp source(23, 7, core::TimePoint(100, core::ClockDomain::replay));
    const control::Pointing pointing{core::Radians(yaw), core::Radians(pitch), 2, -3, 4, -5};
    const control::CheckedCommand checked(source, pointing, core::Metres(1.75),
                                          core::CommandSpace::absolute, true, true);
    CHECK(checked.command.control && checked.command.shoot);
    CHECK(checked.command.yaw == yaw && checked.command.pitch == pitch);
    CHECK(checked.command.horizon_distance == 1.75);
    CHECK(checked.metadata.source->frame_id == 23 && checked.metadata.source->generation == 7);
    CHECK(core::same_time(checked.metadata.source->exposure, source.exposure));
    CHECK(checked.metadata.space == core::CommandSpace::absolute);
    CHECK(checked.metadata.yaw_rate_radps == 2 && checked.metadata.pitch_rate_radps == -3);
    CHECK(checked.metadata.yaw_acceleration_radps2 == 4);
    CHECK(checked.metadata.pitch_acceleration_radps2 == -5);

    const auto stop = control::CheckedCommand::stop(source);
    CHECK(!stop.command.control && !stop.command.shoot);
    CHECK(stop.command.yaw == 0 && stop.command.pitch == 0 && stop.command.horizon_distance == 0);
    CHECK(stop.metadata.source->frame_id == 23 && stop.metadata.source->generation == 7);
    CHECK(core::same_time(stop.metadata.source->exposure, source.exposure));
    CHECK(stop.metadata.yaw_rate_radps == 0 && stop.metadata.pitch_rate_radps == 0);
    CHECK(stop.metadata.yaw_acceleration_radps2 == 0);
    CHECK(stop.metadata.pitch_acceleration_radps2 == 0);
    CHECK(!control::CheckedCommand::stop().metadata.source);

    CHECK_THROWS(std::invalid_argument,
                 control::CheckedCommand(std::nullopt, pointing, core::Metres(1),
                                         core::CommandSpace::absolute, true, false));
    CHECK_THROWS(std::invalid_argument,
                 control::CheckedCommand(source, pointing, core::Metres(1),
                                         core::CommandSpace::absolute, false, true));
    CHECK_THROWS(std::invalid_argument,
                 control::CheckedCommand(source, pointing, core::Metres(-1),
                                         core::CommandSpace::absolute, true, false));
    auto non_finite = pointing;
    non_finite.yaw_rate_radps = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS(std::invalid_argument,
                 control::CheckedCommand(source, non_finite, core::Metres(1),
                                         core::CommandSpace::absolute, true, false));
  });
}
