#pragma once

#include <string>
#include <utility>
#include <variant>

namespace autoaim::core {
enum class ErrorCode { invalid_input, unavailable, expired, out_of_order, io, fault };
const char* error_name(ErrorCode code) noexcept;

struct Error {
  ErrorCode code;
  std::string message;
};

// 成功值和错误互斥；不能默认构造出伪成功状态。可携带 move-only 资源。
template <class T>
class Result {
public:
  static Result success(T value) { return Result(std::move(value)); }
  static Result failure(ErrorCode code, std::string message) {
    return Result(Error{code, std::move(message)});
  }
  bool has_value() const noexcept { return std::holds_alternative<T>(data_); }
  explicit operator bool() const noexcept { return has_value(); }
  T& value() & { return std::get<T>(data_); }
  const T& value() const & { return std::get<T>(data_); }
  T&& value() && { return std::get<T>(std::move(data_)); }
  const Error& error() const { return std::get<Error>(data_); }

private:
  explicit Result(T value) : data_(std::in_place_type<T>, std::move(value)) {}
  explicit Result(Error error) : data_(std::in_place_type<Error>, std::move(error)) {}
  std::variant<T, Error> data_;
};
}  // namespace autoaim::core
