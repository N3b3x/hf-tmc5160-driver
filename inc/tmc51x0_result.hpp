/**
 * @file tmc51x0_result.hpp
 * @brief Result type for error handling in TMC51x0 driver
 *
 * Provides a Rust-inspired Result<T> type for explicit error handling,
 * replacing boolean return values with rich error information.
 */

#ifndef TMC51X0_RESULT_HPP
#define TMC51X0_RESULT_HPP

#include <cstdint>

namespace tmc51x0 {

/**
 * @brief Error codes for TMC51x0 operations
 */
enum class ErrorCode : uint8_t {
  OK = 0,            ///< Operation succeeded
  NOT_INITIALIZED,   ///< Driver not initialized
  COMM_ERROR,        ///< Communication interface error (SPI/UART)
  INVALID_VALUE,     ///< Invalid parameter value
  INVALID_STATE,     ///< Operation not valid in current state
  TIMEOUT,           ///< Operation timed out
  HARDWARE_ERROR,    ///< Hardware fault detected
  SHORT_CIRCUIT,     ///< Short circuit detected
  OPEN_LOAD,         ///< Open load detected
  OVERTEMP_WARNING,  ///< Overtemperature warning threshold
  OVERTEMP_SHUTDOWN, ///< Overtemperature shutdown
  UNSUPPORTED        ///< Feature not supported by this chip variant
};

#ifndef TMC51X0_NO_ERROR_STRINGS
/**
 * @brief Get human-readable error message
 * @param code Error code
 * @return Error message string
 */
inline const char *ErrorMessage(ErrorCode code) {
  switch (code) {
  case ErrorCode::OK:
    return "OK";
  case ErrorCode::NOT_INITIALIZED:
    return "Driver not initialized";
  case ErrorCode::COMM_ERROR:
    return "Communication error";
  case ErrorCode::INVALID_VALUE:
    return "Invalid parameter value";
  case ErrorCode::INVALID_STATE:
    return "Invalid state for operation";
  case ErrorCode::TIMEOUT:
    return "Operation timed out";
  case ErrorCode::HARDWARE_ERROR:
    return "Hardware fault detected";
  case ErrorCode::SHORT_CIRCUIT:
    return "Short circuit detected";
  case ErrorCode::OPEN_LOAD:
    return "Open load detected";
  case ErrorCode::OVERTEMP_WARNING:
    return "Overtemperature warning";
  case ErrorCode::OVERTEMP_SHUTDOWN:
    return "Overtemperature shutdown";
  case ErrorCode::UNSUPPORTED:
    return "Feature not supported";
  default:
    return "Unknown error";
  }
}
#endif

/**
 * @brief Result type for operations that return a value
 * @tparam T The value type
 *
 * Provides explicit error handling with rich error information.
 * Use the bool operator or IsOk()/IsErr() for checking success.
 *
 * @code
 * auto result = driver.GetActualPosition();
 * if (result) {  // Bool operator
 *     float pos = result.Value();
 * } else {
 *     ErrorCode err = result.Error();
 * }
 * @endcode
 */
template <typename T> class Result {
private:
  ErrorCode error_;
  T value_;

public:
  /**
   * @brief Construct a successful result with value
   */
  explicit Result(T &&value) noexcept
      : error_(ErrorCode::OK), value_(static_cast<T &&>(value)) {}

  /**
   * @brief Construct a successful result with value (copy)
   */
  explicit Result(const T &value) noexcept
      : error_(ErrorCode::OK), value_(value) {}

  /**
   * @brief Construct an error result
   */
  explicit Result(ErrorCode error) noexcept : error_(error), value_{} {}

  /**
   * @brief Check if result is OK (bool operator for cleaner syntax)
   * @return true if operation succeeded
   *
   * @code
   * if (result.IsOk()) { // success
   *     float pos = result.Value();
   * }
   * if (result.IsErr()) { // error
   *     ErrorCode err = result.Error();
   * }
   * @endcode
   */
  [[nodiscard]] explicit operator bool() const noexcept {
    return error_ == ErrorCode::OK;
  }

  /**
   * @brief Check if result is OK
   * @return true if operation succeeded
   */
  [[nodiscard]] bool IsOk() const noexcept { return error_ == ErrorCode::OK; }

  /**
   * @brief Check if result is an error
   * @return true if operation failed
   */
  [[nodiscard]] bool IsErr() const noexcept { return error_ != ErrorCode::OK; }

  /**
   * @brief Get the error code
   * @return Error code (OK if successful)
   */
  [[nodiscard]] ErrorCode Error() const noexcept { return error_; }

#ifndef TMC51X0_NO_ERROR_STRINGS
  /**
   * @brief Get human-readable error message
   * @return Error message string
   */
  [[nodiscard]] const char *ErrorMessage() const noexcept {
    return tmc51x0::ErrorMessage(error_);
  }
#endif

  /**
   * @brief Get the result value (mutable reference)
   * @warning Only call if IsOk() returns true
   * @return Reference to the value
   */
  [[nodiscard]] T &Value() noexcept { return value_; }

  /**
   * @brief Get the result value (const reference)
   * @warning Only call if IsOk() returns true
   * @return Const reference to the value
   */
  [[nodiscard]] const T &Value() const noexcept { return value_; }

  /**
   * @brief Get the result value or a default
   * @param default_value Value to return if result is an error
   * @return The result value if OK, otherwise default_value
   */
  [[nodiscard]] T ValueOr(const T &default_value) const noexcept {
    return IsOk() ? value_ : default_value;
  }

  /**
   * @brief Support structured bindings: auto [err, value] = result;
   */
  template <std::size_t N> decltype(auto) get() const noexcept {
    if constexpr (N == 0)
      return error_;
    else if constexpr (N == 1)
      return value_;
  }

  template <std::size_t N> decltype(auto) get() noexcept {
    if constexpr (N == 0)
      return error_;
    else if constexpr (N == 1)
      return (value_);
  }
};

/**
 * @brief Result type for operations that don't return a value
 *
 * Specialized version of Result<T> for void operations.
 *
 * @code
 * auto result = driver.Initialize(config);
 * if (!result) {  // Bool operator
 *     printf("Error: %s\n", result.ErrorMessage());
 * }
 * @endcode
 */
template <> class Result<void> {
private:
  ErrorCode error_;

public:
  /**
   * @brief Construct a successful result
   */
  Result() noexcept : error_(ErrorCode::OK) {}

  /**
   * @brief Construct an error result
   */
  explicit Result(ErrorCode error) noexcept : error_(error) {}

  /**
   * @brief Check if result is OK (bool operator for cleaner syntax)
   * @return true if operation succeeded
   *
   * @code
   * if (result.IsOk()) { // success
   *     // Operation succeeded, no value to retrieve
   * }
   * if (result.IsErr()) { // error
   *     ErrorCode err = result.Error();
   *     const char* msg = result.ErrorMessage();
   * }
   * @endcode
   */
  [[nodiscard]] explicit operator bool() const noexcept {
    return error_ == ErrorCode::OK;
  }

  /**
   * @brief Check if result is OK
   * @return true if operation succeeded
   */
  [[nodiscard]] bool IsOk() const noexcept { return error_ == ErrorCode::OK; }

  /**
   * @brief Check if result is an error
   * @return true if operation failed
   */
  [[nodiscard]] bool IsErr() const noexcept { return error_ != ErrorCode::OK; }

  /**
   * @brief Get the error code
   * @return Error code (OK if successful)
   */
  [[nodiscard]] ErrorCode Error() const noexcept { return error_; }

#ifndef TMC51X0_NO_ERROR_STRINGS
  /**
   * @brief Get human-readable error message
   * @return Error message string
   */
  [[nodiscard]] const char *ErrorMessage() const noexcept {
    return tmc51x0::ErrorMessage(error_);
  }
#endif
};

// Type alias for Result<void> to allow Result<> syntax
using ResultVoid = Result<void>;

} // namespace tmc51x0

// Support for structured bindings
namespace std {
template <typename T>
struct tuple_size<tmc51x0::Result<T>> : integral_constant<size_t, 2> {};

template <typename T> struct tuple_element<0, tmc51x0::Result<T>> {
  using type = tmc51x0::ErrorCode;
};

template <typename T> struct tuple_element<1, tmc51x0::Result<T>> {
  using type = T;
};
} // namespace std

#endif // TMC51X0_RESULT_HPP
