/**
 * MIT License
 *
 * @brief Public datatypes and status contracts for RCLink.
 *
 * @file Types.hpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-08
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */

#pragma once

#include "Constants.hpp"

#include <cstddef>
#include <cstdint>

namespace rc
{
/**
     * @brief Capabilities exposed by a receiver transport.
     */
struct RcTransportCaps
{
    bool has_proto_failsafe{false}; ///< Protocol exposes a receiver/protocol failsafe flag.
    bool has_link_stats{false};     ///< Protocol exposes RSSI/LQ or similar link statistics.
    bool has_telemetry{false};      ///< Transport supports telemetry back to the transmitter.
    bool half_duplex{false};        ///< Transport uses a half-duplex physical link.
};

/**
     * @brief Logical channel type.
     */
enum class ChannelKind : std::uint8_t
{
    Axis,  ///< Continuous control such as a stick or knob.
    Switch ///< Discrete two-or-more-position control.
};

/**
     * @brief Axis calibration and shaping specification.
     */
struct RcAxisSpec
{
    std::int16_t raw_lo{1000};     ///< Lower calibrated receiver value in microseconds.
    std::int16_t raw_hi{2000};     ///< Upper calibrated receiver value in microseconds.
    std::int16_t raw_center{1500}; ///< Physical neutral/centre value in microseconds.
    std::int16_t deadband_us{0};   ///< Deadband half-width around @ref raw_center.

    float out_lo{-100.0f};      ///< Output at @ref raw_lo.
    float out_hi{100.0f};       ///< Output at @ref raw_hi.
    float out_center{0.0f};     ///< Explicit output at @ref raw_center when enabled.
    bool has_out_center{false}; ///< True when @ref out_center was explicitly configured.
    float expo{0.0f};           ///< Cubic-mix shaping factor in the range [0,1].
    bool invert{false};         ///< Reverse the normalized axis direction.
};

static constexpr std::size_t kRcMaxSwitchVals = 8u; ///< Maximum switch positions supported by one role.

/**
     * @brief Discrete switch specification.
     */
struct RcSwitchSpec
{
    std::uint8_t count{0};                       ///< Number of logical output positions.
    float vals[kRcMaxSwitchVals]{};              ///< Output value for each position.
    std::uint8_t raw_count{0};                   ///< Number of explicit raw snap levels.
    std::int16_t raw_levels[kRcMaxSwitchVals]{}; ///< Explicit raw snap levels in microseconds.

    bool auto_levels{false};       ///< Allow commissioning-time centroid learning when explicitly enabled by RcLink.
    std::uint16_t hyst_us{60};     ///< Hysteresis band used while snapping learned centroids.
    float learn_alpha{0.20f};      ///< Commissioning-time centroid EMA factor in the range (0,1].
    std::uint16_t min_sep_us{120}; ///< Minimum accepted spacing between switch centroids.
};

/**
     * @brief Complete specification for one logical receiver role.
     */
struct RcChannelSpec
{
    ChannelKind kind{ChannelKind::Axis}; ///< Active specification type.
    RcAxisSpec axis{};                   ///< Axis specification.
    RcSwitchSpec sw{};                   ///< Switch specification.

    /**
         * @brief Build an axis channel specification.
         *
         * @param a Axis settings to copy.
         * @return Configured channel specification.
         */
    static RcChannelSpec makeAxis(const RcAxisSpec &a)
    {
        RcChannelSpec s;
        s.kind = ChannelKind::Axis;
        s.axis = a;
        return s;
    }

    /**
         * @brief Build a switch channel specification.
         *
         * @param w Switch settings to copy.
         * @return Configured channel specification.
         */
    static RcChannelSpec makeSwitch(const RcSwitchSpec &w)
    {
        RcChannelSpec s;
        s.kind = ChannelKind::Switch;
        s.sw = w;
        return s;
    }
};

/**
     * @brief Per-role output policy used when RCLink must fail safe.
     */
struct Failsafe
{
    enum class Mode : std::uint8_t
    {
        Value,        ///< Apply the configured numeric value.
        HoldLast,     ///< Hold the last acceptable filtered application value.
        ClampToOutLo, ///< Clamp an axis to its configured lower output.
        ClampToOutHi  ///< Clamp an axis to its configured upper output.
    };

    Mode mode{Mode::Value}; ///< Selected failsafe behaviour.
    std::int16_t value{0};  ///< Value used by @ref Mode::Value.
};

/**
     * @brief First configuration problem found while validating a link.
     */
enum class RcConfigError : std::uint8_t
{
    None,
    InvalidRole,
    UnmappedRequiredRole,
    InvalidChannel,
    InvalidLinkTimeout,
    InvalidAxisRange,
    InvalidAxisCenter,
    EndpointCenteredInversion,
    AxisSpanTooSmall,
    InvalidDeadband,
    InvalidOutputRange,
    NonFiniteValue,
    InvalidExpo,
    InvalidFilter,
    InvalidSwitchCount,
    InvalidSwitchLevels,
    InvalidSwitchSeparation,
    InvalidSwitchLearning
};

/**
     * @brief Result returned when a configuration is activated.
     */
struct RcConfigResult
{
    bool ok;              ///< True when the complete configuration is usable.
    RcConfigError error;  ///< First validation failure, or None.
    std::size_t role;     ///< Logical role index associated with the error.
    std::uint8_t channel; ///< Receiver channel associated with the error, when applicable.

    /// @brief Construct a configuration result with optional failure context.
    RcConfigResult(bool success = false, RcConfigError why = RcConfigError::None, std::size_t role_index = 0u,
                   std::uint8_t receiver_channel = kInvalidChannel) noexcept
        : ok(success), error(why), role(role_index), channel(receiver_channel)
    {
    }

    /// @brief Return true when configuration activation succeeded.
    explicit operator bool() const noexcept
    {
        return ok;
    }
};

/**
     * @brief Result returned when configuring a receiver failsafe signature.
     */
enum class RcSignatureError : std::uint8_t
{
    None,
    EmptyMask
};

/**
     * @brief Result returned when configuring a receiver failsafe signature.
     */
struct RcSignatureResult
{
    bool ok;                ///< True when the signature was accepted.
    RcSignatureError error; ///< Validation result.

    /// @brief Construct a receiver-signature validation result.
    RcSignatureResult(bool success = false, RcSignatureError why = RcSignatureError::None) noexcept
        : ok(success), error(why)
    {
    }

    /// @brief Return true when the receiver signature was accepted.
    explicit operator bool() const noexcept
    {
        return ok;
    }
};

/**
     * @brief Unified runtime state for the receiver link.
     *
     * `link_ok` answers only whether an accepted transport frame is fresh.
     * `healthy` additionally requires a valid RCLink configuration, all required
     * roles to be valid, and no receiver/protocol failsafe condition.
     */
struct RcLinkStatus
{
    bool has_frame{false};            ///< At least one transport frame has been accepted.
    bool config_valid{false};         ///< The active logical-role configuration passed validation.
    bool link_ok{false};              ///< Last accepted transport frame is within the link timeout.
    bool frame_valid{false};          ///< Latest accepted frame contained every required logical role.
    bool required_roles_valid{false}; ///< Every required role currently maps to valid channel data.
    bool healthy{false};              ///< Link is fresh, configured, role-valid, and not in failsafe.
    bool proto_failsafe{false};       ///< Protocol-level receiver failsafe flag.
    bool frame_lost{false};           ///< Protocol frame-lost indication, when available.
    bool rx_failsafe_sig{false};      ///< Configured receiver failsafe signature has been confirmed.

    std::uint8_t channel_count{0};       ///< Channels available in the most recent accepted transport frame.
    std::uint32_t channel_valid_mask{0}; ///< Per-channel validity bits for channels 0..31.
    std::uint32_t frame_sequence{0};     ///< RCLink-owned accepted-frame sequence; wraps naturally at uint32_t max.
    std::uint16_t fps{0};                ///< Estimated accepted frames per second.
    std::uint32_t frames{0};             ///< Optional transport-reported frame diagnostic; zero when unavailable.
    std::uint32_t crc_errors{0};         ///< Checksum/footer failures reported by the transport.
    std::uint32_t parse_errors{0};       ///< Framing/format errors reported by the transport.
    std::uint32_t parser_timeouts{0};    ///< Partial-frame inter-byte timeout resets.
    std::uint32_t discarded_bytes{0};    ///< Bytes discarded while re-synchronizing.
    std::uint32_t last_frame_ms{0};      ///< Timestamp of the last accepted transport frame.
    std::uint32_t last_frame_age{kInvalidFrameAgeMs}; ///< Frame age, or @ref kInvalidFrameAgeMs before the first frame.
    std::int8_t rssi_dbm{static_cast<std::int8_t>(-128)}; ///< RSSI when known.
    std::uint8_t lq{255};                                 ///< Link quality percentage; 255 means unknown.
};

/**
     * @brief Latest application-space values for N logical roles.
     */
template <std::size_t N> struct RcFrame
{
    std::int16_t vals[N]{}; ///< Scaled/safe values in logical-role order.
};

/**
     * @brief Receiver-failsafe signature rule in scaled application space.
     */
template <std::size_t N> struct RcFailsafeRule
{
    std::int16_t expected[N]{}; ///< Expected output for each checked role.
    std::uint8_t check[N]{};    ///< Non-zero selects that role for matching.
    std::uint16_t tol{2};       ///< Allowed absolute difference in scaled units.
    std::uint16_t hold_ms{120}; ///< Required duration across matching frames.
};
} // namespace rc
