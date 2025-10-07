/**
 * MIT License
 *
 * @brief Role helpers, sketch macros, and failsafe convenience builders for RcLink.
 *
 * @file RcMacros.hpp
 * @author
 * Little Man Builds (Darren Osborne)
 * @date 2025-10-03
 * @copyright © 2025 Little Man Builds
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <initializer_list>
#include <utility>
#include <Types.hpp>

// ---- Forward declaration so qualified lookup in template succeeds ---- //
namespace rc
{
    template <typename EnumLike>
    inline const char *to_string(EnumLike);
}

// ---- Internal token helpers (do not use directly) ---- //
#define RC__ENUM_VALUE(Name) Name,
#define RC__ENUM_NAME(Name) #Name,

/**
 * @brief Declare an enum class from a role list and generate name/iteration helpers.
 * @details Expands to:
 * - `rc::to_string(EnumName)` → `const char*` role name
 * - `rc::role_count(EnumName)` → number of roles (`constexpr`)
 * - `rc::role_by_index(EnumName, i)` → enum from ordinal index
 * @code
 * #define MY_ROLES(_) \
 *   _(A) _(B) _(C)
 * RC_DECLARE_ROLES(MyEnum, MY_ROLES)
 * @endcode
 */
#define RC_DECLARE_ROLES(EnumName, ROLE_LIST)                                          \
    enum class EnumName : std::uint8_t                                                 \
    {                                                                                  \
        ROLE_LIST(RC__ENUM_VALUE)                                                      \
            Count                                                                      \
    };                                                                                 \
    namespace rc                                                                       \
    {                                                                                  \
        /* Names table at namespace scope (C++11/14 friendly, no inline variables). */ \
        static const char *const EnumName##_names[] = {ROLE_LIST(RC__ENUM_NAME)};      \
                                                                                       \
        /** @brief Get textual name for a role. */                                     \
        inline const char *to_string(EnumName r)                                       \
        {                                                                              \
            return EnumName##_names[static_cast<std::size_t>(r)];                      \
        }                                                                              \
                                                                                       \
        /* Template specialization so calls bound at template-definition time to */    \
        /* rc::to_string<EnumName>(EnumName) link correctly. */                        \
        template <>                                                                    \
        inline const char *to_string<EnumName>(EnumName r)                             \
        {                                                                              \
            return EnumName##_names[static_cast<std::size_t>(r)];                      \
        }                                                                              \
                                                                                       \
        /** @brief Number of roles for this enum. */                                   \
        inline constexpr std::size_t role_count(EnumName /*tag*/)                      \
        {                                                                              \
            return static_cast<std::size_t>(EnumName::Count);                          \
        }                                                                              \
                                                                                       \
        /** @brief Role value by ordinal index [0..Count-1]. */                        \
        inline constexpr EnumName role_by_index(EnumName /*tag*/, std::size_t i)       \
        {                                                                              \
            return static_cast<EnumName>(i);                                           \
        }                                                                              \
    } /* namespace rc */

// ---- Public sketch-friendly macros ---- //

/**
 * @brief Declare a config object with simple spelling.
 * @details RC_CONFIG(Flysky, cfg) -> rc::RcConfig<Flysky> cfg;
 */
#define RC_CONFIG(EnumName, VarName) rc::RcConfig<EnumName> VarName

/**
 * @brief Map roles to receiver channels in order starting at BASE_CH.
 * @details Consistent ordering: (EnumName, CFG, BASE_CH).
 * @code
 * RC_CFG_MAP_RANGE(Flysky, cfg, 0);  // role0->ch0, role1->ch1, ...
 * @endcode
 */
#define RC_CFG_MAP_RANGE(EnumName, CFG, BASE_CH)                                           \
    do                                                                                     \
    {                                                                                      \
        for (std::size_t i = 0; i < static_cast<std::size_t>(EnumName::Count); ++i)        \
        {                                                                                  \
            (CFG).map(static_cast<EnumName>(i), static_cast<std::uint8_t>((BASE_CH) + i)); \
        }                                                                                  \
    } while (0)

/**
 * @brief Map roles to channels in declared order with base = 0.
 * @details Consistent ordering: (EnumName, CFG).
 * @code
 * RC_CFG_MAP_DEFAULT(Flysky, cfg);
 * @endcode
 */
#define RC_CFG_MAP_DEFAULT(EnumName, CFG) RC_CFG_MAP_RANGE(EnumName, CFG, 0)

// ---- Print helpers (link or frame) ---- //
namespace rc
{
    /**
     * @brief Print all roles/values from a link.
     */
    template <class Transport, typename E>
    static inline void print_all(const RcLink<Transport, E> &link, E)
    {
        for (std::size_t i = 0; i < static_cast<std::size_t>(E::Count); ++i)
        {
            const auto role = static_cast<E>(i);
            Serial.print(rc::to_string(role));
            Serial.print(": ");
            Serial.print(link.read(role));
            if (i + 1 != static_cast<std::size_t>(E::Count))
                Serial.print(", ");
        }
        Serial.println();
    }

    /**
     * @brief Print all roles/values from a frame snapshot.
     */
    template <typename E, std::size_t N>
    static inline void print_all(const RcFrame<N> &frame, E)
    {
        for (std::size_t i = 0; i < static_cast<std::size_t>(E::Count); ++i)
        {
            const auto role = static_cast<E>(i);
            Serial.print(rc::to_string(role));
            Serial.print(": ");
            Serial.print(frame.vals[i]);
            if (i + 1 != static_cast<std::size_t>(E::Count))
                Serial.print(", ");
        }
        Serial.println();
    }
} ///< namespace rc.

/**
 * @brief Print "RoleName: value, RoleName: value, ..." for all roles, then newline.
 * @details Accepts either a RcLink<...> or an RcFrame<N> thanks to overloads above.
 * @code
 * RC_PRINT_ALL(rclink, Flysky);
 * RC_PRINT_ALL(rclink.failsafe_expected_frame(), Flysky);
 * @endcode
 */
#define RC_PRINT_ALL(LINK_OR_FRAME, EnumName) ::rc::print_all((LINK_OR_FRAME), EnumName{})

/**
 * @brief Iterate roles and run a simple statement block (no lambdas/templates).
 * @code
 * RC_FOR_EACH_ROLE(Flysky, {
 *   // available: role (EnumName), i (std::size_t)
 *   Serial.println(rc::to_string(role));
 * });
 * @endcode
 */
#define RC_FOR_EACH_ROLE(EnumName, BODY_BLOCK)                                      \
    do                                                                              \
    {                                                                               \
        for (std::size_t i = 0; i < static_cast<std::size_t>(EnumName::Count); ++i) \
        {                                                                           \
            const EnumName role = static_cast<EnumName>(i);                         \
            BODY_BLOCK                                                              \
        }                                                                           \
    } while (0)

/**
 * @brief Failsafe convenience builders (signature rules) and one-liner macros.
 * @details These helpers construct `RcFailsafeRule` instances sized to role enum,
 *          then apply them via `RcLink::set_failsafe_signature()`. They operate on the
 *          **scaled** space (application values), not raw microseconds.
 * @note E must be an enum declared via RC_DECLARE_ROLES and end with `Count`.
 */
namespace rc
{
    /**
     * @brief Shorthand alias for a failsafe rule sized to an enum E.
     * @tparam E  Role enum type (must end with Count).
     */
    template <typename E>
    using FsRule = RcFailsafeRule<static_cast<std::size_t>(E::Count)>;

    /**
     * @brief Build a signature that checks ALL roles against the same expected value.
     * @tparam E Role enum type.
     * @param tag Enum tag instance, e.g. Flysky{} (type-only).
     * @param expected Expected value (scaled units) for every role.
     * @param tol Tolerance (± in scaled counts).
     * @param hold_ms Must match for at least this many milliseconds.
     * @return FsRule<E> Configured rule.
     * @code
     * // Everyone expected at zero (±3) for ≥250 ms:
     * auto sig = rc::make_signature_all(Flysky{}, 0, 3, 250);
     * @endcode
     */
    template <typename E>
    FsRule<E> make_signature_all(E /*tag*/,
                                 std::int16_t expected,
                                 std::uint16_t tol,
                                 std::uint16_t hold_ms)
    {
        FsRule<E> sig{};
        const std::size_t n = static_cast<std::size_t>(E::Count);
        for (std::size_t i = 0; i < n; ++i)
        {
            sig.check[i] = true;
            sig.expected[i] = expected;
        }
        sig.tol = tol;
        sig.hold_ms = hold_ms;
        return sig;
    }

    /**
     * @brief Build a signature with a default expected value, plus per-role overrides.
     * @tparam E Role enum type.
     * @param tag Enum tag instance (type-only).
     * @param default_expected Default expected value (scaled units) for all roles.
     * @param tol Tolerance (± in scaled counts).
     * @param hold_ms Must match for at least this many milliseconds.
     * @param overrides List of {role, expected} pairs to override specific roles.
     * @return FsRule<E> Configured rule.
     * @code
     * auto sig = rc::make_signature_with_overrides(
     *   Flysky{}, [default=0], [tol=3], [hold_ms=250],
     *   { {Flysky::Ch3_LV, 10} } // throttle = 10; all others use default
     * );
     * @endcode
     */
    template <typename E>
    FsRule<E> make_signature_with_overrides(
        E /*tag*/,
        std::int16_t default_expected,
        std::uint16_t tol,
        std::uint16_t hold_ms,
        std::initializer_list<std::pair<E, std::int16_t>> overrides)
    {
        FsRule<E> sig = make_signature_all<E>(E{}, default_expected, tol, hold_ms);
        for (const auto &p : overrides)
        {
            const auto idx = static_cast<std::size_t>(p.first);
            if (idx < static_cast<std::size_t>(E::Count))
            {
                sig.check[idx] = true;
                sig.expected[idx] = p.second;
            }
        }
        return sig;
    }

    /**
     * @brief Build a signature that checks ONLY the provided roles (others ignored).
     * @tparam E Role enum type.
     * @param tag Enum tag instance (type-only).
     * @param tol Tolerance (± in scaled counts).
     * @param hold_ms Must match for at least this many milliseconds.
     * @param selected List of {role, expected} pairs to check. All others ignored.
     * @return FsRule<E> Configured rule.
     * @code
     * auto sig = rc::make_signature_selected(
     *   Flysky{}, [tol=3], [hold_ms=250],
     *   { {Flysky::Ch1_RH, 0}, {Flysky::Ch2_RV, 0},
     *     {Flysky::Ch3_LV, 0}, {Flysky::Ch4_LH, 0} }
     * );
     * @endcode
     */
    template <typename E>
    FsRule<E> make_signature_selected(
        E /*tag*/,
        std::uint16_t tol,
        std::uint16_t hold_ms,
        std::initializer_list<std::pair<E, std::int16_t>> selected)
    {
        FsRule<E> sig{};
        const std::size_t n = static_cast<std::size_t>(E::Count);
        for (std::size_t i = 0; i < n; ++i)
        {
            sig.check[i] = false;
            sig.expected[i] = 0;
        }
        for (const auto &p : selected)
        {
            const auto idx = static_cast<std::size_t>(p.first);
            if (idx < n)
            {
                sig.check[idx] = true;
                sig.expected[idx] = p.second;
            }
        }
        sig.tol = tol;
        sig.hold_ms = hold_ms;
        return sig;
    }

} ///< namespace rc.

// ---- Public sketch-friendly macros ---- //

/**
 * @brief Apply an "all roles == expected" signature.
 * @details Convenience wrapper over rc::make_signature_all + RcLink::set_failsafe_signature().
 * @code
 * RC_SET_SIGNATURE_ALL(Flysky, rclink, [expected=0], [tol=3], [hold_ms=250]);
 * @endcode
 */
#define RC_SET_FS_SIGNATURE_ALL(EnumName, LINK, EXPECTED, TOL, HOLD_MS)            \
    do                                                                             \
    {                                                                              \
        auto _sig = ::rc::make_signature_all(EnumName{}, (EXPECTED),               \
                                             static_cast<std::uint16_t>(TOL),      \
                                             static_cast<std::uint16_t>(HOLD_MS)); \
        (LINK).set_failsafe_signature(_sig);                                       \
    } while (0)

/**
 * @brief Apply "default expected + per-role overrides".
 * @details Convenience wrapper over rc::make_signature_with_overrides + RcLink::set_failsafe_signature().
 * @code
 * RC_SET_SIGNATURE_OVERRIDES(Flysky, rclink, [default=0], 3, 250,
 *   { {Flysky::Ch3_LV, 10} }); // throttle 10, others 0
 * @endcode
 */
#define RC_SET_FS_SIGNATURE_OVERRIDES(EnumName, LINK, DEFAULT_EXPECTED, TOL, HOLD_MS, ...) \
    do                                                                                     \
    {                                                                                      \
        auto _sig = ::rc::make_signature_with_overrides(                                   \
            EnumName{}, (DEFAULT_EXPECTED),                                                \
            static_cast<std::uint16_t>(TOL), static_cast<std::uint16_t>(HOLD_MS),          \
            __VA_ARGS__);                                                                  \
        (LINK).set_failsafe_signature(_sig);                                               \
    } while (0)

/**
 * @brief Apply "only these roles" (others ignored).
 * @details Convenience wrapper over rc::make_signature_selected + RcLink::set_failsafe_signature().
 * @code
 * RC_SET_SIGNATURE_SELECTED(Flysky, rclink, 3, 250,
 *   { {Flysky::Ch1_RH, 0}, {Flysky::Ch2_RV, 0}, {Flysky::Ch3_LV, 0}, {Flysky::Ch4_LH, 0} });
 * @endcode
 */
#define RC_SET_FS_SIGNATURE_SELECTED(EnumName, LINK, TOL, HOLD_MS, ...) \
    do                                                                  \
    {                                                                   \
        auto _sig = ::rc::make_signature_selected(                      \
            EnumName{}, static_cast<std::uint16_t>(TOL),                \
            static_cast<std::uint16_t>(HOLD_MS),                        \
            __VA_ARGS__);                                               \
        (LINK).set_failsafe_signature(_sig);                            \
    } while (0)
