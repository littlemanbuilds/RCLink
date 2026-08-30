/**
 * MIT License
 *
 * @brief Public umbrella header for RCLink.
 *
 * @file RCLink.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-08
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */

#pragma once

// ---- Version ---- //

#define RCLINK_VERSION "1.2.0"
#define RCLINK_VERSION_MAJOR 1
#define RCLINK_VERSION_MINOR 2
#define RCLINK_VERSION_PATCH 0

// ---- Public API ---- //

#include "Config.hpp"
#include "Constants.hpp"
#include "Link.hpp"
#include "RcMacros.hpp"
#include "transport/Ibus.hpp"
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
#include "transport/Sbus_Esp32.hpp"
#endif
#include "Types.hpp"

/**
 * @brief Legacy global aliases retained for source compatibility with RCLink v1.x sketches.
 *
 * RCLink v1.2 no longer uses `using namespace rc;`, which previously imported every
 * RCLink symbol into the includer's global namespace. New code should prefer explicit
 * `rc::` qualification. Define `RCLINK_NO_LEGACY_GLOBAL_ALIASES` before including
 * this header to use the clean namespace-only API.
 */
#ifndef RCLINK_NO_LEGACY_GLOBAL_ALIASES
using RcTransportCaps = rc::RcTransportCaps;
using ChannelKind = rc::ChannelKind;
using RcAxisSpec = rc::RcAxisSpec;
using RcSwitchSpec = rc::RcSwitchSpec;
using RcChannelSpec = rc::RcChannelSpec;
using Failsafe = rc::Failsafe;
using RcLinkStatus = rc::RcLinkStatus;
using RcConfigError = rc::RcConfigError;
using RcConfigResult = rc::RcConfigResult;
using RcSignatureError = rc::RcSignatureError;
using RcSignatureResult = rc::RcSignatureResult;
using RcIbusTransport = rc::RcIbusTransport;

template <std::size_t N> using RcFrame = rc::RcFrame<N>;

template <std::size_t N> using RcFailsafeRule = rc::RcFailsafeRule<N>;

template <typename E> using RcConfig = rc::RcConfig<E>;

template <typename E> using FsRule = rc::FsRule<E>;

template <class Transport, typename E> using RcLink = rc::RcLink<Transport, E>;

using rc::make_signature_all;
using rc::make_signature_selected;
using rc::make_signature_with_overrides;

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
using RcSbusEsp32Transport = rc::RcSbusEsp32Transport;
#endif
#endif // RCLINK_NO_LEGACY_GLOBAL_ALIASES
