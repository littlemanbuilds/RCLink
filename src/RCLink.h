/**
 * MIT License
 *
 * @brief Umbrella header for RCLink.
 *
 * @file RCLink.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-03
 * @copyright © 2025 Little Man Builds
 */

#pragma once

// ---- Default includes ---- //
#include <Types.hpp>
#include <Constants.hpp>
#include <Config.hpp>
#include <Link.hpp>
#include <RcMacros.hpp>

// ---- Transports ---- //
#include <transport/Ibus.hpp>
#include <transport/Sbus_Esp32.hpp>

// ---- Version macro ---- //
#define RCLINK_VERSION "1.0.0"

using namespace rc; ///< Make rc:: types available without prefix.