/*
 * types.h
 *
 * Author: Florian Frieß
 * Copyright (C) 2021 by Universitaet Stuttgart (VISUS).
 * All rights reserved.
 */

#ifndef ENCODER_TYPES_H_INCLUDED
#define ENCODER_TYPES_H_INCLUDED
#if (defined(_MSC_VER) && (_MSC_VER > 1000))
#pragma once
#endif /* (defined(_MSC_VER) && (_MSC_VER > 1000)) */

#include "vislib/StringTokeniser.h"

namespace megamol {
namespace encoder {

    /**
     * The definition of the interval containing the QP values for the foveated encoding.
     */
    struct FoveatedInterval {
        /**
         * Create the default interval.
         */
        FoveatedInterval(void) : High(11u), Low(51u), Medium(31u) { }

        /**
         * Create the interval based on the given values.
         */
        FoveatedInterval(const uint32_t high, const uint32_t low, const uint32_t medium)
                : High(high), Low(low), Medium(medium) {}

        /**
         * Create the interval based on the comma separated string.
         */
        FoveatedInterval(const vislib::TString& interval) {
            // Split the string into its three components and check if there are enough.
            vislib::Array<vislib::TString> values = vislib::TStringTokeniser::Split(interval, ',', true);
            if (values.Count() != 2) {
                megamol::core::utility::log::Log::DefaultLog.WriteMsg(megamol::core::utility::log::Log::LEVEL_WARN,
                    "Failed to create FoveatedInterval from input %s using default interval [11,51] instead.",
                    interval);
                this->High = 11u;
                this->Low = 51u;
                this->Medium = 31u;
            }

            // Convert the values.
            this->High = static_cast<uint32_t>(std::stoi(values[0].PeekBuffer()));
            this->Low = static_cast<uint32_t>(std::stoi(values[1].PeekBuffer()));

            // Make sure they are in the correct value range [1,51].
            if (this->Low < 0u || this->Low > 51u) {
                uint32_t newLow = std::max(0u, std::min(this->Low, 51u));
                megamol::core::utility::log::Log::DefaultLog.WriteMsg(megamol::core::utility::log::Log::LEVEL_WARN,
                    "The low value %u of the interval is not within [1,51] clamping it to %u", this->Low, newLow);
                this->Low = newLow;
            }
            if (this->High < 0u || this->High > 51u) {
                uint32_t newHigh = std::max(0u, std::min(this->High, 51u));
                megamol::core::utility::log::Log::DefaultLog.WriteMsg(megamol::core::utility::log::Log::LEVEL_WARN,
                    "The high value %u of the interval is not within [1,51] clamping it to %u", this->High, newHigh);
                this->High = newHigh;
            }

            // Compute the medium value.
            this->Medium = this->Low + (this->Low - this->High) / 2;
        }

        /**
         * The high QP value for the interval, must be between 1 and 51.
         */
        uint32_t High;

        /**
         * The low QP value for the interval, must be between 1 and 51.
         */
        uint32_t Low;

        /**
         * The medium QP value for the interval, must be between 1 and 51.
         */
        uint32_t Medium;
    };

    /**
     * Struct that is used by the AbstractVideoEncoder to define a point.
     */
    struct PointF {
        /**
         * The x coordinate of the point.
         */
        float x;

        /**
         * The y coordinate of the point.
         */
        float y;
    };

    /**
     * The definition of the Fov tracking rectangle in screen boundaries [0,1].
     */
    struct FovRect {
        /**
         * The left top corner of the Fov rectangle.
         */
        PointF leftTop;

        /**
         * The left bottom corner of the Fov rectangle.
         */
        PointF leftBottom;

        /**
         * The right top corner of the Fov rectangle.
         */
        PointF rightTop;

        /**
         * The right bottom corner of the Fov rectangle.
         */
        PointF rightBottom;
    };

    /**
     * The definition of the Fov tracking rectangle in screen resolution.
     */
    struct ScaledFovRect {
        /**
         * Create the default rectangle.
         */
        ScaledFovRect(void) : corners() { }

        /**
         * Contains the corner of the rectangle in the order: left top, left bottom, right bottom, right top.
         */
        std::array<PointF, 4> corners;
    };

} // namespace encoder
} /* end namespace megamol */

#endif /* ENCODER_TYPES_H_INCLUDED */
