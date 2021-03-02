/*
 * LatencyMeasurement.h
 *
 * Author: Florian Frieß
 * Copyright (C) 2021 by Universitaet Stuttgart (VISUS).
 * All rights reserved.
 */

#ifndef ENCODER_LATENCYMEASUREMENT_H_INCLUDED
#define ENCODER_LATENCYMEASUREMENT_H_INCLUDED
#if (defined(_MSC_VER) && (_MSC_VER > 1000))
#pragma once
#endif /* (defined(_MSC_VER) && (_MSC_VER > 1000)) */

#include "mmcore/utility/log/Log.h"

#include "vislib/sys/Event.h"

#include <algorithm>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

namespace megamol {
namespace encoder {

    /**
     * Class that offers CPU based video encoders
     */
    class LatencyMeasurement {
    public:
        /** Dtor */
        ~LatencyMeasurement(void);

        /**
         * Add the difference to the storage.
         *
         * @param diff The time in ms.
         */
        void AddMeasurement(const double diff);

        /**
         * Initialise the measurements.
         *
         * @param movingAvgSize The number of measurements befor the average, median, minimum, maximum and the
         * MAD is printed.
         * @param name The (unique) name of the CSV file that will contain the measurement data.
         *
         * @returns 'false' in case the CSV file was not created, no data could be written to it or the instance
         * was already initialised, 'true' otherwise.
         */
        bool Initialise(const size_t movingAvgSize, const std::string& name);

        /** Ctor */
        LatencyMeasurement(void);

    private:
        /**
         * Advance the index by one modulo size of storageBuffer.
         *
         * @param idx The index to advance by one.
         *
         * @returns The given index +1 modulo size of storageBuffer.
         */
        size_t advanceIdx(const size_t idx) const;

        /**
         * Compute the average of the measurements.
         */
        double computeAvg(void);

        /**
         * Computes the median absolute deviation.
         *
         * @param median The median of the data.
         *
         * @returns The MAD (https://en.wikipedia.org/wiki/Median_absolute_deviation).
         */
        double computeMAD(const double median);

        /**
         * Compute the median of the given data.
         *
         * @returns The median of the input data.
         */
        double computeMedian(const std::vector<double>& data, const size_t halfSize);

        /**
         * Print the data of the last round.
         */
        void printData(void);

        /** Flag that indicates if the csvFile is empty and can therefore be deleted. */
        bool canDelete;

        /** The handler for the CSV file. */
        FILE* csvFile;

        /** The name of the CSV file. */
        std::string csvFileName;

        /** Controls the lifetime of the printThread. */
        std::atomic<bool> isRunning;

        /** The number of measurements in the storage. */
        size_t measurementCnt;

        /** The event that is signaled once enough measurements have been collected. */
        vislib::sys::Event printDataEvent;

        /** The thread that prints the measurments. */
        std::thread printThread;

        /** The ring buffer that contains the measurements. */
        std::vector<std::vector<double>> storageBuffer;

        /** The read index of the storageBuffer. */
        std::atomic<size_t> readIdx;

        /** The write index of the storageBuffer. */
        std::atomic<size_t> writeIdx;
    };

} // namespace encoder
} /* end namespace megamol */

#endif /* ENCODER_LATENCYMEASUREMENT_H_INCLUDED */
