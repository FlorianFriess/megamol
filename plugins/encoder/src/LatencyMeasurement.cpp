/*
 * LatencyMeasurement.cpp
 *
 * Author: Florian Frieß
 * Copyright (C) 2021 by Universitaet Stuttgart (VISUS).
 * All rights reserved.
 */

#include "LatencyMeasurement.h"


/*
 * megamol::encoder::LatencyMeasurement::~LatencyMeasurement
 */
megamol::encoder::LatencyMeasurement::~LatencyMeasurement(void) {
    // Stop the print thread.
    this->isRunning.store(false);
    {
        std::lock_guard<std::mutex> lock(this->printDataMutex);
        this->printDataState = true;
    }
    this->printDataEvent.notify_one();
    if (this->printThread.joinable()) {
        this->printThread.join();
    }

    // Check if the file needs to be closed.
    if (this->csvFile != nullptr) {
        // Close the file.
        ::fclose(this->csvFile);

        // Check if the file is empty and can be deleted.
        if (this->canDelete) {
            // Delete file and ignore return value...
            std::remove(this->csvFileName.c_str());
        }
    }
}


/*
 * megamol::encoder::LatencyMeasurement::AddMeasurement
 */
void megamol::encoder::LatencyMeasurement::AddMeasurement(const double diff) {
    // Add the new value to the moving average and inrease the counter.
    this->storageBuffer[this->writeIdx][this->measurementCnt++] = diff;

    // Check if the moving average is "full".
    auto storageSize = this->storageBuffer[this->writeIdx].size();
    if (this->measurementCnt % storageSize == 0) {
        // Reset the counter and return.
        this->measurementCnt = 0;

        // Check if the write index can be advanced.
        auto curIdx = this->writeIdx.load();
        auto nxtIdx = this->advanceIdx(curIdx);
        if (nxtIdx != this->readIdx.load()) {
            this->writeIdx.store(nxtIdx);
        } else {
            megamol::core::utility::log::Log::DefaultLog.WriteMsg(
                megamol::core::utility::log::Log::LEVEL_WARN, "Input ring buffer is full, dropping data.", nullptr);
        }

        // Inform the print thread that there is new data to print.
        {
            std::lock_guard<std::mutex> lock(this->printDataMutex);
            this->printDataState = true;
        }
        this->printDataEvent.notify_one();
    }
}


/*
 * megamol::encoder::LatencyMeasurement::advanceIdx
 */
size_t megamol::encoder::LatencyMeasurement::advanceIdx(const size_t idx) const {
    return ((idx + 1) % this->storageBuffer.size());
}


/*
 * megamol::encoder::LatencyMeasurement::computeAvg
 */
double megamol::encoder::LatencyMeasurement::computeAvg(void) {
    // Compute the sum of the measurements.
    auto sum =
        std::accumulate(this->storageBuffer[this->readIdx].begin(), this->storageBuffer[this->readIdx].end(), 0.0);

    // Get the average and return it.
    return sum / static_cast<double>(this->storageBuffer[this->readIdx].size());
}


/*
 * megamol::encoder::LatencyMeasurement::computeMAD
 */
double megamol::encoder::LatencyMeasurement::computeMAD(const double median) {
    // Compute the absolute deviations about the median and sort the resulting vector to get the median absolute
    // deviation.
    for (size_t i = 0; i < this->storageBuffer[this->readIdx].size(); ++i) {
        this->storageBuffer[this->readIdx][i] = std::abs(median - this->storageBuffer[this->readIdx][i]);
    }
    std::sort(this->storageBuffer[this->readIdx].begin(), this->storageBuffer[this->readIdx].end());
    return this->computeMedian(this->storageBuffer[this->readIdx], this->storageBuffer[this->readIdx].size() / 2);
}


/*
 * megamol::encoder::LatencyMeasurement::computeMedian
 */
double megamol::encoder::LatencyMeasurement::computeMedian(const std::vector<double>& data, const size_t halfSize) {
    // Get the size and check if it is valid.
    const auto size = data.size();
    if (size == 0) {
        return 0ll;
    }

    // Compute the median, the given vector must be sorted already.
    double median = 0.0;
    if (size % 2 == 0) {
        median = (data[halfSize - 1] + data[halfSize]) / 2;
    } else {
        median = data[halfSize];
    }
    return median;
}


/*
 * megamol::encoder::LatencyMeasurement::Initialise
 */
bool megamol::encoder::LatencyMeasurement::Initialise(const size_t movingAvgSize, const std::string& name) {
    // Check if the instance has already been initialised.
    if (this->csvFile != nullptr) {
        megamol::core::utility::log::Log::DefaultLog.WriteMsg(
            megamol::core::utility::log::Log::LEVEL_WARN, "%s is already initialised.", this->csvFileName.c_str());
        return false;
    }

    // Potentially resize the storage.
    for (auto& storage : this->storageBuffer) {
        storage.resize(movingAvgSize);
    }

    // Create the CSV file that will contain the measurement info.
    this->csvFile = nullptr;
    if (::fopen_s(&this->csvFile, name.c_str(), "w") != 0) {
        megamol::core::utility::log::Log::DefaultLog.WriteMsg(
            megamol::core::utility::log::Log::LEVEL_ERROR, "Failed to open CSV file %s.", name.c_str());
        return false;
    }

    // Create the header for the file.
    std::string buffer = "AVG;MIN;MAX;MED;MAD\n";
    if (::fwrite(buffer.c_str(), buffer.size(), 1, this->csvFile) != 1) {
        megamol::core::utility::log::Log::DefaultLog.WriteMsg(
            megamol::core::utility::log::Log::LEVEL_ERROR, "Failed to write header for CSV file %s.", name.c_str());
        ::fclose(this->csvFile);
        this->csvFile = nullptr;
        return false;
    }

    // Flush the file in order to write the header.
    if (::fflush(this->csvFile) == EOF) {
        megamol::core::utility::log::Log::DefaultLog.WriteMsg(
            megamol::core::utility::log::Log::LEVEL_ERROR, "Failed to flush CSV file %s.", name.c_str());
        ::fclose(this->csvFile);
        this->csvFile = nullptr;
        return false;
    }

    // Start the print thread.
    this->isRunning.store(true);
    this->printThread = std::thread(std::bind(&LatencyMeasurement::printData, std::ref(*this)));

    // Remeber the file name.
    this->csvFileName = name;
    return true;
}


/*
 * megamol::encoder::LatencyMeasurement::LatencyMeasurement
 */
megamol::encoder::LatencyMeasurement::LatencyMeasurement(void)
        : canDelete(true)
        , csvFile(nullptr)
        , csvFileName("")
        , isRunning(false)
        , measurementCnt(0)
        , printDataEvent()
        , printDataMutex()
        , printDataState(false)
        , printThread()
        , storageBuffer(64)
        , readIdx(0)
        , writeIdx(0) {}


/*
 * megamol::encoder::LatencyMeasurement::printData
 */
void megamol::encoder::LatencyMeasurement::printData(void) {
    // Print data until the programme terminates.
    while (this->isRunning.load()) {
        // Wait until data is ready.
        std::unique_lock<std::mutex> lock(this->printDataMutex);
        this->printDataEvent.wait(lock, [&] { return this->printDataState; });
        this->printDataState = false;
        lock.unlock();

        // Get the next block of data.
        auto curIdx = this->readIdx.load();
        if (curIdx != this->writeIdx.load()) {
            // Check if the file is still open.
            if (this->csvFile == nullptr) {
                // Advance the read index.
                this->readIdx.store(this->advanceIdx(curIdx));
                megamol::core::utility::log::Log::DefaultLog.WriteMsg(megamol::core::utility::log::Log::LEVEL_ERROR,
                    "CSV file %s not open, no data will be written.", this->csvFileName.c_str());
                return;
            }

            // Sort the measurements.
            std::sort(this->storageBuffer[curIdx].begin(), this->storageBuffer[curIdx].end());

            // Get the computed values for the subscriber.
            auto avg = this->computeAvg();
            auto min = this->storageBuffer[curIdx].front();
            auto max = this->storageBuffer[curIdx].back();
            auto med = this->computeMedian(this->storageBuffer[curIdx], this->storageBuffer[curIdx].size() / 2);
            auto mad = this->computeMAD(med);

            // Advance the read index.
            this->readIdx.store(this->advanceIdx(curIdx));

            // Write the data to the CSV file.
            std::string buffer = std::to_string(avg) + ";" + std::to_string(min) + ";" + std::to_string(max) + ";" +
                                 std::to_string(med) + ";" + std::to_string(mad) + "\n";
            if (::fwrite(buffer.c_str(), buffer.size(), 1, this->csvFile) != 1) {
                megamol::core::utility::log::Log::DefaultLog.WriteMsg(megamol::core::utility::log::Log::LEVEL_ERROR,
                    "Failed to write data for CSV file %s.", this->csvFileName.c_str());
                ::fclose(this->csvFile);
                this->csvFile = nullptr;
                return;
            }
            this->canDelete = false;

            // Flush the file in order to write the last round.
            if (::fflush(this->csvFile) == EOF) {
                megamol::core::utility::log::Log::DefaultLog.WriteMsg(megamol::core::utility::log::Log::LEVEL_ERROR,
                    "Failed to flush CSV file %s.", this->csvFileName.c_str());
                ::fclose(this->csvFile);
                this->csvFile = nullptr;
                return;
            }
        }
    }
}
