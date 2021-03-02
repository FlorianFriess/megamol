/*
 * AbstractVideoEncoder.cpp
 *
 * Author: Florian Frieß
 * Copyright (C) 2021 by Universitaet Stuttgart (VISUS).
 * All rights reserved.
 */

#include "AbstractVideoEncoder.h"


/*
 * megamol::encoder::AbstractVideoEncoder::~AbstractVideoEncoder
 */
megamol::encoder::AbstractVideoEncoder::~AbstractVideoEncoder(void) {
    this->Release();
}


/*
 * megamol::encoder::AbstractVideoEncoder::AbstractVideoEncoder
 */
megamol::encoder::AbstractVideoEncoder::AbstractVideoEncoder(void)
        : megamol::core::view::Renderer3DModule_2()
        , active("active", "Bool parameter to play/stop the encoder")
        , avxSupport(false)
        , deleteOutfile(true)
        , encoderCodec("encoderCodec", "Defines the codec used by the encoder")
        , flipFrames("flipFrames", "The slot used to turn on/off the flipping of the frames before encoding.")
        , foveatedInterval("foveatedInterval", "The comma separated lower an upper value of the QP interval")
        , foveatedIntervalVal()
        , height(0)
        , inputMode("inputMode", "Switch between Call and MPI as input")
        , lastMousePos(0.0, 0.0)
        , latencyMeasurement("latencyMeasurement", "Turns on/off the latency measurements")
        , macroblockCentres()
        , macroblockCnt(0)
        , macroblockCntX(0)
        , macroblockCntY(0)
        , macroblockCorners()
        , macroblockHeight()
        , macroblockQualityMapping()
        , macroblockQualityReadIdx(0)
        , macroblockQualityWriteIdx(0)
        , macroblockWidth(0)
        , mpiInputThreadRunning(false)
        #ifdef WITH_NETWORK
        , mpiIntraCall("mpiIntraCall", "Call to send and receive data via the MpiIntraCommunicator")
        #endif
        , offsetHeight("offsetHeight","The height offset of the current node with respect to the overall display")
        , offsetWidth("offsetWidth","The width offset of the current node with respect to the overall display")
        , overallHeight("overallHeight","Stores the overall height, in pixel, of the display that is encoded")
        , overallHeightF(0.0f)
        , overallWidth("overallWidth","Stores the overall width, in pixel, of the display that is encoded")
        , overallWidthF(0.0f)
        , outFile(nullptr)
        , outFilenameSlot("outFilenameSlot", "The path to the file that will contain the encoded frames")
        , outputMode("outputMode", "Select what happens with the encoded frames")
        , previousEncoderCodec(1)
        , previousFlipFrames(false)
        , previousFoveated(false)
        , previousInputMode(0)
        , previousLatency(false)
        , previousOffsetHeight(0)
        , previousOffsetWidth(0)
        , previousOutFilename(".\\outfile.h265")
        , previousOutputMode(0)
        , previousOverallHeight(0)
        , previousOverallWidth(0)
        , previousQPValue(31)
        , previousSIMD(true)
        , previousUseInfiniteGOP(false)
        , qpValue("qpValue", "The QP value used by the encoder in case foveated encoding is not used")
        , sseSupport(false)
        , uniqueInputDataID(0)
        , uniqueInputFoveatedDataID(0)
        , uniqueOutputDataID(0)
        , useFoveated("useFoveated", "Turns on the foveated encdoing")
        , useSIMD("useSIMD", "Turns the usage of SIMD instructions on/off")
        , useInfiniteGOP("useInfiniteGOP", "Turns off periodic key frames and uses intra refresh")
        , width(0) {
    // Initialise the state of the encoder. The default is false (not recording).
    this->active << new megamol::core::param::BoolParam(false);
    this->active.SetUpdateCallback(&AbstractVideoEncoder::activeCallback);
    this->MakeSlotAvailable(&this->active);

    #ifdef WITH_NETWORK
    // Initialise the call to send and receive data via MPI.
    this->mpiIntraCall
        .SetCompatibleCall<megamol::core::factories::CallAutoDescription<megamol::network::MpiIntraCall>>();
    this->MakeSlotAvailable(&this->mpiIntraCall);
    #endif

    // Initialise the codes of the encoder. The default is h265(HEVC).
    this->encoderCodec << new megamol::core::param::EnumParam(1);
    this->encoderCodec.Param<megamol::core::param::EnumParam>()->SetTypePair(0, "h264");
    this->encoderCodec.Param<megamol::core::param::EnumParam>()->SetTypePair(1, "h265");
    this->encoderCodec.SetUpdateCallback(&AbstractVideoEncoder::unchangeableCallback);
    this->MakeSlotAvailable(&this->encoderCodec);

    // Initialise the flipping of the frames before encoding. The default is false (no flipping).
    this->flipFrames << new megamol::core::param::BoolParam(false);
    this->flipFrames.SetUpdateCallback(&AbstractVideoEncoder::unchangeableCallback);
    this->MakeSlotAvailable(&this->flipFrames);

    // Initialise the foveated interval. The default is [11,51].
    this->foveatedInterval << new megamol::core::param::StringParam("11,51");
    this->MakeSlotAvailable(&this->foveatedInterval);

    // Initialise the input mode of the encoder. The default is Call.
    this->inputMode << new megamol::core::param::EnumParam(0);
    this->inputMode.Param<megamol::core::param::EnumParam>()->SetTypePair(0, "Call");
    #ifdef WITH_NETWORK
    this->inputMode.Param<megamol::core::param::EnumParam>()->SetTypePair(1, "MPI");
    #endif
    this->inputMode.SetUpdateCallback(&AbstractVideoEncoder::unchangeableCallback);
    this->MakeSlotAvailable(&this->inputMode);

    // Initialise the state of the latency measurements. The default is false (no measurements).
    this->latencyMeasurement << new megamol::core::param::BoolParam(false);
    this->latencyMeasurement.SetUpdateCallback(&AbstractVideoEncoder::unchangeableCallback);
    this->MakeSlotAvailable(&this->latencyMeasurement);

    // Initialise the height offset of the node. The default is zero.
    this->offsetHeight << new megamol::core::param::IntParam(0, 0);
    this->offsetHeight.SetUpdateCallback(&AbstractVideoEncoder::unchangeableCallback);
    this->MakeSlotAvailable(&this->offsetHeight);

    // Initialise the width offset of the node. The default is zero.
    this->offsetWidth << new megamol::core::param::IntParam(0, 0);
    this->offsetWidth.SetUpdateCallback(&AbstractVideoEncoder::unchangeableCallback);
    this->MakeSlotAvailable(&this->offsetWidth);

    // Initialise the path to the file that will contain the encoded frames. The default is ".\\outfile.h265".
    this->outFilenameSlot << new megamol::core::param::FilePathParam(".\\outfile.h265");
    this->outFilenameSlot.SetUpdateCallback(&AbstractVideoEncoder::unchangeableCallback);
    this->MakeSlotAvailable(&this->outFilenameSlot);

    // Initialise the different output modes. The default is write to file.
    this->outputMode << new megamol::core::param::EnumParam(0);
    this->outputMode.Param<megamol::core::param::EnumParam>()->SetTypePair(0, "file");
    #ifdef WITH_NETWORK
    this->outputMode.Param<megamol::core::param::EnumParam>()->SetTypePair(1, "MPI");
    this->outputMode.Param<megamol::core::param::EnumParam>()->SetTypePair(2, "network");
    #endif
    this->outputMode.SetUpdateCallback(&AbstractVideoEncoder::unchangeableCallback);
    this->MakeSlotAvailable(&this->outputMode);

    // Initialise the overall height of the display. The default is zero.
    this->overallHeight << new megamol::core::param::IntParam(0, 0);
    this->overallHeight.SetUpdateCallback(&AbstractVideoEncoder::unchangeableCallback);
    this->MakeSlotAvailable(&this->overallHeight);

    // Initialise the overall width of the display. The default is zero.
    this->overallWidth << new megamol::core::param::IntParam(0, 0);
    this->overallWidth.SetUpdateCallback(&AbstractVideoEncoder::unchangeableCallback);
    this->MakeSlotAvailable(&this->overallWidth);

    // Initialise the QP value used by the encoder. The default is 31.
    this->qpValue << new megamol::core::param::IntParam(31, 1, 51);
    this->qpValue.SetUpdateCallback(&AbstractVideoEncoder::unchangeableCallback);
    this->MakeSlotAvailable(&this->qpValue);

    // Initialise the mode of the encoder. The default is to not use foveated encoding.
    this->useFoveated << new megamol::core::param::BoolParam(false);
    this->useFoveated.SetUpdateCallback(&AbstractVideoEncoder::unchangeableCallback);
    this->MakeSlotAvailable(&this->useFoveated);

    // Initialise the usage of SIMD instructions. The default is true, i.e. use SIMD.
    this->useSIMD << new megamol::core::param::BoolParam(true);
    this->useSIMD.SetUpdateCallback(&AbstractVideoEncoder::unchangeableCallback);
    this->MakeSlotAvailable(&this->useSIMD);

    // Initialise the usage of key frames. The default is to use key frames.
    this->useInfiniteGOP << new megamol::core::param::BoolParam(false);
    this->useInfiniteGOP.SetUpdateCallback(&AbstractVideoEncoder::unchangeableCallback);
    this->MakeSlotAvailable(&this->useInfiniteGOP);
}


/*
 * megamol::encoder::AbstractVideoEncoder::activeCallback
 */
bool megamol::encoder::AbstractVideoEncoder::activeCallback(megamol::core::param::ParamSlot& slot) {
    // Assert that the slot is the expected one.
    ASSERT(&slot == &this->active);

    // Get the value of the active state.
    auto isActive = this->active.Param<megamol::core::param::BoolParam>()->Value();
    if (isActive) {
        // Check for SIMD instructions.
        this->checkSIMDInstructions();

        // Initialise the latency measurements.
        this->initialiseBaseLatencyMeasurements();
        this->initialiseSpecialLatencyMeasurements();

        // Initialise the foveated encoding.
        this->initialiseFoveated();

        // Initialise the encoder.
        this->initialiseEncoder();

        // Initialise the MpiIntraCmmunicator.
        this->initialiseMPI();

        // Initialise the input.
        this->initialiseInput();

        // Initialise the ouput.
        this->initialiseOutput();

    } else {
        // Stop the encoder.
        this->stopEncoder();

        // Stop the MpiIntraCmmunicator.
        this->stopMPI();

        // Stop the input.
        this->stopInput();

        // Stop the ouput.
        this->stopOutput();
    }
    return true;
}


/*
 * megamol::encoder::AbstractVideoEncoder::checkMBIntersectionFrame
 */
void megamol::encoder::AbstractVideoEncoder::checkMBIntersectionFrame(
    const FovRect* lookAtRects, const std::uint32_t validElements, const size_t timestamp) {
    // Update the foveated interval.
    if (this->foveatedInterval.IsDirty()) {
        this->foveatedIntervalVal =
            FoveatedInterval(this->foveatedInterval.Param<megamol::core::param::StringParam>()->ValueString());
        this->foveatedInterval.ResetDirty();
    }

    // Convert the QP values to floats.
    auto highQP = static_cast<float>(this->foveatedIntervalVal.High);
    auto mediumQP = static_cast<float>(this->foveatedIntervalVal.Medium);
    auto lowQP = static_cast<float>(this->foveatedIntervalVal.Low);

    // Get the current write index.
    auto writeIdx = this->macroblockQualityWriteIdx.load();

    // Scale the incoming rectangles, since they are still in screen coordinates,
    // i.e. in [0,1] and check each macroblock if it is, at least partially, inside
    // the rectangle.
    size_t highIdx = 0;
    auto& curMap = this->macroblockQualityMapping[writeIdx];
    for (std::uint32_t i = 0; i < validElements; ++i) {
        // Get the current scaled look-at rectangle.
        auto rect = this->scaleRectangle(lookAtRects[i]);

        // Get the centre point of the rectangle.
        auto width = rect.corners[3].x - rect.corners[0].x;
        auto height = rect.corners[1].y - rect.corners[0].y;
        PointF centre;
        centre.x = rect.corners[0].x + (width / 2.0f);
        centre.y = rect.corners[0].y + (height / 2.0f);

        // Get the maximum distance.
        auto maxDist = std::sqrtf(std::powf(width / 2.0f, 2.0f) + std::powf(height / 2.0f, 2.0f));

        // Loop over the tiles and check if there are any intersections.
        for (size_t mbID = 0; mbID < this->macroblockCorners.size(); ++mbID) {
            // Check if the current macroblock is interseted by the rectangle.
            bool isInside = megamol::encoder::AbstractVideoEncoder::intersection(this->macroblockCorners[mbID], rect);
            if (isInside) {
                // Compute the distance between the macroblock and the centre of the
                // look-at rectangle and scale it to the interval [0,1].
                auto centreMB = this->macroblockCentres[mbID];
                auto dist = std::sqrtf(std::powf(centre.x - centreMB.x, 2.0f) + std::powf(centre.y - centreMB.y, 2.0f));
                dist /= maxDist;

                // Compute the quality value and transfrom it into the value range
                // of the quality parameter.
                dist = std::powf(1.0f - dist, 3.0f);
                float qpValue = static_cast<float>(lowQP) - static_cast<float>(std::roundf(dist * (lowQP - highQP)));

                // Compute the offset that is used for the macroblock.
                curMap[mbID] = qpValue - 51.0f;
            }
        }
    }

    // Switch the read and the write index for the macroblock quality mappings.
    // Reset the macroblock quality offset of the new write map to zero.
    this->macroblockLock.Lock();
    this->macroblockQualityWriteIdx.store(this->macroblockQualityReadIdx.load());
    this->macroblockQualityReadIdx.store(writeIdx);
    auto newWriteIDx = this->macroblockQualityWriteIdx.load();
    auto& newMap = this->macroblockQualityMapping[newWriteIDx];
    std::fill(newMap.begin(), newMap.end(), 0.0f);
    this->macroblockLock.Unlock();
}


/*
 * megamol::encoder::AbstractVideoEncoder::checkSIMDInstructions
 */
void megamol::encoder::AbstractVideoEncoder::checkSIMDInstructions(void) {
    // Note:
    // The supported instructions can be queried by calling __cpuid, see:
    // https://stackoverflow.com/questions/6121792/how-to-check-if-a-cpu-supports-the-sse3-instruction-set/22521619#22521619
    // for more details on how to do that.
    this->avxSupport = false;
    this->sseSupport = false;
    if (this->useSIMD.Param<megamol::core::param::BoolParam>()->Value()) {
        // Check AVX (256 bit) support.
        {
            std::array<int, 4> cpuInfo;
            __cpuid(cpuInfo.data(), 1);
            bool osUsesXSAVE_XRSTORE = cpuInfo[2] & (1 << 27) || false;
            bool cpuAVXSuport = cpuInfo[2] & (1 << 28) || false;
            if (osUsesXSAVE_XRSTORE && cpuAVXSuport) {
                unsigned long long xcrFeatureMask = _xgetbv(_XCR_XFEATURE_ENABLED_MASK);
                this->avxSupport = (xcrFeatureMask & 0x6) == 0x6;
            }

            __cpuid(cpuInfo.data(), 7);
            bool cpuAVX2Support = cpuInfo[1] & 0x00000020;
            this->avxSupport &= cpuAVX2Support;
            if (this->avxSupport) {
                megamol::core::utility::log::Log::DefaultLog.WriteInfo("The machine supports AVX (256 bit).");
            }
        }

        // Check SSE (128 bit) support.
        {
            std::array<int, 4> cpuInfo;
            __cpuid(cpuInfo.data(), 1);
            bool sse = (cpuInfo[3] & (1 << 25)) != 0;
            bool sse2 = (cpuInfo[3] & (1 << 26)) != 0;
            bool sse3 = (cpuInfo[2] & (1 << 0)) != 0;
            bool ssse3 = (cpuInfo[2] & (1 << 9)) != 0;
            bool sse41 = (cpuInfo[2] & (1 << 19)) != 0;
            bool sse42 = (cpuInfo[2] & (1 << 20)) != 0;
            this->sseSupport = cpuInfo[2] & (1 << 9) || cpuInfo[2] & (1 << 19) || cpuInfo[2] & (1 << 20) || false;
            if (this->sseSupport) {
                megamol::core::utility::log::Log::DefaultLog.WriteInfo("The machine supports SSE (128 bit).");
            }
        }
    }
}


/*
 * megamol::encoder::AbstractVideoEncoder::computeMacroblockCnt
 */
void megamol::encoder::AbstractVideoEncoder::computeMacroblockCnt(void) {
    // Set the macroblock size to 16 as this is the default.
    this->macroblockHeight = 16;
    this->macroblockWidth = 16;

    // Check the height and compute the number of macroblocks.
    if (static_cast<size_t>(this->height) % this->macroblockHeight != 0) {
        auto factor = std::ceil(static_cast<double>(this->height) / static_cast<double>(this->macroblockHeight));
        this->macroblockCntY = static_cast<size_t>(factor);

    } else {
        this->macroblockCntY = static_cast<size_t>(this->height) / this->macroblockHeight;
    }

    // Check the width and compute the number of macroblocks.
    if (static_cast<size_t>(this->width) % this->macroblockWidth != 0) {
        auto factor = std::ceil(static_cast<double>(this->width) / static_cast<double>(this->macroblockWidth));
        this->macroblockCntX = static_cast<size_t>(factor);

    } else {
        this->macroblockCntX = static_cast<size_t>(this->width) / this->macroblockWidth;
    }

    // Compute the number of macroblocks.
    this->macroblockCnt = this->macroblockCntX * this->macroblockCntY;
}


/*
 * megamol::encoder::AbstractVideoEncoder::computeMacroblockRects
 */
void megamol::encoder::AbstractVideoEncoder::computeMacroblockRects(const float offsetHeight, const float offsetWidth) {
    // Check if the frames are flipped.
    if (this->flipFrames.Param<megamol::core::param::BoolParam>()->Value()) {
        // Compute the corners of the macroblocks from top to bottom.
        this->macroblockCorners.resize(this->macroblockCnt);
        this->macroblockCentres.resize(this->macroblockCnt);
        for (size_t row = 0; row < this->macroblockCntY; ++row) {
            for (size_t col = 0; col < this->macroblockCntX; ++col) {
                // Get the ID of the macroblock.
                auto mbID = row * this->macroblockCntX + col;

                // Compute the corners and the centre of the rectangle.
                this->computeRect(row, col, mbID, offsetHeight, offsetWidth);
            }
        }

    } else {
        // Compute the corners of the macroblocks from bottom to top.
        int maxRow = static_cast<int>(this->macroblockCntY) - 1;
        this->macroblockCorners.resize(this->macroblockCnt);
        this->macroblockCentres.resize(this->macroblockCnt);
        for (int row = maxRow; row >= 0; --row) {
            for (size_t col = 0; col < this->macroblockCntX; ++col) {
                // Get the ID of the macroblock.
                auto mbID = (static_cast<size_t>(maxRow) - static_cast<size_t>(row)) * this->macroblockCntX + col;

                // Compute the corners and the centre of the rectangle.
                this->computeRect(static_cast<size_t>(row), col, mbID, offsetHeight, offsetWidth);
            }
        }
    }
}


/*
 * megamol::encoder::AbstractVideoEncoder::computeMacroblockRects
 */
void megamol::encoder::AbstractVideoEncoder::computeRect(
    const size_t row, const size_t col, const size_t id, const float offsetHeight, const float offsetWidth) {
    // Compute the left top corner.
    this->macroblockCorners[id].corners[0].x = offsetWidth + static_cast<float>(col * this->macroblockWidth);
    this->macroblockCorners[id].corners[0].y = offsetHeight + static_cast<float>(row * this->macroblockHeight);

    // Compute the left bottom corner.
    this->macroblockCorners[id].corners[1].x = offsetWidth + static_cast<float>(col * this->macroblockWidth);
    this->macroblockCorners[id].corners[1].y =
        offsetHeight + static_cast<float>((row + 1) * this->macroblockHeight) - 1.0f;

    // Compute the right bottom corner.
    this->macroblockCorners[id].corners[2].x =
        offsetWidth + static_cast<float>((col + 1) * this->macroblockWidth) - 1.0f;
    this->macroblockCorners[id].corners[2].y =
        offsetHeight + static_cast<float>((row + 1) * this->macroblockHeight) - 1.0f;

    // Compute the right top corner.
    this->macroblockCorners[id].corners[3].x =
        offsetWidth + static_cast<float>((col + 1) * this->macroblockWidth) - 1.0f;
    this->macroblockCorners[id].corners[3].y = offsetHeight + static_cast<float>(row * this->macroblockHeight);

    // Compute the centre of the macroblock.
    this->macroblockCentres[id].x = this->macroblockCorners[id].corners[0].x + (this->macroblockWidth / 2.0f);
    this->macroblockCentres[id].y = this->macroblockCorners[id].corners[0].y + (this->macroblockHeight / 2.0f);
}


/*
 * megamol::encoder::AbstractVideoEncoder::create
 */
bool megamol::encoder::AbstractVideoEncoder::create(void) {
    // Remember the values of potentially unchangeable slots.
    this->previousEncoderCodec = this->encoderCodec.Param<megamol::core::param::EnumParam>()->Value();
    this->previousFlipFrames = this->flipFrames.Param<megamol::core::param::BoolParam>()->Value();
    this->previousFoveated = this->useFoveated.Param<megamol::core::param::BoolParam>()->Value();
    this->previousInputMode = this->inputMode.Param<megamol::core::param::EnumParam>()->Value();
    this->previousLatency = this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value();
    this->previousOffsetHeight = this->offsetHeight.Param<megamol::core::param::IntParam>()->Value();
    this->previousOffsetWidth = this->offsetWidth.Param<megamol::core::param::IntParam>()->Value();
    this->previousOutFilename = this->outFilenameSlot.Param<megamol::core::param::FilePathParam>()->ValueString();
    this->previousOutputMode = this->outputMode.Param<megamol::core::param::EnumParam>()->Value();
    this->previousOverallHeight = this->overallHeight.Param<megamol::core::param::IntParam>()->Value();
    this->previousOverallWidth = this->overallWidth.Param<megamol::core::param::IntParam>()->Value();
    this->previousQPValue = this->qpValue.Param<megamol::core::param::IntParam>()->Value();
    this->previousSIMD = this->useSIMD.Param<megamol::core::param::BoolParam>()->Value();
    this->previousUseInfiniteGOP = this->useInfiniteGOP.Param<megamol::core::param::BoolParam>()->Value();
    return true;
}


/*
 * megamol::encoder::AbstractVideoEncoder::GetExtents
 */
bool megamol::encoder::AbstractVideoEncoder::GetExtents(megamol::core::view::CallRender3D_2& call) {
    // Check if the call is valid.
    auto* cr = dynamic_cast<megamol::core::view::CallRender3D_2*>(&call);
    if (cr == nullptr) {
        return false;
    }

    // Pass on the GetExtents call as we do not change it.
    auto* chainedCall = this->chainRenderSlot.CallAs<megamol::core::view::CallRender3D_2>();
    if ((chainedCall != nullptr) && ((*chainedCall)(megamol::core::view::AbstractCallRender::FnGetExtents))) {
        auto mybb = call.AccessBoundingBoxes().BoundingBox();
        mybb.Union(chainedCall->AccessBoundingBoxes().BoundingBox());
        auto mycb = call.AccessBoundingBoxes().ClipBox();
        mycb.Union(chainedCall->AccessBoundingBoxes().ClipBox());
        call.AccessBoundingBoxes().SetBoundingBox(mybb);
        call.AccessBoundingBoxes().SetClipBox(mycb);
        cr->SetTimeFramesCount(chainedCall->TimeFramesCount());

    } else {
        cr->SetTimeFramesCount(1);
        cr->AccessBoundingBoxes().Clear();
    }
    return true;
}


/*
 * megamol::encoder::AbstractVideoEncoder::initialiseBaseLatencyMeasurements
 */
void megamol::encoder::AbstractVideoEncoder::initialiseBaseLatencyMeasurements(void) {
    // Check if latency measurements are turned on.
    if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
        // Check if the frame should be flipped.
        if (this->flipFrames.Param<megamol::core::param::BoolParam>()->Value()) {
            // Add measurements for the flipping of the image.
            this->latencyMeasurementFlippingFrame.Initialise(2000, ".\\FlipingFrame.csv");
        }

        // Add measurements for the processing of the output.
        this->latencyMeasurementOverall.Initialise(2000, ".\\CompleteProcess.csv");
    }
}


/*
 * megamol::encoder::AbstractVideoEncoder::initialiseFoveated
 */
void megamol::encoder::AbstractVideoEncoder::initialiseFoveated(void) {
    // Check if the user wants to use foveation.
    if (this->useFoveated.Param<megamol::core::param::BoolParam>()->Value()) {
        // Compute the number of macroblocks of the image.
        this->computeMacroblockCnt();

        // Compute the macroblock corners and centres.
        float heightOffset = static_cast<float>(this->offsetHeight.Param<megamol::core::param::IntParam>()->Value());
        float widthOffset = static_cast<float>(this->offsetWidth.Param<megamol::core::param::IntParam>()->Value());
        this->computeMacroblockRects(heightOffset, widthOffset);

        // Initialise the macroblock quality mapping.
        this->initialiseQualityMapping();

        // Remeber the dimensions of the overall display.
        this->overallHeightF = static_cast<float>(this->overallHeight.Param<megamol::core::param::IntParam>()->Value());
        this->overallWidthF = static_cast<float>(this->overallWidth.Param<megamol::core::param::IntParam>()->Value());

        // Check if the values are zero, in that case use the width and height of the local display.
        if (this->overallHeight.Param<megamol::core::param::IntParam>()->Value() == 0 &&
            this->overallWidth.Param<megamol::core::param::IntParam>()->Value() == 0) {
            this->overallHeightF = static_cast<float>(this->height);
            this->overallWidthF = static_cast<float>(this->width);
        }
    }
}


/*
 * megamol::encoder::AbstractVideoEncoder::initialiseInput
 */
void megamol::encoder::AbstractVideoEncoder::initialiseInput(void) {
    // Check if the input is MPI.
    if (this->inputMode.Param<megamol::core::param::EnumParam>()->Value() == 1) {
        this->initialiseMPIInput();
    }
}


/*
 * megamol::encoder::AbstractVideoEncoder::initialiseMPI
 */
void megamol::encoder::AbstractVideoEncoder::initialiseMPI(void) {
    #ifdef WITH_NETWORK
    // Check if the input is MPI.
    bool mpiInput = false;
    bool mpiFoveatedInput = false;
    if (this->inputMode.Param<megamol::core::param::EnumParam>()->Value() == 1) {
        // The node uses MPI in order get input.
        mpiInput = true;

        // Compute the unique data ID of the input.
        auto x = this->offsetWidth.Param<megamol::core::param::IntParam>()->Value();
        auto y = this->offsetHeight.Param<megamol::core::param::IntParam>()->Value();
        this->uniqueInputDataID = megamol::network::MpiIntraCall::ComputeUniqueID(
            static_cast<int>(megamol::network::IntraDataType::BITSTREAM),
            megamol::network::MpiIntraCall::ComputeUniqueID(x, y));

        // Check if foveated encoding is used.
        if (this->useFoveated.Param<megamol::core::param::BoolParam>()->Value()) {
            // The node uses MPI in order get foveated regions.
            mpiFoveatedInput = true;

            // Compute the unique data ID of the input.
            this->uniqueInputFoveatedDataID = megamol::network::MpiIntraCall::ComputeUniqueID(
                static_cast<int>(megamol::network::IntraDataType::FOVEATEDREGION),
                megamol::network::MpiIntraCall::ComputeUniqueID(x, y));
        }
    }

    // Check if the output is MPI.
    bool mpiOutput = false;
    if (this->outputMode.Param<megamol::core::param::EnumParam>()->Value() == 1) {
        // The node uses MPI in order to deal with the output.
        mpiOutput = true;

        // Compute the unique data ID of the output.
        auto x = this->offsetWidth.Param<megamol::core::param::IntParam>()->Value();
        auto y = this->offsetHeight.Param<megamol::core::param::IntParam>()->Value();
        this->uniqueOutputDataID = megamol::network::MpiIntraCall::ComputeUniqueID(
            static_cast<int>(megamol::network::IntraDataType::BITSTREAMENC),
            megamol::network::MpiIntraCall::ComputeUniqueID(x, y));
    }

    // Determine the role(s) of the node.
    std::underlying_type<megamol::network::Role>::type myRoles =
        static_cast<std::underlying_type<megamol::network::Role>::type>(megamol::network::Role::NONE);
    if (mpiInput) {
        myRoles |=
            static_cast<std::underlying_type<megamol::network::Role>::type>(megamol::network::Role::DATA_RECEIVER);
    }
    if (mpiOutput) {
        myRoles |=
            static_cast<std::underlying_type<megamol::network::Role>::type>(megamol::network::Role::DATA_PROVIDER);
    }

    // Use the callSendMPIData slot to initialise the MpiIntraCommunicator
    megamol::network::MpiIntraCall* call = this->mpiIntraCall.CallAs<megamol::network::MpiIntraCall>();
    if (call != nullptr) {
        // Initialise the MpiIntraCommunicator.
        call->SetRoles(static_cast<megamol::network::Role>(myRoles));
        if (!(*call)(megamol::network::MpiIntraCall::IDX_INIT_COMM)) {
            return;
        }

        // Set the provided data, i.e. the encoded bitstream.
        if (mpiOutput) {
            call->SetDataType(megamol::network::IntraDataType::BITSTREAMENC);
            call->SetDataID(this->uniqueOutputDataID);
            (*call)(megamol::network::MpiIntraCall::IDX_ADD_PROVIDED_DATA);
        }

        // Set the expected data, i.e. the raw bitstream and the foveated regions.
        if (mpiInput) {
            call->SetDataType(megamol::network::IntraDataType::BITSTREAM);
            call->SetDataID(this->uniqueInputDataID);
            (*call)(megamol::network::MpiIntraCall::IDX_ADD_EXPECTED_DATA);
        }
        if (mpiFoveatedInput) {
            call->SetDataType(megamol::network::IntraDataType::FOVEATEDREGION);
            call->SetDataID(this->uniqueInputFoveatedDataID);
            (*call)(megamol::network::MpiIntraCall::IDX_ADD_EXPECTED_DATA);
        }

        // Start the MpiIntraCommunicator.
        (*call)(megamol::network::MpiIntraCall::IDX_START_COMM);

    } else {
        megamol::core::utility::log::Log::DefaultLog.WriteError(
            "Failed to initialise the MpiIntraCommunicator as the call is a nullptr.");
    }
    #else
    megamol::core::utility::log::Log::DefaultLog.WriteWarn(
        "Cannot initialise MpiIntraCommunicator as the Network plugin is not installed.");
    #endif
}


/*
 * megamol::encoder::AbstractVideoEncoder::initialiseMPIInput
 */
void megamol::encoder::AbstractVideoEncoder::initialiseMPIInput(void) {
    #ifdef WITH_NETWORK
    // Start the MPI input threads.
    this->mpiInputThreadRunning.store(true);
    try {
        // Start the raw bitstream input thread.
        this->mpiInputBitThread = std::thread(
            std::bind(&AbstractVideoEncoder::mpiInput, std::ref(*this), megamol::network::IntraDataType::BITSTREAM));

        // Check if foveated encoding is used.
        if (this->useFoveated.Param<megamol::core::param::BoolParam>()->Value()) {
            // Start the foveated region input thread.
            this->mpiInputFovThread = std::thread(std::bind(
                &AbstractVideoEncoder::mpiInput, std::ref(*this), megamol::network::IntraDataType::FOVEATEDREGION));
        }

    } catch (std::exception& ex) {
        megamol::core::utility::log::Log::DefaultLog.WriteError(
            "Failed to start the MPI input threads with error \"%s\".", ex.what());
    }
    #else
    megamol::core::utility::log::Log::DefaultLog.WriteWarn(
        "Cannot initialise MPI based input as the Network plugin is not installed.");
    #endif
}


/*
 * megamol::encoder::AbstractVideoEncoder::initialiseOutput
 */
void megamol::encoder::AbstractVideoEncoder::initialiseOutput(void) {
    // Check what outout method should be used.
    switch (this->outputMode.Param<megamol::core::param::EnumParam>()->Value()) {
        case 2:
            // TODO...
            break;

        case 1:
            // Nothing to do here.
            break;

        case 0:
        default:
            // Check if the file needs to be closed.
            if (this->outFile != nullptr) {
                // Close the file.
                ::fclose(this->outFile);
                this->outFile = nullptr;

                // Check if the file is empty and can be deleted.
                if (this->deleteOutfile) {
                    // Delete file and ignore return value...
                    std::remove(this->outFilenameSlot.Param<megamol::core::param::FilePathParam>()->ValueString());
                }
            }

            // Open the file in case it was not opend so far.
            if (this->outFile == nullptr) {
                auto isOpen = ::fopen_s(&this->outFile,
                    this->outFilenameSlot.Param<megamol::core::param::FilePathParam>()->ValueString(), "wb");
                if (isOpen != 0) {
                    megamol::core::utility::log::Log::DefaultLog.WriteError("Failed to create file %s with error %d",
                        this->outFilenameSlot.Param<megamol::core::param::FilePathParam>()->ValueString(), isOpen);
                }
                this->deleteOutfile = true;
            }
            break;
    }
}


/*
 * megamol::encoder::AbstractVideoEncoder::initialiseQualityMapping
 */
void megamol::encoder::AbstractVideoEncoder::initialiseQualityMapping(void) {
    // Initialise the macroblock quality mapping.
    for (auto& mapping : this->macroblockQualityMapping) {
        mapping.resize(this->macroblockCnt, 0.0f);
    }
    this->macroblockQualityReadIdx.store(0);
    this->macroblockQualityWriteIdx.store(1);
}


/*
 * megamol::encoder::AbstractVideoEncoder::intersection
 */
bool megamol::encoder::AbstractVideoEncoder::intersection(const ScaledFovRect& a, const ScaledFovRect& b) {
    // Note:
    // Algorithm from the accepted answer adapted to c++.
    // (https://stackoverflow.com/questions/10962379/how-to-check-intersection-between-2-rotated-rectangles)

    // Check for intersection by using projecting the second rectangle onto the
    // first rectangle.
    if (!megamol::encoder::AbstractVideoEncoder::intersection(a, b, a)) {
        return false;
    }

    // Check for intersection by using projecting the first rectangle onto the
    // second rectangle.
    if (!megamol::encoder::AbstractVideoEncoder::intersection(a, b, b)) {
        return false;
    }
    return true;
}


/*
 * megamol::encoder::AbstractVideoEncoder::intersection
 */
bool megamol::encoder::AbstractVideoEncoder::intersection(
    const ScaledFovRect& a, const ScaledFovRect& b, const ScaledFovRect& rect) {
    // Loop over all corners of the current rectangle.
    for (size_t i = 0; i < rect.corners.size(); ++i) {
        // Get the two points of the current edge.
        size_t j = (i + 1) % rect.corners.size();
        auto p1 = rect.corners[i];
        auto p2 = rect.corners[j];

        // Compute the normal of the current edge.
        PointF normal;
        normal.x = p2.y - p1.y;
        normal.y = p1.x - p2.x;

        // Test if all points of the first rectangle are on the same side of the
        // current edge.
        float minA = std::numeric_limits<float>::max();
        float maxA = std::numeric_limits<float>::min();
        for (const auto& p : a.corners) {
            auto projected = normal.x * p.x + normal.y * p.y;
            minA = (projected < minA) ? projected : minA;
            maxA = (projected > maxA) ? projected : maxA;
        }

        // Test if all points of the second rectangle are on the same side of the
        // current edge.
        float minB = std::numeric_limits<float>::max();
        float maxB = std::numeric_limits<float>::min();
        for (const auto& p : b.corners) {
            auto projected = normal.x * p.x + normal.y * p.y;
            minB = (projected < minB) ? projected : minB;
            maxB = (projected > maxB) ? projected : maxB;
        }

        // Check if this edge is a seperating line.
        if (maxA < minB || maxB < minA) {
            return false;
        }
    }
    return true;
}


#ifdef WITH_NETWORK
/*
 * megamol::encoder::AbstractVideoEncoder::mpiInput
 */
void megamol::encoder::AbstractVideoEncoder::mpiInput(const megamol::network::IntraDataType dataType) {
    // Check for new received data from the MpiIntraCommunicator.
    while (this->mpiInputThreadRunning.load()) {
        // Check if the call to the MpiIntraCommunicator is active.
        auto isActive = this->active.Param<megamol::core::param::BoolParam>()->Value();
        megamol::network::MpiIntraCall* call = this->mpiIntraCall.CallAs<megamol::network::MpiIntraCall>();
        if (call != nullptr && isActive) {
            // Get the raw data or the foveated regions.
            call->SetDataType(dataType);
            if ((*call)(megamol::network::MpiIntraCall::IDX_DELIVER_DATA)) {
                // New data available, call the onInputMPI function.
                this->onInputMPI(call->GetDeliverData(), call->GetDeliverDataCnt(), dataType);

            } else {
                // An error occured while delivering the data. Aborting...
                megamol::core::utility::log::Log::DefaultLog.WriteError(
                    "An error occured while delivering the data. Aborting...");
                break;
            }

        } else {
            megamol::core::utility::log::Log::DefaultLog.WriteError(
                "Failed to get the received data from the MpiIntraCommunicator as the call is a nullptr. Aborting...");
            break;
        }
    }
}
#endif


/*
 * megamol::encoder::AbstractVideoEncoder::OnMouseMove
 */
bool megamol::encoder::AbstractVideoEncoder::OnMouseMove(double x, double y) {
    // Check if the input mode is Call and not MPI.
    auto encoderActive = this->active.Param<megamol::core::param::BoolParam>()->Value();
    if (this->inputMode.Param<megamol::core::param::EnumParam>()->Value() == 0 && encoderActive) {
        // Check if foveated encoding should be used.
        if (this->useFoveated.Param<megamol::core::param::BoolParam>()->Value()) {
            // Check if the position has changed.
            auto hasChanged = x != this->lastMousePos.first && y != this->lastMousePos.second;
            if (hasChanged) {
                // Get the current position of the cursor and scale to [0,1].
                double xScaled = x / static_cast<double>(this->width);
                double yScaled = y / static_cast<double>(this->height);

                // Compute the rectangle.
                FovRect lookAtRect;
                lookAtRect.leftTop.x = xScaled - (128.0 / static_cast<double>(this->width));
                lookAtRect.leftTop.y = yScaled - (128.0 / static_cast<double>(this->height));
                lookAtRect.rightTop.x = xScaled + (128.0 / static_cast<double>(this->width));
                lookAtRect.rightTop.y = yScaled - (128.0 / static_cast<double>(this->height));
                lookAtRect.leftBottom.x = xScaled - (128.0 / static_cast<double>(this->width));
                lookAtRect.leftBottom.y = yScaled + (128.0 / static_cast<double>(this->height));
                lookAtRect.rightBottom.x = xScaled + (128.0 / static_cast<double>(this->width));
                lookAtRect.rightBottom.y = yScaled + (128.0 / static_cast<double>(this->height));

                // Update the macroblocks.
                this->checkMBIntersectionFrame(&lookAtRect, 1, 0);
            }
        }
    }
    return false;
}


/*
 * megamol::encoder::AbstractVideoEncoder::processOutput
 */
void megamol::encoder::AbstractVideoEncoder::processOutput(
    void* data, const size_t dataCnt, const size_t dataElementSize, const size_t timestamp) {
    // Check what should be done with the encoded frames.
    switch (this->outputMode.Param<megamol::core::param::EnumParam>()->Value()) {
        case 2:
            // TODO...
            break;

        case 1:
            #ifdef WITH_NETWORK
            // Use the callSendMPIData slot to send data via MPI.
            {
                megamol::network::MpiIntraCall* call = this->mpiIntraCall.CallAs<megamol::network::MpiIntraCall>();
                if (call != nullptr) {
                    // Set the data.
                    call->SetData(data);
                    call->SetDataCnt(dataCnt);
                    call->SetDataID(this->uniqueOutputDataID);
                    call->SetHeight(this->height);
                    call->SetTimestamp(static_cast<int64_t>(timestamp));
                    call->SetWidth(this->width);

                    // Call the send data function.
                    (*call)(megamol::network::MpiIntraCall::IDX_SEND_DATA);
                }
            }
            #else
            megamol::core::utility::log::Log::DefaultLog.WriteWarn(
                "Cannot process MPI based output as the Network plugin is not installed.");
            #endif
            break;

        case 0:
        default:
            // Write the frame into the file.
            if (this->outFile != nullptr) {
                auto written = ::fwrite(data, dataElementSize, dataCnt, this->outFile);
                if (written != dataCnt) {
                    megamol::core::utility::log::Log::DefaultLog.WriteError(
                        "Failed to save frame (%zu bytes) with timestamp %lld with error %d", dataCnt, timestamp,
                        written);
                }
                this->deleteOutfile = false;
            }
            break;
    }
}


/*
 * megamol::encoder::AbstractVideoEncoder::release
 */
void megamol::encoder::AbstractVideoEncoder::release(void) {
    // Stop the input.
    this->stopInput();

    // Stop the output.
    this->stopOutput();
}


/*
 * megamol::encoder::AbstractVideoEncoder::Render
 */
bool megamol::encoder::AbstractVideoEncoder::Render(megamol::core::view::CallRender3D_2& call) {
    // Check if the call is valid.
    auto* cr = dynamic_cast<megamol::core::view::CallRender3D_2*>(&call);
    if (cr == nullptr) {
        return false;
    }

    // Check if the input mode is Call and not MPI.
    auto encoderActive = this->active.Param<megamol::core::param::BoolParam>()->Value();
    if (this->inputMode.Param<megamol::core::param::EnumParam>()->Value() == 0 && encoderActive) {
        this->onInputCall(call);
    }
    return true;
}


/*
 * megamol::encoder::AbstractVideoEncoder::scaleRectangle
 */
megamol::encoder::ScaledFovRect megamol::encoder::AbstractVideoEncoder::scaleRectangle(const FovRect& rect) {
    // Create the scaled look-at rectangle.
    ScaledFovRect dst;

    // Scale the left top point.
    dst.corners[0].x = rect.leftTop.x * this->overallWidthF;
    dst.corners[0].y = rect.leftTop.y * this->overallHeightF;

    // Scale the left bottom point.
    dst.corners[1].x = rect.leftBottom.x * this->overallWidthF;
    dst.corners[1].y = rect.leftBottom.y * this->overallHeightF;

    // Scale the right bottom point.
    dst.corners[2].x = rect.rightBottom.x * this->overallWidthF;
    dst.corners[2].y = rect.rightBottom.y * this->overallHeightF;

    // Scale the right top point.
    dst.corners[3].x = rect.rightTop.x * this->overallWidthF;
    dst.corners[3].y = rect.rightTop.y * this->overallHeightF;

    // Return the scaled rectangle.
    return dst;
}


/*
 * megamol::encoder::AbstractVideoEncoder::stopEncoder
 */
void megamol::encoder::AbstractVideoEncoder::stopEncoder(void) {
}


/*
 * megamol::encoder::AbstractVideoEncoder::stopInput
 */
void megamol::encoder::AbstractVideoEncoder::stopInput(void) {
    // Check if the input was MPI.
    if (this->inputMode.Param<megamol::core::param::EnumParam>()->Value() == 1) {
        this->stopMPIInput();
    }
}


/*
 * megamol::encoder::AbstractVideoEncoder::stopInput
 */
void megamol::encoder::AbstractVideoEncoder::stopMPI(void) {
    #ifdef WITH_NETWORK
    // Use the callSendMPIData slot to stop the MpiIntraCommunicator
    megamol::network::MpiIntraCall* call = this->mpiIntraCall.CallAs<megamol::network::MpiIntraCall>();
    if (call != nullptr) {
        // Stop the MpiIntraCommunicator.
        (*call)(megamol::network::MpiIntraCall::IDX_STOP_COMM);
    }
    #else
    megamol::core::utility::log::Log::DefaultLog.WriteWarn(
        "Cannot stop MpiIntraCommunicator as the Network plugin is not installed.");
    #endif
}


/*
 * megamol::encoder::AbstractVideoEncoder::stopMPIInput
 */
void megamol::encoder::AbstractVideoEncoder::stopMPIInput(void) {
    #ifdef WITH_NETWORK
    // Stop the raw bitstream thread.
    this->mpiInputThreadRunning.store(false);
    if (this->mpiInputBitThread.joinable()) {
        this->mpiInputBitThread.join();
    }

    // Stop the foveated region thread.
    this->mpiInputThreadRunning.store(false);
    if (this->mpiInputFovThread.joinable()) {
        this->mpiInputFovThread.join();
    }
    #else
    megamol::core::utility::log::Log::DefaultLog.WriteWarn(
        "Cannot stop MPI based input as the Network plugin is not installed.");
    #endif
}


/*
 * megamol::encoder::AbstractVideoEncoder::stopOutput
 */
void megamol::encoder::AbstractVideoEncoder::stopOutput(void) {
    // Check what outout method was used.
    switch (this->outputMode.Param<megamol::core::param::EnumParam>()->Value()) {
        case 2:
            // TODO...
            break;

        case 1:
            // Nothing to do here.
            break;

        case 0:
        default:
            // Check if the file needs to be closed.
            if (this->outFile != nullptr) {
                // Close the file.
                ::fclose(this->outFile);
                this->outFile = nullptr;

                // Check if the file is empty and can be deleted.
                if (this->deleteOutfile) {
                    // Delete file and ignore return value...
                    std::remove(this->outFilenameSlot.Param<megamol::core::param::FilePathParam>()->ValueString());
                }
            }
            break;
    }
}


/*
 * megamol::encoder::AbstractVideoEncoder::unchangeableCallback
 */
bool megamol::encoder::AbstractVideoEncoder::unchangeableCallback(megamol::core::param::ParamSlot& slot) {
    // Check if the encoder is active.
    if (this->active.Param<megamol::core::param::BoolParam>()->Value()) {
        // Check which slot triggered the callback.
        if (&slot == &this->encoderCodec) {
            // Log the warning and restore the previous encoder codec.
            megamol::core::utility::log::Log::DefaultLog.WriteError(
                "Unable to change the encoder codec while the encoder is running.");
            this->encoderCodec.Param<megamol::core::param::EnumParam>()->SetValue(this->previousEncoderCodec, false);

        } else if (&slot == &this->flipFrames) {
            // Log the warning and restore the previous flip frames state.
            megamol::core::utility::log::Log::DefaultLog.WriteError(
                "Unable to change the flip frames state while the encoder is running.");
            this->flipFrames.Param<megamol::core::param::BoolParam>()->SetValue(this->previousFlipFrames, false);

        } else if (&slot == &this->useFoveated) {
            // Log the warning and restore the previous foveated encoding state.
            megamol::core::utility::log::Log::DefaultLog.WriteError(
                "Unable to change the foveated encoding while the encoder is running.");
            this->useFoveated.Param<megamol::core::param::BoolParam>()->SetValue(this->previousFoveated, false);

        } else if (&slot == &this->inputMode) {
            // Log the warning and restore the input mode.
            megamol::core::utility::log::Log::DefaultLog.WriteError(
                "Unable to change the input mode while the encoder is running.");
            this->inputMode.Param<megamol::core::param::EnumParam>()->SetValue(this->previousInputMode, false);

        } else if (&slot == &this->latencyMeasurement) {
            // Log the warning and restore the previous foveated encoding state.
            megamol::core::utility::log::Log::DefaultLog.WriteError(
                "Unable to change the latency measurement while the encoder is running.");
            this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->SetValue(this->previousLatency, false);

        } else if (&slot == &this->offsetHeight) {
            // Log the warning and restore the previous height offset.
            megamol::core::utility::log::Log::DefaultLog.WriteError(
                "Unable to change the height offset while the encoder is running.");
            this->offsetHeight.Param<megamol::core::param::IntParam>()->SetValue(this->previousOffsetHeight, false);

        } else if (&slot == &this->offsetWidth) {
            // Log the warning and restore the previous width offset.
            megamol::core::utility::log::Log::DefaultLog.WriteError(
                "Unable to change the width offset while the encoder is running.");
            this->offsetWidth.Param<megamol::core::param::IntParam>()->SetValue(this->previousOffsetWidth, false);

        } else if (&slot == &this->outFilenameSlot) {
            // Log the warning and restore the previous filename.
            megamol::core::utility::log::Log::DefaultLog.WriteError(
                "Unable to change the filename while already writing to a different file.");
            this->outFilenameSlot.Param<megamol::core::param::FilePathParam>()->SetValue(
                this->previousOutFilename, false);

        } else if (&slot == &this->outputMode) {
            // Log the warning and restore the previous output mode.
            megamol::core::utility::log::Log::DefaultLog.WriteError(
                "Unable to change the output mode while the encoder is running.");
            this->outputMode.Param<megamol::core::param::EnumParam>()->SetValue(this->previousOutputMode, false);

        } else if (&slot == &this->overallHeight) {
            // Log the warning and restore the previous overall height.
            megamol::core::utility::log::Log::DefaultLog.WriteError(
                "Unable to change the overall height while the encoder is running.");
            this->overallHeight.Param<megamol::core::param::IntParam>()->SetValue(this->previousOverallHeight, false);

        } else if (&slot == &this->overallWidth) {
            // Log the warning and restore the previous overall width.
            megamol::core::utility::log::Log::DefaultLog.WriteError(
                "Unable to change the overall width while the encoder is running.");
            this->overallWidth.Param<megamol::core::param::IntParam>()->SetValue(this->previousOverallWidth, false);

        } else if (&slot == &this->qpValue) {
            // Log the warning and restore the previous QP value.
            megamol::core::utility::log::Log::DefaultLog.WriteError(
                "Unable to change the QP value while the encoder is running.");
            this->qpValue.Param<megamol::core::param::IntParam>()->SetValue(this->previousQPValue, false);

        } else if (&slot == &this->useSIMD) {
            // Log the warning and restore the previous SIMD state.
            megamol::core::utility::log::Log::DefaultLog.WriteError(
                "Unable to change the usage of SIMD instructions while the encoder is running.");
            this->useSIMD.Param<megamol::core::param::BoolParam>()->SetValue(this->previousSIMD, false);

        } else if (&slot == &this->useInfiniteGOP) {
            // Log the warning and restore the previous GOP state.
            megamol::core::utility::log::Log::DefaultLog.WriteError(
                "Unable to change the GOP length while the encoder is running.");
            this->useInfiniteGOP.Param<megamol::core::param::BoolParam>()->SetValue(
                this->previousUseInfiniteGOP, false);

        } else {
            // Called by an unexpected slot.
            megamol::core::utility::log::Log::DefaultLog.WriteError(
                "The unchangeableCallback was called by an unexpected slot 0x%p.", &slot);
        }

    } else {
        // Check which slot triggered the callback.
        if (&slot == &this->encoderCodec) {
            // Remeber the new value.
            this->previousEncoderCodec = this->encoderCodec.Param<megamol::core::param::EnumParam>()->Value();

        } else if (&slot == &this->flipFrames) {
            // Remeber the new value.
            this->previousFlipFrames = this->flipFrames.Param<megamol::core::param::BoolParam>()->Value();

        } else if (&slot == &this->useFoveated) {
            // Remeber the new value.
            this->previousFoveated = this->useFoveated.Param<megamol::core::param::BoolParam>()->Value();

        } else if (&slot == &this->inputMode) {
            // Remeber the new value.
            this->previousInputMode = this->inputMode.Param<megamol::core::param::EnumParam>()->Value();

        } else if (&slot == &this->latencyMeasurement) {
            // Remeber the new value.
            this->previousLatency = this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value();

        } else if (&slot == &this->offsetHeight) {
            // Remeber the new value.
            this->previousOffsetHeight = this->offsetHeight.Param<megamol::core::param::IntParam>()->Value();

        } else if (&slot == &this->offsetWidth) {
            // Remeber the new value.
            this->previousOffsetWidth = this->offsetWidth.Param<megamol::core::param::IntParam>()->Value();

        } else if (&slot == &this->outFilenameSlot) {
            // Remeber the new value.
            this->previousOutFilename =
                this->outFilenameSlot.Param<megamol::core::param::FilePathParam>()->ValueString();

        } else if (&slot == &this->outputMode) {
            // Remeber the new value.
            this->previousOutputMode = this->outputMode.Param<megamol::core::param::EnumParam>()->Value();

        } else if (&slot == &this->overallHeight) {
            // Remeber the new value.
            this->previousOverallHeight = this->overallHeight.Param<megamol::core::param::IntParam>()->Value();

        } else if (&slot == &this->overallWidth) {
            // Remeber the new value.
            this->previousOverallWidth = this->overallWidth.Param<megamol::core::param::IntParam>()->Value();

        } else if (&slot == &this->qpValue) {
            // Remeber the new value.
            this->previousQPValue = this->qpValue.Param<megamol::core::param::IntParam>()->Value();

        } else if (&slot == &this->useSIMD) {
            // Remeber the new value.
            this->previousSIMD = this->useSIMD.Param<megamol::core::param::BoolParam>()->Value();

        } else if (&slot == &this->useInfiniteGOP) {
            // Remeber the new value.
            this->previousUseInfiniteGOP = this->useInfiniteGOP.Param<megamol::core::param::BoolParam>()->Value();

        } else {
            // Called by an unexpected slot.
            megamol::core::utility::log::Log::DefaultLog.WriteError(
                "The unchangeableCallback was called by an unexpected slot 0x%p.", &slot);
        }
    }
    return true;
}
