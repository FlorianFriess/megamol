/*
 * AbstractVideoEncoder.h
 *
 * Author: Florian Frieß
 * Copyright (C) 2021 by Universitaet Stuttgart (VISUS).
 * All rights reserved.
 */

#ifndef ENCODER_ABSTRACTVIDEOENCODER_H_INCLUDED
#define ENCODER_ABSTRACTVIDEOENCODER_H_INCLUDED
#if (defined(_MSC_VER) && (_MSC_VER > 1000))
#pragma once
#endif /* (defined(_MSC_VER) && (_MSC_VER > 1000)) */

#include "mmcore/CalleeSlot.h"
#include "mmcore/CallerSlot.h"
#include "mmcore/Module.h"
#include "mmcore/param/BoolParam.h"
#include "mmcore/param/EnumParam.h"
#include "mmcore/param/FilePathParam.h"
#include "mmcore/param/IntParam.h"
#include "mmcore/param/StringParam.h"
#include "mmcore/param/ParamSlot.h"
#include "mmcore/view/Renderer3DModule_2.h"

#include "vislib/sys/CriticalSection.h"
#include "vislib/sys/PerformanceCounter.h"

#include "LatencyMeasurement.h"
#include "types.h"
#ifdef WITH_NETWORK
#include "MpiIntraCall.h"
#endif


namespace megamol {
namespace encoder {

    /**
     * Abstract class that is used as the base class for video encoders
     */
    class AbstractVideoEncoder : public megamol::core::view::Renderer3DModule_2 {
    public:
        /**
         * Answer the name of this module.
         *
         * @return The name of this module.
         */
        static const char* ClassName(void) {
            return "AbstractVideoEncoder";
        }

        /**
         * Answer a human readable description of this module.
         *
         * @return A human readable description of this module.
         */
        static const char* Description(void) {
            return "Base classfor he video encoder modules, do not use directly.";
        }

        /** Dtor */
        virtual ~AbstractVideoEncoder(void);

        /** Ctor */
        AbstractVideoEncoder(void);

    protected:
        /**
         * Callback function for the active slot.
         *
         * @param slot Must be active.
         */
        virtual bool activeCallback(megamol::core::param::ParamSlot& slot);

        /**
         * Check if the given loot-at rectangle intersects the local screen and if so compute the intersected
         * macroblocks.
         *
         * @param lookAtRects The vector that contains the look-at rectangles.
         * @param validElements The number of valid look-at rectangles in the lookAtRects vector.
         * @param timestamp The timestamp of the look-at rectangles.
         */
        virtual void checkMBIntersectionFrame(
            const FovRect* lookAtRects, const std::uint32_t validElements, const size_t timestamp);

        /**
         * Check if the machine supports SSE2 and AVX2.
         */
        virtual void checkSIMDInstructions(void);

        /**
         * Computes the number of macroblocks in the frame.
         */
        virtual void computeMacroblockCnt(void);

        /**
         * Computes the rectangle of each macroblock.
         *
         * @param offsetHeight The y offset of the node in respect to its position on the overall display.
         * @param offsetWidth The x offset of the node in respect to its position on the overall display.
         */
        virtual void computeMacroblockRects(const float offsetHeight, const float offsetWidth);

        /**
         * Compute the rectangle representing the macroblock.
         *
         * @param row The row the macroblock is in.
         * @param col The column the macroblock is in.
         * @param id The ID of the macrobrlock.
         * @param offsetHeight The y offset of the node in respect to its position on the overall display.
         * @param offsetWidth The x offset of the node in respect to its position on the overall display.
         */
        virtual void computeRect(
            const size_t row, const size_t col, const size_t id, const float offsetHeight, const float offsetWidth);

        /**
         * Implementation of 'Create'.
         *
         * @return 'true' on success, 'false' otherwise.
         */
        virtual bool create(void);

        /**
         * Encode the current frame.
         */
        virtual void encodeFrame(void) = 0;

        /**
         * Flipp the image as OpenGL uses the left bottom corner as the starting point, if requested.
         */
        virtual void flipImage(void) = 0;

        /**
         * The get extents callback. The module should set the members of 'call' to tell the caller the extents
         * of its data (bounding boxes and times).
         *
         * @param call The calling call.
         *
         * @return The return value of the function.
         */
        virtual bool GetExtents(megamol::core::view::CallRender3D_2& call);

        /**
         * Initialise the latency measurements used by all child classes.
         */
        virtual void initialiseBaseLatencyMeasurements(void);

        /**
         * Initialise the encoder.
         */
        virtual void initialiseEncoder(void) = 0;

        /**
         * Initialise the number of macroblocks, the rectangles for the macroblocks and the quality maps.
         */
        virtual void initialiseFoveated(void);

        /**
         * Initialise the input.
         */
        virtual void initialiseInput(void);

        /**
         * Initialise the MpiIntraCmmunicator.
         */
        virtual void initialiseMPI(void);

        /**
         * Initialise the MPI input thread that queries the MpiIntraCommunicator for received data.
         */
        virtual void initialiseMPIInput(void);

        /**
         * Initialise the current output method.
         */
        virtual void initialiseOutput(void);

        /**
         * Overwrite this function and initialse the special measurements for the child class.
         */
        virtual void initialiseSpecialLatencyMeasurements(void) = 0;

        /**
         * Initialise the QP value for all macroblocks.
         */
        virtual void initialiseQualityMapping(void);

        /**
         * Checks if the two rectangles intersect.
         *
         * @param a The first rectangle.
         * @param b The second rectangle.
         *
         * @return 'true' if they intersect, 'false' otherwise.
         */
        static bool intersection(const ScaledFovRect& a, const ScaledFovRect& b);

        /**
         * Checks if the two rectangles intersect by trying to find an edge that separates the two rectangles.
         *
         * @param a The first rectangle.
         * @param b The second rectangle.
         * @param rect The current rectangle.
         * 
         * @return 'true' if they intersect, 'false' otherwise.
         */
        static bool intersection(const ScaledFovRect& a, const ScaledFovRect& b, const ScaledFovRect& rect);

        /**
         * The funtion used by the mpiInputThread.
         *
         * @param dataType The type of the data.
         */
        #ifdef WITH_NETWORK
        void mpiInput(const megamol::network::IntraDataType dataType);
        #endif

        /**
         * Called whenever a new frame was rendered and the AbstractVideoEncoder::Render function is called.
         *
         * @param call The call containing the last rendered frame.
         */
        virtual void onInputCall(megamol::core::view::CallRender3D_2& call) = 0;

        /**
         * Called whenever a new frame was received via MPI.
         *
         * @param data The data of the frame.
         * @param dataCnt The number of valid elements in data.
         * @param dataType The type of the data.
         */
        #ifdef WITH_NETWORK
        virtual void onInputMPI(std::vector<std::pair<void*, size_t>>& data, const size_t dataCnt,
            const megamol::network::IntraDataType dataType) = 0;
        #endif

        /**
         * The mouse movement callback.
         */
        virtual bool OnMouseMove(double x, double y);

        /**
         * Process the output of the encoder.
         *
         * @param data The pointer to the data.
         * @param dataCnt The number of elements in the data.
         * @param dataElementSize The size, in byte, of one element of the data.
         * @param timestamp The timestamp of the data.
         */
        virtual void processOutput(
            void* data, const size_t dataCnt, const size_t dataElementSize, const size_t timestamp);

        /**
         * Implementation of 'Release'.
         */
        virtual void release(void);

        /**
         * The render callback.
         *
         * @param call The calling call.
         *
         * @return The return value of the function.
         */
        virtual bool Render(megamol::core::view::CallRender3D_2& call);

        /**
        * Scale the incoming rectangle based on the overall width of the dispaly.
        *
        * @param rect The received rectangle in [0,1] coordinates.
        * @return The scaled version of the received rectangle.
        */
        virtual ScaledFovRect scaleRectangle(const FovRect& rect);

        /**
         * Stop the encoder.
         */
        virtual void stopEncoder(void) = 0;

        /**
         * Stop the input.
         */
        virtual void stopInput(void);

        /**
         * Stop the MpiIntraCommunicator.
         */
        virtual void stopMPI(void);

        /**
         * Stop the MPI input thread.
         */
        virtual void stopMPIInput(void);

        /**
         * Stop the output.
         */
        virtual void stopOutput(void);

        /**
         * Callback function for all of the following slots: encoderCodec, flipFrames, useFoveated, inputMode,
         * latencyMeasurement, offsetHeight, offsetWidth, outFilenameSlot, outputMode, overallHeight,
         * overallWidth, qpValue, useSIMD and useInfiniteGOP.
         *
         * @param slot Must be one of the following slots: encoderCodec, flipFrames, useFoveated, inputMode,
         * latencyMeasurement, offsetHeight, offsetWidth, outFilenameSlot, outputMode, overallHeight,
         * overallWidth, qpValue, useSIMD and useInfiniteGOP.
         */
        virtual bool unchangeableCallback(megamol::core::param::ParamSlot& slot);

        /** The state of the encoder. */
        megamol::core::param::ParamSlot active;

        /** Flag that indicates AVX and AVX2 support. */
        bool avxSupport;

        /** Call to send and receive data via the MpiIntraCommunicator. */
        #ifdef WITH_NETWORK
        megamol::core::CallerSlot mpiIntraCall;
        #endif

        /** Flag that indicates the deletion of the outfile in case it is empty. */
        bool deleteOutfile;

        /** The slot used to select between h264 and h265. */
        megamol::core::param::ParamSlot encoderCodec;

        /** The slot used to turn on/off the flipping of the frames before encoding. */
        megamol::core::param::ParamSlot flipFrames;

        /** The foveated interval containing the QP values for the high, low and medium settings. */
        megamol::core::param::ParamSlot foveatedInterval;
        FoveatedInterval foveatedIntervalVal;

        /** The height of the incoming frames. */
        int height;

        /** Switches between MPI and a Call as the input mode. */
        megamol::core::param::ParamSlot inputMode;

        /** The last mouse position. */
        std::pair<double, double> lastMousePos;

        /** Turns on/off the latency measurements. */
        megamol::core::param::ParamSlot latencyMeasurement;

        /** The latency measurements for flipping the frame upside down. */
        LatencyMeasurement latencyMeasurementFlippingFrame;

        /** The latency measurements that covers the whole process. */
        LatencyMeasurement latencyMeasurementOverall;

        /** Store the centres, in pixel values, of the macroblocks. */
        std::vector<PointF> macroblockCentres;

        /** Stores the number of macroblocks in the frames. */
        size_t macroblockCnt;

        /** Stores the number of macroblocks along the width of the frame. */
        size_t macroblockCntX;

        /** Stores the number of macroblocks along the height of the frame. */
        size_t macroblockCntY;

        /** Store the corners, in pixel values, of the macroblocks. */
        std::vector<ScaledFovRect> macroblockCorners;

        /** Stores the height of a macroblock. */
        size_t macroblockHeight;

        /** Critical section for protecting 'queue'. */
        vislib::sys::CriticalSection macroblockLock;

        /** The quality to macroblock mapping for full frames. */
        std::array<std::vector<float>, 2> macroblockQualityMapping;

        /** The index of the macroblockQualityMapping that will be used for encoding. */
        std::atomic<size_t> macroblockQualityReadIdx;

        /** The index of the macroblockQualityMapping that will contain the new qulity settings. */
        std::atomic<size_t> macroblockQualityWriteIdx;

        /** Stores the width of a macroblock. */
        size_t macroblockWidth;

        /** The thread that queries the MpiIntraCommunicator for new received bitstream data. */
        std::thread mpiInputBitThread;

        /** The thread that queries the MpiIntraCommunicator for new received foveated regions. */
        std::thread mpiInputFovThread;

        /** Controls the lifetime of the mpiInputThread. */
        std::atomic<bool> mpiInputThreadRunning;

        /** The height offset of the current node with respect to the overall display. */
        megamol::core::param::ParamSlot offsetHeight;

        /** The width offset of the current node with respect to the overall display. */
        megamol::core::param::ParamSlot offsetWidth;

        /**
         * Stores the overall height, in pixel, of the display that is captured.
         *
         * @remark In case of displays that consist of multiple display nodes this value is the height of the
         * display, not the height of a single node unless they are equal.
         */
        megamol::core::param::ParamSlot overallHeight;
        float overallHeightF;

        /**
         * Stores the overall width, in pixel, of the display that is captured.
         *
         * @remark In case of displays that consist of multiple display nodes this value is the width of the
         * display, not the height of a single node unless they are equal.
         */
        megamol::core::param::ParamSlot overallWidth;
        float overallWidthF;

        /** The handle for the output file. */
        FILE* outFile;

        /** The path to the output file. */
        megamol::core::param::ParamSlot outFilenameSlot;

        /** The slot used to select what happens with the encoded frames. */
        megamol::core::param::ParamSlot outputMode;

        /** Stores the previous encoder codec. */
        int previousEncoderCodec;

        /** Stores the previous flip frames state. */
        bool previousFlipFrames;

        /** Stores the previous foveated encoding state. */
        bool previousFoveated;

        /** Stores the previous input mode. */
        int previousInputMode;

        /** Stores the previous latency measurement state. */
        bool previousLatency;

        /** Stores the previous height offset. */
        int previousOffsetHeight;

        /** Stores the previous width offset. */
        int previousOffsetWidth;

        /** Stores the previous filename of the output file. */
        vislib::TString previousOutFilename;

        /** Stores the previous output mode. */
        int previousOutputMode;

        /** Stores the previous overall height. */
        int previousOverallHeight;

        /** Stores the previous overall width. */
        int previousOverallWidth;

        /** Stores the previous QP value. */
        int previousQPValue;

        /** Stores the previous SIMD state. */
        bool previousSIMD;

        /** Stores the previous state of the infinite GOP length. */
        bool previousUseInfiniteGOP;

        /** The QP value used by the encoder in case foveated encoding is not used. */
        megamol::core::param::ParamSlot qpValue;

        /** Flag that indicates SSE, SSE2, SSE3, SSE41 and SSE42 support. */
        bool sseSupport;

        /** The unique data ID for the bitstream input. */
        int uniqueInputDataID;

        /** The unique data ID for the foveated input. */
        int uniqueInputFoveatedDataID;

        /** The unique data ID for the output. */
        int uniqueOutputDataID;

        /** The slot used to turn on/off foveated encoding. */
        megamol::core::param::ParamSlot useFoveated;

        /** The slot used to turn on/off SIMD instructions. */
        megamol::core::param::ParamSlot useSIMD;

        /** Flag that switches between periodic key frames and intra refresh (no key frames). */
        megamol::core::param::ParamSlot useInfiniteGOP;

        /** The width of the incoming frames. */
        int width;
    };

} // namespace encoder
} /* end namespace megamol */

#endif /* ENCODER_ABSTRACTVIDEOENCODER_H_INCLUDED */
