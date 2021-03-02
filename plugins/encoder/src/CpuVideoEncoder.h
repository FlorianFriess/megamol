/*
 * CpuVideoEncoder.h
 *
 * Author: Florian Frieß
 * Copyright (C) 2021 by Universitaet Stuttgart (VISUS).
 * All rights reserved.
 */

#ifndef ENCODER_CPUVIDEOENCODER_H_INCLUDED
#define ENCODER_CPUVIDEOENCODER_H_INCLUDED
#if (defined(_MSC_VER) && (_MSC_VER > 1000))
#pragma once
#endif /* (defined(_MSC_VER) && (_MSC_VER > 1000)) */

#include "AbstractVideoEncoder.h"
#include "x264.h"
#include "include/x265.h"

namespace megamol {
namespace encoder {

    /**
     * Class that offers CPU based video encoders
     */
    class CpuVideoEncoder : public AbstractVideoEncoder {
    public:
        /**
         * Answer the name of this module.
         *
         * @return The name of this module.
         */
        static const char* ClassName(void) {
            return "CpuEncoder";
        }

        /**
         * Answer a human readable description of this module.
         *
         * @return A human readable description of this module.
         */
        static const char* Description(void) {
            return "Uses CPU based video encoders to encode incoming bitstreams.";
        }

        /**
         * Answers whether this module is available on the current system.
         *
         * @return 'true' if the module is available, 'false' otherwise.
         */
        static bool IsAvailable(void) {
            return true;
        }

        /** Dtor */
        virtual ~CpuVideoEncoder(void);

        /** Ctor */
        CpuVideoEncoder(void);

    protected:
        /**
         * Implementation of 'Create'.
         *
         * @return 'true' on success, 'false' otherwise.
         */
        virtual bool create(void);

        /**
         * Implementation of 'AbstractVideoEncoder::encodeFrame'.
         */
        virtual void encodeFrame(void);

        /**
         * Implementation of 'AbstractVideoEncoder::flipImage'.
         */
        virtual void flipImage(void);

        /**
         * Implementation of 'AbstractVideoEncoder::initialiseEncoder'.
         */
        virtual void initialiseEncoder(void);

        /**
         * Overwrite this function and initialse the special measurements for the child class.
         */
        virtual void initialiseSpecialLatencyMeasurements(void);

        /**
         * Implementation of 'AbstractVideoEncoder::onInputCall'.
         */
        virtual void onInputCall(megamol::core::view::CallRender3D_2& call);

        /**
         * Implementation of 'AbstractVideoEncoder::onInputMPI'.
         */
        #ifdef WITH_NETWORK
        virtual void onInputMPI(std::vector<std::pair<void*, size_t>>& data, const size_t dataCnt,
            const megamol::network::IntraDataType dataType);
        #endif

        /**
         * Implementation of 'Release'.
         */
        virtual void release(void);

        /**
         * Implementation of 'stopEncoder'.
         */
        virtual void stopEncoder(void);

    private:
#pragma region struct X264Data
        /**
         * Contains the data needed by x264 library for encoding.
         */
        struct X264Data {
            /** Ctor */
            X264Data(void) : encoder(nullptr), inputFrame(), nal(nullptr), nalCnt(0u), outputFrame(), param() {}

            /** Pointer of the x264 encoder. */
            x264_t* encoder;

            /** The input frame. */
            x264_picture_t inputFrame;

            /** Contains the nal data of the encoded frame. */
            x264_nal_t* nal;

            /** The number of nals. */
            int nalCnt;

            /** The output frame. */
            x264_picture_t outputFrame;

            /** The parameter set used to initialise the encoder. */
            x264_param_t param;
        };
#pragma endregion
#pragma region struct X265Data
        /**
         * Contains the data needed by x265 library for encoding.
         */
        struct X265Data {
            /** Ctor */
            X265Data(void) : encoder(nullptr), inputFrame(), nal(nullptr), nalCnt(0u), outputFrame(), param() {}

            /** Pointer of the x265 encoder. */
            x265_encoder* encoder;

            /** The input frame. */
            x265_picture inputFrame;

            /** Contains the nal data of the encoded frame. */
            x265_nal* nal;

            /** The number of nals. */
            uint32_t nalCnt;

            /** The output frame. */
            x265_picture outputFrame;

            /** The parameter set used to initialise the encoder. */
            x265_param param;

            /** Store the converted YUV420 (planar) image. */
            std::vector<BYTE> yuv420Image;
        };
#pragma endregion

        /**
         * Flush the x264 encoder and close the session afterwards.
         */
        void closeEncoderX264(void);

        /**
         * Flush the x265 encoder and close the session afterwards.
         */
        void closeEncoderX265(void);

        /**
         * Convert the BGR image into a YUV420 (planar) image.
         */
        void convertBGRToYUV420(void);

        /**
         * Convert the BGR image into a YUV420 (planar) image using AVX.
         */
        void convertImageAVX(void);

        /**
         * Create the encoder based on the settings initialised in the create function.
         *
         * @param height The height of incoming frames.
         * @param width The width of incoming frames.
         */
        bool createEncoder(const int height, const int width);

        /**
         * Create the encoder based on the settings initialised in the create function.
         *
         * @param height The height of incoming frames.
         * @param width The width of incoming frames.
         */
        bool createEncoderX264(const int height, const int width);

        /**
         * Create the encoder based on the settings initialised in the create function.
         *
         * @param height The height of incoming frames.
         * @param width The width of incoming frames.
         */
        bool createEncoderX265(const int height, const int width);

        /**
         * Use the x2654 library to encode the incoming data.
         */
        void encodeFrameX264(void);

        /**
         * Use the x264 library to encode the incoming data using the current Foveated regions.
         */
        void encodeFrameX264Foveated(void);

        /**
         * Use the x265 library to encode the incoming data.
         */
        void encodeFrameX265(void);

        /**
         * Use the x265 library to encode the incoming data using the current Foveated regions.
         */
        void encodeFrameX265Foveated(void);

        /**
         * Called whenever a new frame was rendered and the AbstractVideoEncoder::Render function is called.
         * Special implementation for the x264 sdk.
         *
         * @param call The call containing the last rendered frame.
         */
        void onInputCallX264(megamol::core::view::CallRender3D_2& call);

        /**
         * Called whenever a new frame was rendered and the AbstractVideoEncoder::Render function is called.
         * Special implementation for the x265 sdk.
         *
         * @param call The call containing the last rendered frame.
         */
        void onInputCallX265(megamol::core::view::CallRender3D_2& call);

        /**
         * The funtion used by the separateImageAVXThreads to separate the colour channels from BGRBGRBGR...BGR
         * to BBB...GGG...RRR.
         */
        void separateImageAVX(void);

        /** Flag that indicates if the encoder still accept frames for encoding. */
        std::atomic<bool> acceptFrames;

        /** Indicates if the encoder needs to be created. */
        std::atomic<bool> createEnc;

        /**
         * Used to hold the colour separated framebuffer in main memory for the AVX based colour format conversion.
         */
        std::vector<BYTE> data;

        /** Used to hold the data of the framebuffer in main memory. */
        std::vector<BYTE> dataOpenGL;

        /** Remeber the size (widht * height) of the image as we need that for encoding. */
        size_t imageSize;

        /** The latency measurements for the encoding call. */
        LatencyMeasurement latencyMeasurementEncoding;

        /** The latency measurements for converting the frame from BGR to YUV420. */
        LatencyMeasurement latencyMeasurementFormatConversion;

        /** The latency measurements for downloading the frame from OpnGL. */
        LatencyMeasurement latencyMeasurementOpenGLCopy;

        /** The latency measurements for the processing of the output. */
        LatencyMeasurement latencyMeasurementProcessOutput;

        /** The timestamp of the frame that is encoded. */
        int64_t timestamp;

        /** Contains the data needed by x264 library for encoding. */
        X264Data x264Data;

        /** Contains the data needed by x265 library for encoding. */
        X265Data x265Data;
    };

} // namespace encoder
} /* end namespace megamol */

#endif /* ENCODER_CPUVIDEOENCODER_H_INCLUDED */
