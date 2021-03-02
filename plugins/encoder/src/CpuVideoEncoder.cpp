/*
 * CpuVideoEncoder.cpp
 *
 * Author: Florian Frieß
 * Copyright (C) 2021 by Universitaet Stuttgart (VISUS).
 * All rights reserved.
 */

#include "CpuVideoEncoder.h"


/*
 * megamol::encoder::CpuVideoEncoder::~CpuVideoEncoder
 */
megamol::encoder::CpuVideoEncoder::~CpuVideoEncoder(void) {
    this->Release();
}


/*
 * megamol::encoder::CpuVideoEncoder::CpuVideoEncoder
 */
megamol::encoder::CpuVideoEncoder::CpuVideoEncoder(void)
        : megamol::encoder::AbstractVideoEncoder()
        , acceptFrames(false)
        , createEnc(true)
        , data()
        , dataOpenGL()
        , imageSize(0)
        , timestamp(0)
        , x264Data()
        , x265Data() {}


/*
 * megamol::encoder::CpuVideoEncoder::closeEncoderX264
 */
void megamol::encoder::CpuVideoEncoder::closeEncoderX264(void) {
    // Check if the encoder is in a valid state.
    if (this->x264Data.encoder == nullptr) {
        return;
    }

    // Flush the remaining frames from the encoder.
    int ret = 1;
    this->acceptFrames.store(false);
    while (::x264_encoder_delayed_frames(this->x264Data.encoder)) {
        ret = ::x264_encoder_encode(
            this->x264Data.encoder, &this->x264Data.nal, &this->x264Data.nalCnt, nullptr, &this->x264Data.outputFrame);
        if (ret > 0) {
            // There is some encoded data ready.
            for (int i = 0; i < this->x264Data.nalCnt; ++i) {
                // Process the encoded frame.
                this->processOutput(static_cast<void*>(this->x264Data.nal[i].p_payload),
                    static_cast<size_t>(this->x264Data.nal[i].i_payload), sizeof(uint8_t),
                    static_cast<size_t>(this->x264Data.outputFrame.i_pts));
            }
        }
    }

    // Close the encoder.
    ::x264_encoder_close(this->x264Data.encoder);
    this->x264Data.encoder = nullptr;
}


/*
 * megamol::encoder::CpuVideoEncoder::closeEncoderX265
 */
void megamol::encoder::CpuVideoEncoder::closeEncoderX265(void) {
    // Check if the encoder is in a valid state.
    if (this->x265Data.encoder == nullptr) {
        return;
    }

    // Flush the remaining frames from the encoder.
    int ret = 1;
    this->acceptFrames.store(false);
    while (ret > 0) {
        // Post a new flush command.
        ret = ::x265_encoder_encode(
            this->x265Data.encoder, &this->x265Data.nal, &this->x265Data.nalCnt, nullptr, &this->x265Data.outputFrame);

        // Check if there is a delayed frame.
        if (ret == 1) {
            // There is some encoded data ready.
            for (uint32_t i = 0; i < this->x265Data.nalCnt; ++i) {
                // Process the encoded frame.
                this->processOutput(static_cast<void*>(this->x265Data.nal[i].payload),
                    static_cast<size_t>(this->x265Data.nal[i].sizeBytes), sizeof(uint8_t),
                    static_cast<size_t>(this->x265Data.outputFrame.pts));
            }
        }
    }

    // Close the encoder.
    ::x265_encoder_close(this->x265Data.encoder);
    this->x265Data.encoder = nullptr;
}


/*
 * megamol::encoder::CpuVideoEncoder::convertBGRToYUV420
 */
void megamol::encoder::CpuVideoEncoder::convertBGRToYUV420(void) {
    // Check if AVX can be used.
    auto canUseAVX = (this->dataOpenGL.size() % sizeof(__m256i) == 0) && (this->imageSize % sizeof(__m256i) == 0) &&
                     (this->imageSize % sizeof(__m128i) == 0) && this->avxSupport && this->sseSupport &&
                     this->useSIMD.Param<megamol::core::param::BoolParam>()->Value();
    if (canUseAVX) {
        // Use AVX to separate the colour channels in order to speed up the format conversion.
        this->separateImageAVX();

        // Use AVX to convert the frame.
        this->convertImageAVX();

    } else {
        // Initialise the constants for the conversion.
        size_t yPos = 0;
        size_t uPos = 0;
        size_t vPos = 0;

        // Loop over each row of the image.
        const BYTE* bgraPtr = this->dataOpenGL.data();
        BYTE* dstY = this->x265Data.yuv420Image.data();
        BYTE* dstU = this->x265Data.yuv420Image.data() + this->imageSize;
        BYTE* dstV = this->x265Data.yuv420Image.data() + this->imageSize + this->imageSize / 4;
        for (size_t row = 0; row < height; ++row) {
            for (size_t x = 0; x < width; x += 1) {
                // Get the BGR values of the current pixel.
                uint8_t b = bgraPtr[yPos * 4 + 0];
                uint8_t g = bgraPtr[yPos * 4 + 1];
                uint8_t r = bgraPtr[yPos * 4 + 2];

                // Compute the Y value.
                dstY[yPos++] = ((66 * r + 129 * g + 25 * b) >> 8) + 16;

                // Check if this is an even or an odd row.
                if (row % 2 == 0 && x % 2 == 0) {
                    // Compute the U and V values.
                    dstU[uPos++] = ((-38 * r + -74 * g + 112 * b) >> 8) + 128;
                    dstV[vPos++] = ((112 * r + -94 * g + -18 * b) >> 8) + 128;
                }
            }
        }
    }
}


/*
 * megamol::encoder::CpuVideoEncoder::convertImageAVX
 */
void megamol::encoder::CpuVideoEncoder::convertImageAVX(void) {
    // Initialise the constants for the SIMD instructions used to convert the image.
    const __m256i constantY = _mm256_set1_epi16(16);
    const __m256i constantUV = _mm256_set1_epi16(128);
    const __m256i maskY = _mm256_set_epi8(
        15, 13, 11, 9, 7, 5, 3, 1, 14, 12, 10, 8, 6, 4, 2, 0, 15, 13, 11, 9, 7, 5, 3, 1, 14, 12, 10, 8, 6, 4, 2, 0);
    const __m256i maskUV = _mm256_set_epi8(
        15, 14, 13, 11, 10, 9, 7, 6, 5, 3, 2, 1, 12, 8, 4, 0, 15, 14, 13, 11, 10, 9, 7, 6, 5, 3, 2, 1, 12, 8, 4, 0);
    const __m128i maskStoreY = _mm_set_epi64x(0, -1);
    const __m128i maskStoreUV = _mm_set_epi32(0, 0, 0, -1);
    const __m256i mulBY = _mm256_set1_epi16(25);
    const __m256i mulGY = _mm256_set1_epi16(129);
    const __m256i mulRY = _mm256_set1_epi16(66);
    const __m256i mulBU = _mm256_set1_epi16(112);
    const __m256i mulGU = _mm256_set1_epi16(-74);
    const __m256i mulRU = _mm256_set1_epi16(-38);
    const __m256i mulBV = _mm256_set1_epi16(-18);
    const __m256i mulGV = _mm256_set1_epi16(-94);
    const __m256i mulRV = _mm256_set1_epi16(112);

    // Initialise the constants for the conversion.
    BYTE* dstY = this->x265Data.yuv420Image.data();
    BYTE* dstU = this->x265Data.yuv420Image.data() + this->imageSize;
    BYTE* dstV = this->x265Data.yuv420Image.data() + this->imageSize + this->imageSize / 4;
    __m256i* yOut = reinterpret_cast<__m256i*>(dstY);
    __m128i* uOut = reinterpret_cast<__m128i*>(dstU);
    __m128i* vOut = reinterpret_cast<__m128i*>(dstV);
    const __m256i permMaskY0 = _mm256_set_epi32(7, 6, 3, 2, 5, 4, 1, 0);
    const __m256i permMaskY1 = _mm256_set_epi32(5, 4, 1, 0, 2, 3, 6, 7);
    const __m256i permMaskUV0 = _mm256_set_epi32(7, 6, 5, 3, 2, 1, 4, 0);
    const __m256i permMaskUV1 = _mm256_set_epi32(7, 6, 5, 3, 4, 0, 2, 1);
    __m256i b0, b1, g0, g1, r0, r1, y0, y1, u, u0, v, v0;

    // Loop over the frame and get 16 pixels at once.
    size_t readsPerRow = static_cast<size_t>(width) / (2 * sizeof(__m128i));
    size_t totalIterations = this->imageSize / (2 * sizeof(__m128i));
    const __m128i* bPtr = reinterpret_cast<const __m128i*>(this->data.data());
    const __m128i* gPtr = reinterpret_cast<const __m128i*>(this->data.data() + this->imageSize);
    const __m128i* rPtr = reinterpret_cast<const __m128i*>(this->data.data() + (2 * this->imageSize));
    for (size_t j = 0; j < totalIterations; ++j) {
        // Read the first 32 blue values.
        b0 = _mm256_cvtepu8_epi16(_mm_stream_load_si128(bPtr++));
        b1 = _mm256_cvtepu8_epi16(_mm_stream_load_si128(bPtr++));

        // Read the first 32 green values.
        g0 = _mm256_cvtepu8_epi16(_mm_stream_load_si128(gPtr++));
        g1 = _mm256_cvtepu8_epi16(_mm_stream_load_si128(gPtr++));

        // Read the first 32 red values.
        r0 = _mm256_cvtepu8_epi16(_mm_stream_load_si128(rPtr++));
        r1 = _mm256_cvtepu8_epi16(_mm_stream_load_si128(rPtr++));

        // Compute the first 16 Y values.
        y0 = _mm256_adds_epu16(
            _mm256_srl_epi16(_mm256_add_epi16(_mm256_mullo_epi16(b0, mulBY),
                                 _mm256_add_epi16(_mm256_mullo_epi16(g0, mulGY), _mm256_mullo_epi16(r0, mulRY))),
                _mm_cvtsi32_si128(8)),
            constantY);
        y0 = _mm256_permutevar8x32_epi32(_mm256_shuffle_epi8(y0, maskY), permMaskY0);

        // Compute the second 16 Y values.
        y1 = _mm256_adds_epu16(
            _mm256_srl_epi16(_mm256_add_epi16(_mm256_mullo_epi16(b1, mulBY),
                                 _mm256_add_epi16(_mm256_mullo_epi16(g1, mulGY), _mm256_mullo_epi16(r1, mulRY))),
                _mm_cvtsi32_si128(8)),
            constantY);
        y1 = _mm256_permutevar8x32_epi32(_mm256_shuffle_epi8(y1, maskY), permMaskY1);

        // Combine the Y values and store them.
        _mm256_stream_si256(yOut++, _mm256_blend_epi32(y0, y1, 240));

        // Check if we are in an even row.
        size_t row = j / readsPerRow;
        if (row % 2 == 0) {
            // Compute the first 16 U values, of which we only need 8.
            u = _mm256_adds_epu16(
                _mm256_srl_epi16(_mm256_add_epi16(_mm256_mullo_epi16(b0, mulBU),
                                     _mm256_add_epi16(_mm256_mullo_epi16(g0, mulGU), _mm256_mullo_epi16(r0, mulRU))),
                    _mm_cvtsi32_si128(8)),
                constantUV);
            u = _mm256_permutevar8x32_epi32(_mm256_shuffle_epi8(u, maskUV), permMaskUV0);

            // Compute the second 16 U values, of which we only need 8.
            u0 = _mm256_adds_epu16(
                _mm256_srl_epi16(_mm256_add_epi16(_mm256_mullo_epi16(b1, mulBU),
                                     _mm256_add_epi16(_mm256_mullo_epi16(g1, mulGU), _mm256_mullo_epi16(r1, mulRU))),
                    _mm_cvtsi32_si128(8)),
                constantUV);
            u0 = _mm256_permutevar8x32_epi32(_mm256_shuffle_epi8(u0, maskUV), permMaskUV1);

            // Get the both 8 U values and combine them.
            u = _mm256_blend_epi32(u, u0, 12);

            // Compute the first 16 V values, of which we only need 8.
            v = _mm256_adds_epu16(
                _mm256_srl_epi16(_mm256_add_epi16(_mm256_mullo_epi16(b0, mulBV),
                                     _mm256_add_epi16(_mm256_mullo_epi16(g0, mulGV), _mm256_mullo_epi16(r0, mulRV))),
                    _mm_cvtsi32_si128(8)),
                constantUV);
            v = _mm256_permutevar8x32_epi32(_mm256_shuffle_epi8(v, maskUV), permMaskUV0);

            // Compute the second 16 V values, of which we only need 8.
            v0 = _mm256_adds_epu16(
                _mm256_srl_epi16(_mm256_add_epi16(_mm256_mullo_epi16(b1, mulBV),
                                     _mm256_add_epi16(_mm256_mullo_epi16(g1, mulGV), _mm256_mullo_epi16(r1, mulRV))),
                    _mm_cvtsi32_si128(8)),
                constantUV);
            v0 = _mm256_permutevar8x32_epi32(_mm256_shuffle_epi8(v0, maskUV), permMaskUV1);

            // Get the both 8 V values and combine them.
            v = _mm256_blend_epi32(v, v0, 12);

            // Write the 16 U and V values.
            _mm_stream_si128(uOut++, _mm256_extracti128_si256(u, 0));
            _mm_stream_si128(vOut++, _mm256_extracti128_si256(v, 0));
        }
    }
}


/*
 * megamol::encoder::CpuVideoEncoder::create
 */
bool megamol::encoder::CpuVideoEncoder::create(void) {
    // Call the create function of the parent first.
    auto retval = megamol::encoder::AbstractVideoEncoder::create();

    // Check if latency measurement is turned on.
    if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
        this->initialiseSpecialLatencyMeasurements();
    }
    return retval;
}


/*
 * megamol::encoder::CpuVideoEncoder::createEncoder
 */
bool megamol::encoder::CpuVideoEncoder::createEncoder(const int height, const int width) {
    // Initialise the buffer to hold the copy of the framebuffer in main memory.
    this->dataOpenGL.resize(static_cast<size_t>(this->width) * static_cast<size_t>(this->height) * size_t(4), 0);
    this->data.resize(static_cast<size_t>(this->width) * static_cast<size_t>(this->height) * size_t(3), 0);

    // Check which codec should be used, (0 == h264, 1 == h265).
    auto codec = this->encoderCodec.Param<core::param::EnumParam>()->Value();
    if (codec == 0) {
        // Create the x264 encoder.
        return this->createEncoderX264(height, width);

    } else {
        // Create the x265 encoder.
        return this->createEncoderX265(height, width);
    }
    return false;
}


/*
 * megamol::encoder::CpuVideoEncoder::createEncoderX264
 */
bool megamol::encoder::CpuVideoEncoder::createEncoderX264(const int height, const int width) {
    // Get the default encoder preset of x264.
    auto error = ::x264_param_default_preset(&this->x264Data.param, "ultrafast", "zerolatency");
    if (error < 0) {
        megamol::core::utility::log::Log::DefaultLog.WriteError(
            "Unable to load default parameter preset for x264 with error %d", error);
        return false;
    }

    // Change the parameter to what we want.
    this->x264Data.param.i_csp = X264_CSP_BGR;
    this->x264Data.param.i_width = width;
    this->x264Data.param.i_height = height;
    this->x264Data.param.b_vfr_input = 0;
    this->x264Data.param.b_repeat_headers = 1;
    this->x264Data.param.b_annexb = 1;
    this->x264Data.param.i_fps_num = 60;
    this->x264Data.param.i_fps_den = 1;
    // Change between key frames (aka GOP lenght = fps * 4) [bIntraRefresh = 0] and intra refresh
    // (no/infinite GOP length) [bIntraRefresh = 1].
    if (this->useInfiniteGOP.Param<core::param::BoolParam>()->Value()) {
        this->x264Data.param.b_intra_refresh = 1;
        this->x264Data.param.i_keyint_max = -1;

    } else {
        this->x264Data.param.b_intra_refresh = 0;
        this->x264Data.param.i_keyint_max = this->x264Data.param.i_fps_num * 4;
    }
    this->x264Data.param.rc.i_rc_method = X264_RC_CRF;
    this->x264Data.param.rc.i_qp_max = 51;
    this->x264Data.param.rc.i_qp_min = 1;
    this->x264Data.param.rc.f_rf_constant_max = 51;
    // Check if foveated encoding should be used.
    if (this->useFoveated.Param<core::param::BoolParam>()->Value()) {
        this->x264Data.param.rc.i_qp_constant = 51;
        this->x264Data.param.rc.f_rf_constant = 51;
        this->x264Data.param.rc.f_ip_factor = 0.1f;
        this->x264Data.param.rc.f_pb_factor = 0.1f;
        this->x264Data.param.rc.i_qp_step = 0;
        this->x264Data.param.rc.i_aq_mode = X264_AQ_VARIANCE;
        this->x264Data.param.rc.f_aq_strength = 1.0f;

    } else {
        this->x264Data.param.rc.i_qp_constant = this->qpValue.Param<megamol::core::param::IntParam>()->Value();
        this->x264Data.param.rc.f_rf_constant =
            static_cast<float>(this->qpValue.Param<megamol::core::param::IntParam>()->Value());
        this->x264Data.param.rc.i_aq_mode = X264_AQ_NONE;
    }

    // Apply the changes.
    error = ::x264_param_apply_profile(&this->x264Data.param, "high444");
    if (error < 0) {
        megamol::core::utility::log::Log::DefaultLog.WriteError(
            "Unable to apply changes to default parameter preset for x264 with error %d", error);
        return false;
    }

    // Open the encoder.
    this->x264Data.encoder = x264_encoder_open(&this->x264Data.param);
    if (this->x264Data.encoder == nullptr) {
        megamol::core::utility::log::Log::DefaultLog.WriteError("Unable to open x264 encoder with error %d", error);
        return false;
    }

    // Allocate the x265 input frames, holding the data that will be encoded.
    ::x264_picture_init(&this->x264Data.inputFrame);
    this->x264Data.inputFrame.img.i_csp = X264_CSP_BGR;
    this->x264Data.inputFrame.img.i_plane = 1;
    this->imageSize = static_cast<size_t>(width) * static_cast<size_t>(height);
    this->acceptFrames.store(true);
    return true;
}


/*
 * megamol::encoder::CpuVideoEncoder::createEncoderX265
 */
bool megamol::encoder::CpuVideoEncoder::createEncoderX265(const int height, const int width) {
    // Get the default encoder preset of x265.medium
    auto error = ::x265_param_default_preset(&this->x265Data.param, "ultrafast", "zerolatency");
    if (error < 0) {
        megamol::core::utility::log::Log::DefaultLog.WriteError(
            "Unable to load default parameter preset for x265 with error %d", error);
        return false;
    }

    // Change the parameter to what we want.
    this->x265Data.param.internalCsp = X265_CSP_I420;
    this->x265Data.param.sourceWidth = width;
    this->x265Data.param.sourceHeight = height;
    this->x265Data.param.bRepeatHeaders = 1;
    this->x265Data.param.fpsNum = 60;
    this->x265Data.param.fpsDenom = 1;
    // Change between key frames (aka GOP lenght = fps * 4) [bIntraRefresh = 0] and intra refresh
    // (no/infinite GOP length) [bIntraRefresh = 1].
    if (this->useInfiniteGOP.Param<core::param::BoolParam>()->Value()) {
        this->x265Data.param.bIntraRefresh = 1;
        this->x265Data.param.keyframeMax = -1;

    } else {
        this->x265Data.param.keyframeMax = this->x265Data.param.fpsNum * 4;
        this->x265Data.param.bIntraRefresh = 0;
    }
    this->x265Data.param.rc.rateControlMode = X265_RC_CRF;
    this->x265Data.param.rc.qpMax = 51;
    this->x265Data.param.rc.qpMin = 1;
    this->x265Data.param.rc.rfConstantMax = 51;
    // Check if foveated encoding should be used.
    if (this->useFoveated.Param<core::param::BoolParam>()->Value()) {
        this->x265Data.param.rc.aqMode = X265_AQ_VARIANCE;
        this->x265Data.param.rc.aqStrength = 1.0f;
        this->x265Data.param.rc.qp = 51;
        this->x265Data.param.rc.rfConstant = 51;
        this->x265Data.param.rc.ipFactor = 0.1f;
        this->x265Data.param.rc.pbFactor = 0.1f;
        this->x265Data.param.rc.qpStep = 0;

    } else {
        this->x265Data.param.rc.rfConstant =
            static_cast<double>(this->qpValue.Param<megamol::core::param::IntParam>()->Value());
        this->x265Data.param.rc.qp = this->qpValue.Param<megamol::core::param::IntParam>()->Value();
        this->x265Data.param.rc.aqMode = X265_AQ_NONE;
    }

    // Apply the changes.
    error = ::x265_param_apply_profile(&this->x265Data.param, "main");
    if (error < 0) {
        megamol::core::utility::log::Log::DefaultLog.WriteError(
            "Unable to apply changes to default parameter preset for x265 with error %d", error);
        return false;
    }

    // Open the encoder.
    this->x265Data.encoder= ::x265_encoder_open(&this->x265Data.param);
    if (this->x265Data.encoder == nullptr) {
        megamol::core::utility::log::Log::DefaultLog.WriteError("Unable to open x265 encoder with error %d", error);
        return false;
    }

    // Allocate the x265 input frames, holding the data that will be encoded.
    ::x265_picture_init(&this->x265Data.param, &this->x265Data.inputFrame);
    this->x265Data.yuv420Image.resize(width * height * 3 / 2, 0);
    this->imageSize = static_cast<size_t>(width) * static_cast<size_t>(height);
    this->acceptFrames.store(true);
    return true;
}


/*
 * megamol::encoder::CpuVideoEncoder::encodeFrame
 */
void megamol::encoder::CpuVideoEncoder::encodeFrame(void) {
    // Check if the encoder currently accepts frames.
    if (!this->acceptFrames.load()) {
        megamol::core::utility::log::Log::DefaultLog.WriteWarn(
            "The encoder is currently not accepting frames for encoding.");
        return;
    }

    // Check which codec should be used, (0 == h264, 1 == h265).
    auto codec = this->encoderCodec.Param<core::param::EnumParam>()->Value();
    if (codec == 0) {
        // Check if foveated encoding is requested.
        if (this->useFoveated.Param<megamol::core::param::BoolParam>()->Value()) {
            // Encode the frame using x264 foveated.
            this->encodeFrameX264Foveated();

        } else {
            // Encode the frame using x264.
            this->encodeFrameX264();
        }

    } else {
        // Check if the latency should be measured.
        double start = 0.0;
        if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
            start = vislib::sys::PerformanceCounter::QueryMillis();
        }

        // Check if the format is correct.
        if (this->x265Data.param.internalCsp != X265_CSP_BGR) {
            // Convert to NV12 as this is the other supportet format.
            this->convertBGRToYUV420();
        }

        // Check if the latency should be measured.
        if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
            this->latencyMeasurementFormatConversion.AddMeasurement(
                vislib::sys::PerformanceCounter::QueryMillis() - start);
        }

        // Check if foveated encoding is requested.
        if (this->useFoveated.Param<megamol::core::param::BoolParam>()->Value()) {
            // Encode the frame using x265 foveated.
            this->encodeFrameX265Foveated();

        } else {
            // Encode the frame using x265.
            this->encodeFrameX265();
        }
    }
}


/*
 * megamol::encoder::CpuVideoEncoder::encodeFrameX264
 */
void megamol::encoder::CpuVideoEncoder::encodeFrameX264(void) {
    // Copy the data to the input frame.
    this->x264Data.inputFrame.img.plane[0] = this->data.data();
    this->x264Data.inputFrame.img.i_stride[0] = width * 3;
    this->x264Data.inputFrame.i_pts = this->timestamp;

    // Check if the latency should be measured.
    double start = 0.0;
    if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
        start = vislib::sys::PerformanceCounter::QueryMillis();
    }

    // Encode the frame.pic.i_pts = i_frame;
    auto ret = ::x264_encoder_encode(this->x264Data.encoder, &this->x264Data.nal, &this->x264Data.nalCnt,
        &this->x264Data.inputFrame, &this->x264Data.outputFrame);

    // Check if the latency should be measured.
    if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
        this->latencyMeasurementEncoding.AddMeasurement(vislib::sys::PerformanceCounter::QueryMillis() - start);
    }

    // Check the return value of the encode call.
    if (ret < 0) {
        // An error occured in the last encode call.
        megamol::core::utility::log::Log::DefaultLog.WriteError(
            "Failed to encode the frame 0x%p with timestamp %lld with error %d", this->data.data(), this->timestamp,
            ret);

    } else {
        // Check if the encoder has produces encoded frames already (ret >= 1) or if the encoder pipeline is still
        // filling or is empty after flushing (ret == 0).
        if (ret >= 1) {
            // Check if the latency should be measured.
            start = 0.0;
            if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
                start = vislib::sys::PerformanceCounter::QueryMillis();
            }

            // There is some encoded data ready.
            for (int i = 0; i < this->x264Data.nalCnt; ++i) {
                // Process the encoded frame.
                this->processOutput(static_cast<void*>(this->x264Data.nal[i].p_payload),
                    static_cast<size_t>(this->x264Data.nal[i].i_payload), sizeof(uint8_t),
                    static_cast<size_t>(this->x264Data.outputFrame.i_pts));
            }

            // Check if the latency should be measured.
            if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
                auto end = vislib::sys::PerformanceCounter::QueryMillis();
                this->latencyMeasurementProcessOutput.AddMeasurement(end - start);
                this->latencyMeasurementOverall.AddMeasurement(
                    end - static_cast<double>(this->x264Data.outputFrame.i_pts));
            }
        }
    }
}


/*
 * megamol::encoder::CpuVideoEncoder::encodeFrameX264Foveated
 */
void megamol::encoder::CpuVideoEncoder::encodeFrameX264Foveated(void) {
    // Get the current quality map for the macroblocks.
    this->macroblockLock.Lock();
    size_t readIdx = this->macroblockQualityReadIdx.load();
    auto& curMap = this->macroblockQualityMapping[readIdx];

    // Set the current offsets for the input image.
    this->x264Data.inputFrame.prop.quant_offsets = curMap.data();

    // Copy the data to the input frame.
    this->x264Data.inputFrame.img.plane[0] = this->data.data();
    this->x264Data.inputFrame.img.i_stride[0] = width * 3;
    this->x264Data.inputFrame.i_pts = this->timestamp;

    // Check if the latency should be measured.
    double start = 0.0;
    if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
        start = vislib::sys::PerformanceCounter::QueryMillis();
    }

    // Encode the frame.pic.i_pts = i_frame;
    auto ret = ::x264_encoder_encode(this->x264Data.encoder, &this->x264Data.nal, &this->x264Data.nalCnt,
        &this->x264Data.inputFrame, &this->x264Data.outputFrame);
    this->macroblockLock.Unlock();

    // Check if the latency should be measured.
    if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
        this->latencyMeasurementEncoding.AddMeasurement(vislib::sys::PerformanceCounter::QueryMillis() - start);
    }

    // Check the return value of the encode call.
    if (ret < 0) {
        // An error occured in the last encode call.
        megamol::core::utility::log::Log::DefaultLog.WriteError(
            "Failed to encode the frame 0x%p with timestamp %lld with error %d", this->data.data(), this->timestamp,
            ret);

    } else {
        // Check if the encoder has produces encoded frames already (ret >= 1) or if the encoder pipeline is still
        // filling or is empty after flushing (ret == 0).
        if (ret >= 1) {
            // Check if the latency should be measured.
            start = 0.0;
            if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
                start = vislib::sys::PerformanceCounter::QueryMillis();
            }

            // There is some encoded data ready.
            for (int i = 0; i < this->x264Data.nalCnt; ++i) {
                // Process the encoded frame.
                this->processOutput(static_cast<void*>(this->x264Data.nal[i].p_payload),
                    static_cast<size_t>(this->x264Data.nal[i].i_payload), sizeof(uint8_t),
                    static_cast<size_t>(this->x264Data.outputFrame.i_pts));
            }

            // Check if the latency should be measured.
            if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
                auto end = vislib::sys::PerformanceCounter::QueryMillis();
                this->latencyMeasurementProcessOutput.AddMeasurement(end - start);
                this->latencyMeasurementOverall.AddMeasurement(
                    end - static_cast<double>(this->x264Data.outputFrame.i_pts));
            }
        }
    }
}


/*
 * megamol::encoder::CpuVideoEncoder::encodeFrameX265
 */
void megamol::encoder::CpuVideoEncoder::encodeFrameX265(void) {
    // Copy the data to the input frame.
    this->x265Data.inputFrame.pts = this->timestamp;
    this->x265Data.inputFrame.planes[0] = this->x265Data.yuv420Image.data();
    this->x265Data.inputFrame.planes[1] = this->x265Data.yuv420Image.data() + this->imageSize;
    this->x265Data.inputFrame.planes[2] = this->x265Data.yuv420Image.data() + this->imageSize + this->imageSize / 4;
    this->x265Data.inputFrame.stride[0] = this->width;
    this->x265Data.inputFrame.stride[1] = this->width / 2;
    this->x265Data.inputFrame.stride[2] = this->width / 2;

    // Check if the latency should be measured.
    double start = 0.0;
    if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
        start = vislib::sys::PerformanceCounter::QueryMillis();
    }

    // Encode the frame.
    auto ret = ::x265_encoder_encode(this->x265Data.encoder, &this->x265Data.nal, &this->x265Data.nalCnt,
        &this->x265Data.inputFrame, &this->x265Data.outputFrame);

    // Check if the latency should be measured.
    if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
        this->latencyMeasurementEncoding.AddMeasurement(vislib::sys::PerformanceCounter::QueryMillis() - start);
    }

    // Check the return value of the encode call.
    if (ret < 0) {
        // An error occured in the last encode call.
        megamol::core::utility::log::Log::DefaultLog.WriteError(
            "Failed to encode the frame 0x%p with timestamp %lld with error %d", this->x265Data.yuv420Image.data(),
            this->timestamp, ret);

    } else {
        // Check if the encoder has produces encoded frames already (ret == 1) or if the encoder pipeline is still
        // filling or is empty after flushing (ret == 0).
        if (ret == 1) {
            // Check if the latency should be measured.
            start = 0.0;
            if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
                start = vislib::sys::PerformanceCounter::QueryMillis();
            }

            // There is some encoded data ready.
            for (uint32_t i = 0; i < this->x265Data.nalCnt; ++i) {
                // Process the encoded frame.
                this->processOutput(static_cast<void*>(this->x265Data.nal[i].payload),
                    static_cast<size_t>(this->x265Data.nal[i].sizeBytes), sizeof(uint8_t),
                    static_cast<size_t>(this->x265Data.outputFrame.pts));
            }

            // Check if the latency should be measured.
            if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
                auto end = vislib::sys::PerformanceCounter::QueryMillis();
                this->latencyMeasurementProcessOutput.AddMeasurement(end - start);
                this->latencyMeasurementOverall.AddMeasurement(
                    end - static_cast<double>(this->x265Data.outputFrame.pts));
            }
        }
    }
}


/*
 * megamol::encoder::CpuVideoEncoder::encodeFrameX265Foveated
 */
void megamol::encoder::CpuVideoEncoder::encodeFrameX265Foveated(void) {
    // Get the current quality map for the macroblocks.
    this->macroblockLock.Lock();
    size_t readIdx = this->macroblockQualityReadIdx.load();
    auto& curMap = this->macroblockQualityMapping[readIdx];

    // Set the current offsets for the input image.
    this->x265Data.inputFrame.quantOffsets = curMap.data();

    // Copy the data to the input frame.
    this->x265Data.inputFrame.pts = this->timestamp;
    this->x265Data.inputFrame.planes[0] = this->x265Data.yuv420Image.data();
    this->x265Data.inputFrame.planes[1] = this->x265Data.yuv420Image.data() + this->imageSize;
    this->x265Data.inputFrame.planes[2] = this->x265Data.yuv420Image.data() + this->imageSize + this->imageSize / 4;
    this->x265Data.inputFrame.stride[0] = this->width;
    this->x265Data.inputFrame.stride[1] = this->width / 2;
    this->x265Data.inputFrame.stride[2] = this->width / 2;

    // Check if the latency should be measured.
    double start = 0.0;
    if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
        start = vislib::sys::PerformanceCounter::QueryMillis();
    }

    // Encode the frame.
    auto ret = ::x265_encoder_encode(this->x265Data.encoder, &this->x265Data.nal, &this->x265Data.nalCnt,
        &this->x265Data.inputFrame, &this->x265Data.outputFrame);
    this->macroblockLock.Unlock();

    // Check if the latency should be measured.
    if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
        this->latencyMeasurementEncoding.AddMeasurement(vislib::sys::PerformanceCounter::QueryMillis() - start);
    }

    // Check the return value of the encode call.
    if (ret < 0) {
        // An error occured in the last encode call.
        megamol::core::utility::log::Log::DefaultLog.WriteError(
            "Failed to encode the frame 0x%p with timestamp %lld with error %d", this->x265Data.yuv420Image.data(),
            this->timestamp, ret);

    } else {
        // Check if the encoder has produces encoded frames already (ret == 1) or if the encoder pipeline is still
        // filling or is empty after flushing (ret == 0).
        if (ret == 1) {
            // Check if the latency should be measured.
            start = 0.0;
            if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
                start = vislib::sys::PerformanceCounter::QueryMillis();
            }

            // There is some encoded data ready.
            for (uint32_t i = 0; i < this->x265Data.nalCnt; ++i) {
                // Process the encoded frame.
                this->processOutput(static_cast<void*>(this->x265Data.nal[i].payload),
                    static_cast<size_t>(this->x265Data.nal[i].sizeBytes), sizeof(uint8_t),
                    static_cast<size_t>(this->x265Data.outputFrame.pts));
            }

            // Check if the latency should be measured.
            if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
                auto end = vislib::sys::PerformanceCounter::QueryMillis();
                this->latencyMeasurementProcessOutput.AddMeasurement(end - start);
                this->latencyMeasurementOverall.AddMeasurement(
                    end - static_cast<double>(this->x265Data.outputFrame.pts));
            }
        }
    }
}


/*
 * megamol::encoder::CpuVideoEncoder::flipImage
 */
void megamol::encoder::CpuVideoEncoder::flipImage(void) {
    // Check if the frame should be flipped.
    if (this->flipFrames.Param<megamol::core::param::BoolParam>()->Value()) {
        // Check if the latency should be measured.
        double start = 0.0;
        if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
            start = vislib::sys::PerformanceCounter::QueryMillis();
        }

        // Initialise the necessary variables.
        BYTE* src = this->dataOpenGL.data();
        size_t step = static_cast<size_t>(this->width) * 4;
        size_t cntByte = sizeof(BYTE) * step;
        std::vector<BYTE> tmp = std::vector<BYTE>(step);
        BYTE* dst = this->dataOpenGL.data() + (static_cast<size_t>(this->height) - 1) * step;

        // Flipp the frame line by line.
        for (size_t i = 0; i < static_cast<size_t>(this->height) / 2; ++i) {
            // Copy the current line into the tmp.
            ::memcpy(tmp.data(), src, cntByte);

            // Copy the destination line to the current line.
            ::memcpy(src, dst, cntByte);

            // Copy the tmp line to the destination line.
            ::memcpy(dst, tmp.data(), cntByte);

            // Update the pointer.
            src += step;
            dst -= step;
        }

        // Check if the latency should be measured.
        if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
            this->latencyMeasurementFlippingFrame.AddMeasurement(
                vislib::sys::PerformanceCounter::QueryMillis() - start);
        }
    }
}


/*
 * megamol::encoder::CpuVideoEncoder::initialiseEncoder
 */
void megamol::encoder::CpuVideoEncoder::initialiseEncoder(void) {
    // Check if we already know the size of the incoming frames.
    if (this->width == 0 || this->height == 0) {
        return;
    }

    // Create the encoder, and check if it worked.
    if (!this->createEncoder(this->height, this->width)) {
        megamol::core::utility::log::Log::DefaultLog.WriteError("Unable to create encoder, not data will be encoded");
        return;
    }
}


/*
 * megamol::encoder::CpuVideoEncoder::initialiseSpecialLatencyMeasurements
 */
void megamol::encoder::CpuVideoEncoder::initialiseSpecialLatencyMeasurements(void) {
    // Check if latency measurements are turned on.
    if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
        // Check if the input mode is Call and not MPI.
        if (this->inputMode.Param<megamol::core::param::EnumParam>()->Value() == 0) {
            // Add measurements for the copy from OpenGL to main memory.
            this->latencyMeasurementOpenGLCopy.Initialise(2000, ".\\OpenGLCopy.csv");
        }

        // Check if the x265 encoder is used.
        if (this->encoderCodec.Param<megamol::core::param::EnumParam>()->Value() == 1) {
            // Add measurements for converting the image from BGR to YUV420.
            this->latencyMeasurementFormatConversion.Initialise(2000, ".\\FormatConversion.csv");
        }

        // Add measurements for encoding.
        this->latencyMeasurementEncoding.Initialise(2000, ".\\Encoding.csv");

        // Add measurements for encoding.
        this->latencyMeasurementProcessOutput.Initialise(2000, ".\\ProcessOutput.csv");
    }
}


/*
 * megamol::encoder::CpuVideoEncoder::onInputCall
 */
void megamol::encoder::CpuVideoEncoder::onInputCall(megamol::core::view::CallRender3D_2& call) {
    // Get the pointer to the framebuffer.
    auto* framebuffer = call.FrameBufferObject();

    // Get the width and height.
    if (framebuffer != nullptr) {
        // Get the width and height from the framebuffer.
        auto h = static_cast<int>(framebuffer->GetHeight());
        auto w = static_cast<int>(framebuffer->GetWidth());

        // Check if the resolution changed.
        if (this->height != h || this->width != w) {
            this->height = h;
            this->width = w;
            this->createEnc.store(true);
        }

    } else {
        // Get the width and height from the viewport.
        auto viewPort = call.GetViewport();
        auto h = viewPort.Height();
        auto w = viewPort.Width();

        // Check if the resolution changed.
        if (this->height != h || this->width != w) {
            this->height = h;
            this->width = w;
            this->createEnc.store(true);
        }
    }

    // Check if the encoder needs to be initialised.
    if (this->createEnc.load()) {
        // Initialise the encoder.
        this->initialiseEncoder();
        this->createEnc.store(false);
    }

    // Check which codec is used, (0 == h264, 1 == h265).
    auto codec = this->encoderCodec.Param<core::param::EnumParam>()->Value();
    if (codec == 0) {
        // Use the x264 encoder.
        this->onInputCallX264(call);

    } else {
        // Use the x265 encoder.
        this->onInputCallX265(call);
    }
}


/*
 * megamol::encoder::CpuVideoEncoder::onInputMPI
 */
#ifdef WITH_NETWORK
void megamol::encoder::CpuVideoEncoder::onInputMPI(
    std::vector<std::pair<void*, size_t>>& data, const size_t dataCnt, const megamol::network::IntraDataType dataType) {
    // Loop over all delivered data.
    for (size_t i = 0; i < dataCnt; ++i) {
        // Loop over all coalesced messages.
        size_t idx = 0;
        BYTE* dataPtr = reinterpret_cast<BYTE*>(data[i].first);
        while (idx < data[i].second) {
            // Extract the next message header
            auto header = reinterpret_cast<megamol::network::Message*>(dataPtr + idx);
            idx += sizeof(megamol::network::Message);

            // Check if this is a raw bitstream.
            if (dataType == megamol::network::IntraDataType::BITSTREAM) {
                // Check if the resolution changed.
                if (this->height != header->Height || this->width != header->Width) {
                    this->height = header->Height;
                    this->width = header->Width;
                    this->createEnc.store(true);
                }

                // Check if the encoder needs to be initialised.
                if (this->createEnc.load()) {
                    // Initialise the encoder.
                    this->initialiseEncoder();
                    this->createEnc.store(false);
                }

                // Check if the latency should be measured.
                double start = 0.0;
                if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
                    start = vislib::sys::PerformanceCounter::QueryMillis();
                    this->timestamp = static_cast<int64_t>(start);

                } else {
                    this->timestamp += header->Timestamp;
                }

                // Get the raw data.
                ::memcpy(this->data.data(), dataPtr + idx, header->DataCnt * sizeof(BYTE));

                // Check if the latency should be measured.
                if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
                    this->latencyMeasurementOpenGLCopy.AddMeasurement(
                        vislib::sys::PerformanceCounter::QueryMillis() - start);
                }

                // Encode the frame.
                this->encodeFrame();

            } else if (dataType == megamol::network::IntraDataType::FOVEATEDREGION) {
                // Get the pointer to the foveated rectangles and the number of valid rectangles.
                FovRect* regions = reinterpret_cast<FovRect*>(dataPtr + idx);
                uint32_t regionCnt = header->DataCnt / sizeof(FovRect);
                this->checkMBIntersectionFrame(regions, regionCnt, header->Timestamp);

            } else {
                // This type of data is not expected by the encoder so write a warning...
                #if 0
                megamol::core::utility::log::Log::DefaultLog.WriteWarn(
                    "CpuVideoEncoder received unexpected data of type %d via MPI. Skipping this...", static_cast<int>(dataType));
                #else
                this->processOutput(dataPtr + idx, header->DataCnt, sizeof(BYTE), header->Timestamp);
                #endif
            }

            // Skip the data.
            idx += header->DataCnt;
        }
    }
}
#endif


/*
 * megamol::encoder::CpuVideoEncoder::onInputCallX264
 */
void megamol::encoder::CpuVideoEncoder::onInputCallX264(megamol::core::view::CallRender3D_2& call) {
    // Check if the latency should be measured.
    double start = 0.0;
    if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
        start = vislib::sys::PerformanceCounter::QueryMillis();
        this->timestamp = static_cast<int64_t>(start);

    } else {
        this->timestamp += static_cast<int64_t>(std::round(call.LastFrameTime()));
    }

    // Get the raw data.
    auto* framebuffer = call.FrameBufferObject();
    if (framebuffer != nullptr) {
        // Copy the pixel values in the BGR format.
        framebuffer->GetColourTexture(this->dataOpenGL.data(), 0u, GL_BGR, GL_UNSIGNED_BYTE);

    } else {
        // Bind the output buffer of the call and read the pixel values in the BGR format.
        ::glReadBuffer(call.OutputBuffer());
        ::glReadPixels(0, 0, this->width, this->height, GL_BGR, GL_UNSIGNED_BYTE, this->data.data());
    }

    // Check if the latency should be measured.
    if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
        this->latencyMeasurementOpenGLCopy.AddMeasurement(vislib::sys::PerformanceCounter::QueryMillis() - start);
    }

    // Encode the frame.
    this->encodeFrame();
}


/*
 * megamol::encoder::CpuVideoEncoder::onInputCallX265
 */
void megamol::encoder::CpuVideoEncoder::onInputCallX265(megamol::core::view::CallRender3D_2& call) {
    // Check if the latency should be measured.
    double start = 0.0;
    if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
        start = vislib::sys::PerformanceCounter::QueryMillis();
        this->timestamp = static_cast<int64_t>(start);

    } else {
        this->timestamp += static_cast<int64_t>(std::round(call.LastFrameTime()));
    }

    // Get the raw data.
    auto* framebuffer = call.FrameBufferObject();
    if (framebuffer != nullptr) {
        // Copy the pixel values in the BGR format.
        framebuffer->GetColourTexture(this->dataOpenGL.data(), 0u, GL_BGRA, GL_UNSIGNED_BYTE);

    } else {
        // Bind the output buffer of the call and read the pixel values in the BGR format.
        ::glReadBuffer(call.OutputBuffer());
        ::glReadPixels(0, 0, this->width, this->height, GL_BGRA, GL_UNSIGNED_BYTE, this->dataOpenGL.data());
    }

    // Check if the latency should be measured.
    if (this->latencyMeasurement.Param<megamol::core::param::BoolParam>()->Value()) {
        this->latencyMeasurementOpenGLCopy.AddMeasurement(vislib::sys::PerformanceCounter::QueryMillis() - start);
    }

    // Encode the frame.
    this->encodeFrame();
}


/*
 * megamol::encoder::CpuVideoEncoder::release
 */
void megamol::encoder::CpuVideoEncoder::release(void) {
    // Call the release function of the parent first.
    megamol::encoder::AbstractVideoEncoder::release();

    // Stop the encoder.
    this->stopEncoder();
}


/*
 * megamol::encoder::CpuVideoEncoder::separateImageAVX
 */
void megamol::encoder::CpuVideoEncoder::separateImageAVX(void) {
    // Initialise the constants for the SIMD instructions used to separate the colour channels.
    const __m256i shuffleMask = _mm256_set_epi8(
        15, 11, 7, 3, 14, 10, 6, 2, 13, 9, 5, 1, 12, 8, 4, 0, 15, 11, 7, 3, 14, 10, 6, 2, 13, 9, 5, 1, 12, 8, 4, 0);
    const __m256i permMask0 = _mm256_set_epi32(7, 3, 6, 2, 5, 1, 4, 0);
    const __m256i permMask1 = _mm256_set_epi32(6, 2, 5, 1, 4, 0, 7, 3);
    const __m256i permMask2 = _mm256_set_epi32(5, 1, 4, 0, 7, 3, 6, 2);
    const __m256i permMask3 = _mm256_set_epi32(4, 0, 7, 3, 6, 2, 5, 1);
    const __m256i permMaskG = _mm256_set_epi32(1, 0, 7, 6, 5, 4, 3, 2);
    const __m256i permMaskR = _mm256_set_epi32(3, 2, 1, 0, 7, 6, 5, 4);

    // Initialise the source and destination pointer for the colour channel separation.
    __m256i permutated0, permutated1, permutated2, permutated3;
    __m256i* srcPtr = reinterpret_cast<__m256i*>(this->dataOpenGL.data());
    __m256i* dstPtrB = reinterpret_cast<__m256i*>(this->data.data());
    __m256i* dstPtrG = reinterpret_cast<__m256i*>(this->data.data() + this->imageSize);
    __m256i* dstPtrR = reinterpret_cast<__m256i*>(this->data.data() + (2 * this->imageSize));

    // Iterate over the source and separate the colour channels from BGRA to BB...GG...RR.
    size_t totalIterations = this->dataOpenGL.size() / (4 * sizeof(__m256i));
    for (size_t i = 0; i < totalIterations; ++i) {
        // Read and sort the data from BGRABGRA...BGRA to BBBBGGGGRRRRAAAA.
        permutated0 =
            _mm256_permutevar8x32_epi32(_mm256_shuffle_epi8(_mm256_stream_load_si256(srcPtr), shuffleMask), permMask0);
        srcPtr++;

        // Read and sort the data from BGRABGRA...BGRA to AAAABBBBGGGGRRRR.
        permutated1 =
            _mm256_permutevar8x32_epi32(_mm256_shuffle_epi8(_mm256_stream_load_si256(srcPtr), shuffleMask), permMask1);
        srcPtr++;

        // Read and sort the data from BGRABGRA...BGRA to RRRRAAAABBBBGGGG.
        permutated2 =
            _mm256_permutevar8x32_epi32(_mm256_shuffle_epi8(_mm256_stream_load_si256(srcPtr), shuffleMask), permMask2);
        srcPtr++;

        // Read and sort the data from BGRABGRA...BGRA to GGGGRRRRAAAABBBB.
        permutated3 =
            _mm256_permutevar8x32_epi32(_mm256_shuffle_epi8(_mm256_stream_load_si256(srcPtr), shuffleMask), permMask3);
        srcPtr++;

        // Write the blue colour values.
        _mm256_stream_si256(
            dstPtrB, _mm256_blend_epi32(permutated3,
                         _mm256_blend_epi32(permutated2, _mm256_blend_epi32(permutated0, permutated1, 12), 15), 63));
        dstPtrB++;

        // Resort the green colour values from G3,G0,G1,G2 to G0,G1,G2,G3 and write them.
        _mm256_stream_si256(dstPtrG,
            _mm256_permutevar8x32_epi32(
                _mm256_blend_epi32(permutated3,
                    _mm256_blend_epi32(permutated2, _mm256_blend_epi32(permutated0, permutated1, 48), 60), 252),
                permMaskG));
        dstPtrG++;

        // Resort the red colour values from R2,R3,R0,R1 to R0,R1,R2,R3 and write them.
        _mm256_stream_si256(dstPtrR,
            _mm256_permutevar8x32_epi32(
                _mm256_blend_epi32(permutated3,
                    _mm256_blend_epi32(permutated2, _mm256_blend_epi32(permutated0, permutated1, 192), 240), 243),
                permMaskR));
        dstPtrR++;
    }
}


/*
 * megamol::encoder::CpuVideoEncoder::stopEncoder
 */
void megamol::encoder::CpuVideoEncoder::stopEncoder(void) {
    // Check which codec should be used, (0 == h264, 1 == h265).
    auto codec = this->encoderCodec.Param<core::param::EnumParam>()->Value();
    if (codec == 0) {
        // Close the old encoder.
        this->closeEncoderX264();

    } else {
        // Close the old encoder.
        this->closeEncoderX265();
    }
}
