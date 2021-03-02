/*
 * RecvData.cpp
 *
 * Author: Florian Frieß
 * Copyright (C) 2021 by Universitaet Stuttgart (VISUS).
 * All rights reserved.
 */

#include "RecvData.h"


/*
 * megamol::network::RecvData::~RecvData
 */
megamol::network::RecvData::~RecvData(void) {}


/*
 * megamol::network::RecvData::RecvData
 */
megamol::network::RecvData::RecvData(void) : Data(), FrameSize(0), Source(-1) {}
