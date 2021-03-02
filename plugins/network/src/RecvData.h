/*
 * RecvData.h
 *
 * Author: Florian Frieß
 * Copyright (C) 2021 by Universitaet Stuttgart (VISUS).
 * All rights reserved.
 */

#ifndef NETWORK_RECVDATA_H_INCLUDED
#define NETWORK_RECVDATA_H_INCLUDED
#if (defined(_MSC_VER) && (_MSC_VER > 1000))
#pragma once
#endif /* (defined(_MSC_VER) && (_MSC_VER > 1000)) */

#include <vector>

namespace megamol {
namespace network {

    /**
     * The underlying data structure that contains the information about the data received by the MpiIntraCommunicator.
     */
    class RecvData {
    public:
        /** Dtor */
        virtual ~RecvData(void);

        /** Ctor */
        RecvData(void);

        /**
         * The vector containing the data.
         */
        std::vector<unsigned char> Data;

        /**
         * The number of bytes in data.
         */
        int FrameSize;

        /**
         * The ID of the source of the data.
         */
        int Source;
    };

} // namespace network
} /* end namespace megamol */

#endif /* NETWORK_RECVDATA_H_INCLUDED */
