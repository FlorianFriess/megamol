/*
 * MPIRecvBuffer.h
 *
 * Author: Florian Frieß
 * Copyright (C) 2021 by Universitaet Stuttgart (VISUS).
 * All rights reserved.
 */

#ifndef NETWORK_MPIRECVBUFFER_H_INCLUDED
#define NETWORK_MPIRECVBUFFER_H_INCLUDED
#if (defined(_MSC_VER) && (_MSC_VER > 1000))
#pragma once
#endif /* (defined(_MSC_VER) && (_MSC_VER > 1000)) */

#include <atomic>
#include <vector>

#include "RecvData.h"

namespace megamol {
namespace network {

    /**
     * The ring buffer implementation that is used by the MpiIntraCommunicator to hold the received data.
     */
    class MPIRecvBuffer {
    public:
        /** Dtor */
        virtual ~MPIRecvBuffer(void);

        /**
         * Check if the canPop flag is 'true' and TryPop can be called.
         *
         * @returns The state of the canPop flag.
         */
        bool CanTryPop(void);
        
        /**
         * Answer whether the buffer is currently full.
         *
         * @return true if the buffer is full, false otherwise.
         */
        bool Full(void) const;

        /** Ctor */
        MPIRecvBuffer(void);

        /** Ctor */
        MPIRecvBuffer(std::vector<RecvData>&& queue);

        /**
         * Peek the ring buffer.
         *
         * @returns A nullptr in case the ring buffer is empty, a pointer to the current read element otherwise.
         */
        RecvData* Peek(void);

        /**
         * Peek the ring buffer.
         *
         * @returns A nullptr in case the ring buffer is empty, a pointer to the current read element otherwise.
         */
        RecvData* Peek(void) const;

        /**
         * Add an item item in-place.
         *
         * @tparam U An updater function which receives an parameter of type U&.
         *
         * @param updater The update functor. The functor must be a valid function pointer, lambda expression, etc.
         * The method will perform no checks before invoking it.
         *
         * @return true if the item was added, false if the queue was full.
         */
        template<class U>
        inline bool TryEmplace(U updater) {
            // Check if the write pointer can be advanced.
            auto cur = this->curWrite.load();
            auto nxt = this->advanceIndex(cur);
            if (nxt != this->curRead.load()) {
                // Buffer is not full so call the callback function and advance.
                updater(this->queue[cur]);
                this->curWrite = nxt;
                return true;

            } else {
                // Buffer is full...
                return false;
            }
        }

        /**
         * Advance the read pointer (if possible), discaring the actual return value.
         *
         * @return 'true' if an item was consumed, 'false' otherwise.
         */
        bool TryPop(void);

        /**
         * Do not advance the read pointer, instead setting the canPop flag to 'true' so that the read pointer
         * can be advanced later.
         */
        void TryPopNext(void);

    private:
        /**
         * Increments the given index, honouring the size of the ring buffer.
         *
         * @param idx The current index.
         *
         * @return The next index.
         */
        inline size_t advanceIndex(const size_t idx) const {
            return ((idx + 1) % this->queue.size());
        }

        /** Flag that indicates if TryPop can be called. */
        bool canPop;

        /** The index of the next element to be read. */
        std::atomic<size_t> curRead;

        /** The index of the next element to be written. */
        std::atomic<size_t> curWrite;

        /** The container holding the data. */
        std::vector<RecvData> queue;
    };

} // namespace network
} /* end namespace megamol */

#endif /* NETWORK_MPIRECVBUFFER_H_INCLUDED */
