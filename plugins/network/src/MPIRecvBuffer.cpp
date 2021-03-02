/*
 * MPIRecvBuffer.cpp
 *
 * Author: Florian Frieß
 * Copyright (C) 2021 by Universitaet Stuttgart (VISUS).
 * All rights reserved.
 */

#include "MPIRecvBuffer.h"


/*
 * megamol::network::MPIRecvBuffer::~MPIRecvBuffer
 */
megamol::network::MPIRecvBuffer::~MPIRecvBuffer(void) {}


/*
 * megamol::network::MPIRecvBuffer::CanTryPop
 */
bool megamol::network::MPIRecvBuffer::CanTryPop(void) {
    return this->canPop;
}


/*
 * megamol::network::MPIRecvBuffer::Full
 */
bool megamol::network::MPIRecvBuffer::Full(void) const {
    return (this->curRead.load() == this->advanceIndex(this->curWrite.load()));
}


/*
 * megamol::network::MPIRecvBuffer::MPIRecvBuffer
 */
megamol::network::MPIRecvBuffer::MPIRecvBuffer(void) : canPop(false), curRead(0), curWrite(0), queue() {
    // The queue should have at least a size of two.
    if (this->queue.size() < 2) {
        this->queue.resize(2);
    }
}


/*
 * megamol::network::MPIRecvBuffer::MPIRecvBuffer
 */
megamol::network::MPIRecvBuffer::MPIRecvBuffer(std::vector<RecvData>&& queue)
        : canPop(false), curRead(0), curWrite(0), queue(std::move(queue)) {
    // The queue should have at least a size of two.
    if (this->queue.size() < 2) {
        this->queue.resize(2);
    }
}


/*
 * megamol::network::MPIRecvBuffer::Peek
 */
megamol::network::RecvData* megamol::network::MPIRecvBuffer::Peek(void) {
    // Check if the buffer is not full.
    auto cur = this->curRead.load();
    if (cur != this->curWrite.load()) {
        return std::addressof(this->queue[cur]);
    } else {
        return nullptr;
    }
}


/*
 * megamol::network::MPIRecvBuffer::Peek
 */
megamol::network::RecvData* megamol::network::MPIRecvBuffer::Peek(void) const {
    return const_cast<MPIRecvBuffer*>(this)->Peek();
}


/*
 * megamol::network::MPIRecvBuffer::TryPop
 */
bool megamol::network::MPIRecvBuffer::TryPop(void) {
    // Check if the buffer is not empty.
    auto cur = this->curRead.load();
    if (cur != this->curWrite.load()) {
        this->canPop = false;
        this->curRead = this->advanceIndex(cur);
        return true;

    } else {
        return false;
    }
}


/*
 * megamol::network::MPIRecvBuffer::TryPop
 */
void megamol::network::MPIRecvBuffer::TryPopNext(void) {
    this->canPop = true;
}
