/*
 * MpiIntraCommunicator.cpp
 *
 * Author: Florian Frieß
 * Copyright (C) 2021 by Universitaet Stuttgart (VISUS).
 * All rights reserved.
 */

#include "MpiIntraCommunicator.h"


/*
 * megamol::network::MpiIntraCommunicator::~MpiIntraCommunicator
 */
megamol::network::MpiIntraCommunicator::~MpiIntraCommunicator(void) {
    this->Release();
}


/*
 * megamol::network::MpiIntraCommunicator::collectStreams
 */
bool megamol::network::MpiIntraCommunicator::collectStreams(
    const std::vector<StreamData>& streams, const int streamCnt, const int operationType) {
#ifdef WITH_MPI
    // Initialise the data that is used to collect the stream names via MPI.
    int res = 0;
    int myResponse = -1;
    int streamID = -1;
    std::vector<int> streamTypes(this->worldSize);
    std::vector<int> streamIDs(this->worldSize);

    // Set names of the stream types.
    std::vector<std::string> streamTypeNames = { "bit", "encoded bit", "camera", "foveated" };

    // Loop over the given stream data and intialise the given operations.
    bool retval = false;
    for (int i = 0; i < streamCnt; ++i) {
        // Get the responses of the current node.
        myResponse = -1;
        streamID = -1;
        if (streams.size() > i) {
            myResponse = static_cast<int>(streams[i].first);
            streamID = static_cast<int>(streams[i].second);
        }

        // Send the type of the provided/expected stream to all nodes.
        std::fill(streamTypes.begin(), streamTypes.end(), -1);
        {
            this->mpiLock.lock();
            res = ::MPI_Allgather(&myResponse, 1, MPI_INT, streamTypes.data(), 1, MPI_INT, this->megamolComm);
            CHECK_MPI_LOG_AND_THROW(res,
                "Failed to collect the type of the provided stream % d per node with error \"%s\".", i,
                this->getMPIError(res));
            this->mpiLock.unlock();
        }

        // Send the IDs of the provided/expected stream to all nodes.
        std::fill(streamIDs.begin(), streamIDs.end(), -1);
        {
            this->mpiLock.lock();
            res = ::MPI_Allgather(&streamID, 1, MPI_INT, streamIDs.data(), 1, MPI_INT, this->megamolComm);
            CHECK_MPI_LOG_AND_THROW(res,
                "Failed to collect the ID of the provided stream %d per node with error \"%s\".", i,
                this->getMPIError(res));
            this->mpiLock.unlock();
        }

        // Check if the current node expects any of the recevied streams.
        if (operationType == 0) {
            retval |= this->initialiseReceiveOperations(streamTypes, streamIDs, streamTypeNames);

        } else {
            retval |= this->initialiseExpectedOperations(streamTypes, streamIDs, streamTypeNames);
        }
    }
    return retval;
#else
    return false;
#endif
}


/*
 * megamol::network::MpiIntraCommunicator::communicate
 */
void megamol::network::MpiIntraCommunicator::communicate(void) {
    // Call DoDataExchange until the programme terminates or runAgain is false.
    bool runAgain = true;
    while (this->isRunning.load() && runAgain) {
        // Exchange the data via the MPI communicator.
        this->doDataExchange(runAgain);
    }
}


/*
 * megamol::network::MpiIntraCommunicator::create
 */
bool megamol::network::MpiIntraCommunicator::create(void) {
    return true;
}


/*
 * megamol::network::MpiIntraCommunicator::doDataExchange
 */
void megamol::network::MpiIntraCommunicator::doDataExchange(bool& runAgain) {
    #ifdef WITH_MPI
    // Run the initialisation of the stream and display nodes.
    if (!this->isInitialised) {
        // Get the number of data streams provided by each node in the MPI world and get the maximum provided stream
        // count.
        int myResponse = static_cast<int>(this->providedStreams.size());
        std::vector<int> providedStreamCnt(this->worldSize, -1);
        {
            this->mpiLock.lock();
            auto res =
                ::MPI_Allgather(&myResponse, 1, MPI_INT, providedStreamCnt.data(), 1, MPI_INT, this->megamolComm);
            CHECK_MPI_LOG_AND_THROW(res, "Failed to collect the number of provided streams per node with error \"%s\".",
                this->getMPIError(res));
            this->mpiLock.unlock();
        }
        auto maxProvidedStreamCnt = (*std::max_element(providedStreamCnt.begin(), providedStreamCnt.end()));

        // Get the number of data streams expected by each node in the MPI world and get the maximum expected stream
        // count.
        myResponse = static_cast<int>(this->expectedStreams.size());
        std::vector<int> expectedStreamCnt(this->worldSize, -1);
        {
            this->mpiLock.lock();
            auto res =
                ::MPI_Allgather(&myResponse, 1, MPI_INT, expectedStreamCnt.data(), 1, MPI_INT, this->megamolComm);
            CHECK_MPI_LOG_AND_THROW(res, "Failed to collect the number of expected streams per node with error \"%s\".",
                this->getMPIError(res));
            this->mpiLock.unlock();
        }
        auto maxExpectedStreamCnt = (*std::max_element(expectedStreamCnt.begin(), expectedStreamCnt.end()));

        // Loop over the provided streams and initialise the receive operations for the nodes that need to receieve
        // the provided stream.
        bool isExpecting = this->collectStreams(this->providedStreams, maxProvidedStreamCnt, 0);

        // Loop over the expected streams and initialise the send operations for the nodes that need to provide the
        // expected stream.
        bool isProvider = this->collectStreams(this->expectedStreams, maxExpectedStreamCnt, 1);

        // Get the management tag based on the highest tag used for the data exchange.
        this->managementData.Tag = 0;
        for (const auto& operation : this->receiveOperations) {
            this->managementData.Tag =
                (operation.ID > this->managementData.Tag) ? operation.ID : this->managementData.Tag;
        }
        for (const auto& operation : this->sendOperations) {
            this->managementData.Tag =
                (operation.ID > this->managementData.Tag) ? operation.ID : this->managementData.Tag;
        }
        this->managementData.Tag++;

        // Initialise the remaining management data.
        this->managementData.Completed = 0;
        if (this->worldRank == 0) {
            this->managementData.Dst = this->worldSize - 1;
        } else {
            this->managementData.Dst = this->worldRank - 1;
        }
        this->managementData.Src = (this->worldRank + 1) % this->worldSize;
        megamol::core::utility::log::Log::DefaultLog.WriteInfo(
            "Node %d uses %d as management tag, the src is %d and the dst is %d.", this->worldRank,
            this->managementData.Tag, this->managementData.Src, this->managementData.Dst);

        // Check if the node receives data and therefore needs a buffer.
        auto createBuffer = std::any_of(
            this->expectingStreams.begin(), this->expectingStreams.end(), [](bool val) { return val == true; });
        if (createBuffer) {
            // Get the number of threads that will be needed to deliver the received data from each node.
            size_t threadCnt = 0;
            for (const auto& operation : this->receiveOperations) {
                threadCnt += operation.srcNodes.size();
            }
            megamol::core::utility::log::Log::DefaultLog.WriteInfo(
                "Creating %zu thread(s) to handle incomming MPI streams.", threadCnt);

            // Create the ring buffers for receiving data.
            this->receiveBuffer = std::vector<std::shared_ptr<MPIRecvBuffer>>(threadCnt);
            this->receiveBufferData = std::vector<std::vector<RecvData>>(threadCnt);
            for (size_t i = 0; i < threadCnt; ++i) {
                this->receiveBufferData[i] = std::vector<RecvData>(1024);
                this->receiveBuffer[i] = std::make_shared<MPIRecvBuffer>(std::move(this->receiveBufferData[i]));
                megamol::core::utility::log::Log::DefaultLog.WriteInfo(
                    "Created ring buffer 0x%p for receiving data via MPI.", this->receiveBuffer[i].get());
            }

            // Create the mapping from the MPI node rank and the stream type to the corresponding ring buffer.
            // Additionally create a mapping between the data type and the index of the buffer that receivess data
            // belonging to the type.
            size_t index = 0;
            for (const auto& operation : this->receiveOperations) {
                for (const auto rank : operation.srcNodes) {
                    // Get the unique ID from the stream type (i) and the world rank of the node that provides the
                    // stream data.
                    auto uniqueID = this->getUniqueID(operation.ID, rank);

                    // Add to the ring buffer map.
                    this->receiveBufferMap.insert(std::make_pair(uniqueID, index));
                    megamol::core::utility::log::Log::DefaultLog.WriteInfo(
                        "Mapped source node %d with ID %d to ring buffer 0x%p for receive operation %d.", rank,
                        uniqueID, this->receiveBuffer[index].get(), operation.ID);

                    // Add to the deliver map.
                    this->receiveBufferDeliverMap.insert(std::make_pair(operation.type, index));
                    megamol::core::utility::log::Log::DefaultLog.WriteInfo(
                        "Mapped ring buffer (index %zu) to the data type %d.", index,
                        static_cast<int>(operation.type));
                    index++;
                }

                // Create the deliver event for the type of the operation in case it does not exist.
                auto it = this->deliverEvents.find(operation.type);
                if (it == this->deliverEvents.end()) {
                    this->deliverEvents.insert(
                        std::make_pair(operation.type, std::make_shared<vislib::sys::Event>(false, false)));
                }
            }
        }

        // Wait for all nodes to finish the initialisation then return.
        this->GlobalBarrier();

        // Change the state to is initialised and check if the node needs to run this function again.
        this->isInitialised = true;
        runAgain = isProvider || isExpecting;
        this->singleNode = (!runAgain) ? true : false;
        return;
    }

    // Check if the node can recieve any data.
    if (this->roles & static_cast<int>(Role::DATA_RECEIVER)) {
        for (auto& operation : this->receiveOperations) {
            // Check if the operation has completed.
            if (operation.inUse && this->expectingStreams[static_cast<int>(operation.type)]) {
                // Check if the current node can recieve data.
                int completionCnt = 0;
                for (size_t i = 0; i < operation.srcNodes.size(); ++i) {
                    if (operation.completed[i] != 1) {
                        // Probe the source for new data.
                        {
                            this->mpiLock.lock();
                            auto res = ::MPI_Iprobe(operation.srcNodes[i], operation.ID, this->megamolComm,
                                &operation.completed[i], &operation.status);
                            CHECK_MPI_LOG_AND_THROW(
                                res, "MPI_Iprobe failed with error \"%s\".", this->getMPIError(res));
                            this->mpiLock.unlock();
                        }

                        // If there is new data get the size and then recevie it.
                        if (operation.completed[i] == 1) {
                            // Get the size of the data the current node will recieve.
                            {
                                this->mpiLock.lock();
                                auto res = ::MPI_Get_count(&operation.status, MPI_BYTE, &operation.frameSize);
                                CHECK_MPI_LOG_AND_THROW(
                                    res, "MPI_Get_count failed with error \"%s\".", this->getMPIError(res));
                                this->mpiLock.unlock();
                            }

                            // Add the data to the ring buffer if possible.
                            auto uniqueID = this->getUniqueID(operation.ID, operation.status.MPI_SOURCE);
                            auto idx = this->receiveBufferMap[uniqueID];
                            if (!this->receiveBuffer[idx]->Full()) {
                                this->receiveBuffer[idx]->TryEmplace([&](RecvData& cur) {
                                    // Initialise the data vector.
                                    if (cur.Data.size() < static_cast<size_t>(operation.frameSize)) {
                                        cur.Data.resize(static_cast<size_t>(operation.frameSize));
                                    }
                                    cur.FrameSize = operation.frameSize;
                                    cur.Source = operation.status.MPI_SOURCE;

                                    // Receive the data.
                                    {
                                        this->mpiLock.lock();
                                        auto res = ::MPI_Recv(cur.Data.data(), operation.frameSize, MPI_BYTE,
                                            operation.status.MPI_SOURCE, operation.status.MPI_TAG, this->megamolComm,
                                            &operation.status);
                                        CHECK_MPI_LOG_AND_THROW(
                                            res, "MPI_Recv failed with error \"%s\".", this->getMPIError(res));
                                        this->mpiLock.unlock();
                                    }
                                });

                            } else {
                                // We still need to receive the data and then dropp  it.
                                megamol::core::utility::log::Log::DefaultLog.WriteWarn(
                                    "The receive buffer 0x%p is full.", this->receiveBuffer[idx].get());

                                // Resize the vector if necessary and receive the data.
                                if (this->droppData.size() < static_cast<size_t>(operation.frameSize)) {
                                    this->droppData.resize(static_cast<size_t>(operation.frameSize));
                                }
                                {
                                    this->mpiLock.lock();
                                    auto res = ::MPI_Recv(this->droppData.data(), operation.frameSize, MPI_BYTE,
                                        operation.status.MPI_SOURCE, operation.status.MPI_TAG, this->megamolComm,
                                        MPI_STATUS_IGNORE);
                                    CHECK_MPI_LOG_AND_THROW(
                                        res, "MPI_Recv failed with error \"%s\".", this->getMPIError(res));
                                    this->mpiLock.unlock();
                                }
                            }
                            this->deliverEvents[operation.type]->Set();

                            // Send the received frame ID back to the source to inform the source that the frame
                            // has been received.
                            operation.answer = operation.frameID;
                            {
                                this->mpiLock.lock();
                                auto res = ::MPI_Send(&operation.answer, 1, MPI_INT, operation.srcNodes[i],
                                    operation.ID, this->megamolComm);
                                CHECK_MPI_LOG_AND_THROW(
                                    res, "MPI_Send failed with error \"%s\".", this->getMPIError(res));
                                this->mpiLock.unlock();
                            }
                        }
                    }
                    completionCnt += operation.completed[i];
                }

                // If all nodes have recieved the data, reset the recieve operation for the stream type.
                bool completed = operation.completed.size() == static_cast<size_t>(completionCnt);
                if (completed) {
                    operation.frameID = MpiIntraCommunicator::increaseOperationFrameID(operation.frameID);
                    operation.inUse = false;
                    std::fill(operation.completed.begin(), operation.completed.end(), 0);
                }
            }
        }
    }

    // Check if there is new data to send from this node.
    if (this->roles & static_cast<int>(Role::DATA_PROVIDER)) {
        for (auto& operation : this->sendOperations) {
            // Check if there is a send operation for that stream type in progress.
            this->sendLocks[operation.sendBufferIdx]->lock();
            auto idx = this->nextSendBuffer[operation.sendBufferIdx];
            bool canSend = (!operation.inUse) && (this->providingStreams[static_cast<int>(operation.type)]) &&
                           (this->sendCount[operation.sendBufferIdx][idx] > 0);
            if (canSend) {
                // Note:
                // We can only send a maximum of ~2147483647 bytes per MPI_Isend as this is the maximum value of
                // an integer. If sendCount is bigger we get an overflow and try to send a negative amount of
                // data which leads to this error:
                // Fatal error in MPI_Isend: Invalid count, error stack:
                // MPI_Isend(buf = 0x000002075E3B6040, count = -2146701120, MPI_UNSIGNED_CHAR, dest = 1, tag = 0,
                //          comm = 0x84000000, request = 0x000002043B299AA0) failed
                // Negative count, value is - 2146701120
                //
                // Therefore we risk loosing data by cutting off the last bytes instead of MPI crashing the
                // rogramme.
                constexpr size_t intMax = static_cast<size_t>(std::numeric_limits<int>::max());
                operation.dataCnt = static_cast<int>(std::min(this->sendCount[operation.sendBufferIdx][idx], intMax));
                if (this->sendCount[operation.sendBufferIdx][idx] > intMax) {
                    megamol::core::utility::log::Log::DefaultLog.WriteWarn(
                        "Trying to send more data (%zu) than possible (%zu). Data will be lost as the number of bytes "
                        "is reduced.", this->sendCount[operation.sendBufferIdx][idx], intMax);
                }

                // Send the data to the clients.
                for (size_t i = 0; i < operation.destNodes.size(); ++i) {
                    // Send the data to the display node.
                    this->mpiLock.lock();
                    auto res = ::MPI_Isend(this->sendBuffer[operation.sendBufferIdx][idx].data(), operation.dataCnt,
                        MPI_UNSIGNED_CHAR, operation.destNodes[i], operation.ID, this->megamolComm,
                        &operation.requests[i]);
                    CHECK_MPI_LOG_AND_THROW(res, "MPI_Isend failed with error \"%s\".", this->getMPIError(res));
                    this->mpiLock.unlock();
                }

                // The asynchronoues sends started, fill the next buffer.
                operation.inUse = true;
                auto cur = this->nextSendBuffer[operation.sendBufferIdx];
                this->nextSendBuffer[operation.sendBufferIdx] =
                    (cur + 1) % this->sendBuffer[operation.sendBufferIdx].size();
                operation.sendIdx = cur;

                // Signal the thread to wait for the operation to complete.
                operation.waitEvent->Set();
            }
            this->sendLocks[operation.sendBufferIdx]->unlock();
        }
    }

	// Start a new recieve operation for each stream type.
	if (this->roles & static_cast<int>(Role::DATA_RECEIVER)) {
		for (auto& operation : this->receiveOperations) {
			// Check if there is a recieve operation for that stream type in progress.
			bool canReceive = (!operation.inUse) && (this->expectingStreams[static_cast<int>(operation.type)]);
			if (canReceive) {
				operation.inUse = true;
			}
		}
	}

    // Check if there is something on the management channel.
    {
        this->mpiLock.lock();
        auto res = ::MPI_Iprobe(this->managementData.Src, this->managementData.Tag, this->megamolComm,
            &this->managementData.Completed, &this->managementData.Status);
        CHECK_MPI_LOG_AND_THROW(res, "MPI_Iprobe failed with error \"%s\".", this->getMPIError(res));
        this->mpiLock.unlock();
    }
    if (this->managementData.Completed == 1) {
        // Receive the new state.
        {
            this->mpiLock.lock();
            auto res = ::MPI_Recv(&this->managementData.Data, 1, MPI_INT, this->managementData.Src,
                this->managementData.Tag, this->megamolComm, &this->managementData.Status);
            this->mpiLock.unlock();
        }

        // Forward the new state.
        {
            this->mpiLock.lock();
            auto res = ::MPI_Send(&this->managementData.Data, 1, MPI_INT, this->managementData.Dst,
                this->managementData.Tag, this->megamolComm);
            CHECK_MPI_LOG_AND_THROW(res, "MPI_Send failed with error \"%s\".", this->getMPIError(res));
            this->mpiLock.unlock();
        }

        // Check what to do.
        if (this->managementData.Data == -1) {
            // Reset the send and receive operations.
            this->reset();
        }

        // Reset the management data.
        this->managementData.Completed = 0;
        this->managementData.Data = 0;
    }
#else
    runAgain = false;
#endif
}


/*
 * megamol::network::MpiIntraCommunicator::GlobalBarrier
 */
void megamol::network::MpiIntraCommunicator::GlobalBarrier(void) const {
    #ifdef WITH_MPI
    this->mpiLock.lock();
    auto res = ::MPI_Barrier(this->megamolComm);
    CHECK_MPI_LOG_AND_THROW(res, "MPI barrier failed with error error \"%s\".", this->getMPIError(res));
    this->mpiLock.unlock();
    #endif
}


/*
 * megamol::network::MpiIntraCommunicator::getMPIError
 */
std::string megamol::network::MpiIntraCommunicator::getMPIError(const int errorCode) const {
    #ifdef WITH_MPI
    std::string msg;
    msg.resize(MPI_MAX_ERROR_STRING);
    int len = 0;
    if (::MPI_Error_string(errorCode, msg.data(), &len) == MPI_SUCCESS) {
        msg[len] = 0;
    }
    return msg;
    #else
    return "";
    #endif
}


/*
 * megamol::network::MpiIntraCommunicator::getUniqueID
 */
int megamol::network::MpiIntraCommunicator::getUniqueID(const int streamType, const int worldRank) {
    return (((streamType + worldRank) * (streamType + worldRank + 1)) / 2) + worldRank;
}


/*
 * megamol::network::MpiIntraCommunicator::increaseOperationFrameID
 */
int megamol::network::MpiIntraCommunicator::increaseOperationFrameID(const int oldFrameID) {
    return (oldFrameID + 1) % 1000000000;
}


/*
 * megamol::network::MpiIntraCommunicator::Initialise
 */
bool megamol::network::MpiIntraCommunicator::Initialise(const Role ownRole) {
    // Initialise the MPI session.
    if (!this->initMPI()) {
        megamol::core::utility::log::Log::DefaultLog.WriteError("Failed to initialise MPI.");
        return false;
    }

    // Check if the nodes provides data.
    {
        megamol::core::utility::log::Log::DefaultLog.WriteInfo("Negotiating which nodes provides data ...");
        int myResponse = (static_cast<int>(ownRole) & static_cast<int>(Role::DATA_PROVIDER)) ? this->worldRank : -1;
        this->sourceNodes.resize(this->worldSize);
        {
            this->mpiLock.lock();
            auto res =
                ::MPI_Allgather(&myResponse, 1, MPI_INT, this->sourceNodes.data(), 1, MPI_INT, this->megamolComm);
            CHECK_MPI_LOG_AND_THROW(
                res, "Failed to negotiate which nodes provide data with error \"%s\".", this->getMPIError(res));
            this->mpiLock.unlock();
        }

        // Remove all non-participating entries.
        typename std::vector<int>::iterator end = this->sourceNodes.end();
        sourceNodes.erase(std::remove(this->sourceNodes.begin(), end, -1), end);
        if (myResponse != -1) {
            megamol::core::utility::log::Log::DefaultLog.WriteInfo(
                "This node (%d) is an MPI data provider.", this->worldRank);
            this->roles |= static_cast<int>(Role::DATA_PROVIDER);
        }
    }

    // Check if the nodes receives data.
    {
        megamol::core::utility::log::Log::DefaultLog.WriteInfo("Negotiating which nodes receives data ...");
        int myResponse = (static_cast<int>(ownRole) & static_cast<int>(Role::DATA_RECEIVER)) ? this->worldRank : -1;
        this->targetNodes.resize(this->worldSize);
        {
            this->mpiLock.lock();
            auto res =
                ::MPI_Allgather(&myResponse, 1, MPI_INT, this->targetNodes.data(), 1, MPI_INT, this->megamolComm);
            CHECK_MPI_LOG_AND_THROW(
                res, "Failed to negotiate which nodes receive data with error \"%s\".", this->getMPIError(res));
            this->mpiLock.unlock();
        }

        // Remove all non-participating entries.
        typename std::vector<int>::iterator end = this->targetNodes.end();
        targetNodes.erase(std::remove(this->targetNodes.begin(), end, -1), end);
        if (myResponse != -1) {
            megamol::core::utility::log::Log::DefaultLog.WriteInfo(
                "This node (%d) is an MPI data receiver.", this->worldRank);
            this->roles |= static_cast<int>(Role::DATA_RECEIVER);
        }
    }
}


/*
 * megamol::network::MpiIntraCommunicator::initialiseExpectedOperations
 */
bool megamol::network::MpiIntraCommunicator::initialiseExpectedOperations(const std::vector<int>& streamTypes,
    const std::vector<int>& streamIDs, const std::vector<std::string>& streamTypeNames) {
    // Check if the current node provides any of the expected streams.
    std::map<std::pair<IntraDataType, int>, std::vector<int>> streamNodeMap;
    bool retval = false;
    for (auto& stream : this->providedStreams) {
        for (size_t i = 0; i < streamTypes.size(); ++i) {
            bool providesStream = (static_cast<int>(stream.first) == streamTypes[i]) && (stream.second == streamIDs[i]);
            if (providesStream) {
                // Check if the operation already exists.
                int existingOperation = -1;
                size_t idx = 0;
                for (const auto& operation : this->sendOperations) {
                    auto equal =
                        (static_cast<int>(operation.type) == streamTypes[i]) && (operation.uniqueID == streamIDs[i]);
                    if (equal) {
                        existingOperation = static_cast<int>(idx);
                        break;
                    }
                    idx++;
                }
                if (existingOperation == -1) {
                    // Get the send buffer based on the name of the stream.
                    auto it = this->sendBufferMap.find(streamIDs[i]);
                    if (it == this->sendBufferMap.end()) {
                        megamol::core::utility::log::Log::DefaultLog.WriteError(
                            "There is no buffer for the operation with the data ID %d.", streamIDs[i]);
                    }

                    // Initialise the new send operation struct.
                    this->sendOperations.emplace_back();
                    auto& operation = this->sendOperations.back();
                    operation.dataCnt = 0;
                    operation.destNodes.emplace_back(static_cast<int>(i));
                    operation.frameID = 0;
                    operation.ID = this->getUniqueID(streamIDs[i], this->worldRank);
                    operation.inUse = false;
                    operation.nodeCompleted.resize(operation.destNodes.size());
                    operation.requests.resize(operation.destNodes.size());
                    operation.responseCnt = 1;
                    operation.responseFrameID.resize(operation.destNodes.size());
                    operation.responseRequests.resize(operation.destNodes.size());
                    operation.responseStatus.resize(operation.destNodes.size());
                    operation.sendBufferIdx = it->second;
                    operation.sendIdx = 0;
                    operation.status.resize(operation.destNodes.size());
                    operation.type = static_cast<IntraDataType>(streamTypes[i]);
                    operation.uniqueID = streamIDs[i];
                    operation.waitEvent = std::make_shared<vislib::sys::Event>(false, false);
                    operation.waitThread = std::thread(std::bind(
                        &MpiIntraCommunicator::waitForResponses, std::ref(*this), this->sendOperations.size() - 1));
                    megamol::core::utility::log::Log::DefaultLog.WriteInfo(
                        "Created new operation %d for sending data to node %zu, for data ID %d.", operation.ID, i,
                        streamIDs[i]);

                } else {
                    // Add the current node as a source for the operation.
                    auto& operation = this->sendOperations[existingOperation];
                    operation.destNodes.emplace_back(static_cast<int>(i));
                    operation.nodeCompleted.resize(operation.destNodes.size());
                    operation.requests.resize(operation.destNodes.size());
                    operation.responseCnt++;
                    operation.responseFrameID.resize(operation.destNodes.size());
                    operation.responseRequests.resize(operation.destNodes.size());
                    operation.responseStatus.resize(operation.destNodes.size());
                    operation.status.resize(operation.destNodes.size());
                    megamol::core::utility::log::Log::DefaultLog.WriteInfo(
                        "Added node %zu to operation %d in order to send data belonging to data ID %d.", i,
                        operation.ID, streamIDs[i]);
                }

                // Add the new information to the map.
                auto key = std::make_pair(static_cast<IntraDataType>(streamTypes[i]), streamIDs[i]);
                auto it = streamNodeMap.find(key);
                if (it == streamNodeMap.end()) {
                    // Add the new key and the node.
                    std::vector<int> value = {static_cast<int>(i)};
                    streamNodeMap.insert(std::make_pair(key, value));

                } else {
                    // Add the node to the exisiting key.
                    it->second.emplace_back(static_cast<int>(i));
                }
                retval = true;
            }
        }
    }

    // Log the information and return.
    for (auto it = streamNodeMap.begin(); it != streamNodeMap.end(); ++it) {
        // Get all the destination nodes of the stream.
        std::string nodeInfo = "node ";
        if (it->second.size() > 1) {
            nodeInfo = "nodes ";
        }
        for (const auto& node : it->second) {
            nodeInfo.append(std::to_string(node).c_str());
            nodeInfo.append(", ");
        }
        nodeInfo = nodeInfo.substr(0, nodeInfo.length() - 2);

        // Logg the information about the current stream.
        megamol::core::utility::log::Log::DefaultLog.WriteInfo(
            "Node %d provides data with ID %d of type %sstream for %s.", this->worldRank, it->first.second,
            streamTypeNames[static_cast<int>(it->first.first)].c_str(), nodeInfo.c_str());
    }
    return retval;
}


/*
 * megamol::network::MpiIntraCommunicator::initialiseReceiveOperations
 */
bool megamol::network::MpiIntraCommunicator::initialiseReceiveOperations(const std::vector<int>& streamTypes,
    const std::vector<int>& streamIDs, const std::vector<std::string>& streamTypeNames) {
    // Check if the current node expects any of the recevied streams.
    std::map<std::pair<IntraDataType, int>, std::vector<int>> streamNodeMap;
    bool retval = false;
    for (size_t i = 0; i < streamTypes.size(); ++i) {
        // Check if the node expects the stream.
        for (auto& stream : this->expectedStreams) {
            bool expectsStream = (static_cast<int>(stream.first) == streamTypes[i]) && (stream.second == streamIDs[i]);
            if (expectsStream) {
                // Check if the operation already exists.
                int existingOperation = -1;
                size_t idx = 0;
                for (const auto& operation : this->receiveOperations) {
                    auto equal =
                        (static_cast<int>(operation.type) == streamTypes[i]) && (operation.uniqueID == streamIDs[i]);
                    if (equal) {
                        existingOperation = static_cast<int>(idx);
                        break;
                    }
                    idx++;
                }
                if (existingOperation == -1) {
                    // Initialise the new recevie operation struct.
                    this->receiveOperations.emplace_back();
                    auto& operation = this->receiveOperations.back();
                    operation.completed.resize(operation.completed.size() + 1, 0);
                    operation.frameID = 0;
                    operation.ID = this->getUniqueID(streamIDs[i], i);
                    operation.inUse = false;
                    operation.srcNodes.emplace_back(static_cast<int>(i));
                    operation.status = MPI_Status();
                    operation.type = static_cast<IntraDataType>(streamTypes[i]);
                    operation.uniqueID = streamIDs[i];
                    megamol::core::utility::log::Log::DefaultLog.WriteInfo(
                        "Created new operation %d for receiving data rom node %zu, for data ID %d.", operation.ID, i,
                        streamIDs[i]);

                } else {
                    // Add the current node as a source for the operation.
                    auto& operation = this->receiveOperations[existingOperation];
                    operation.completed.resize(operation.completed.size() + 1, 0);
                    operation.srcNodes.emplace_back(static_cast<int>(i));
                    megamol::core::utility::log::Log::DefaultLog.WriteInfo(
                        "Added node %zu to operation %d in order to receive data belonging to data ID %d.", i, operation.ID,
                        streamIDs[i]);
                }

                // Add the new information to the map.
                auto key = std::make_pair(static_cast<IntraDataType>(streamTypes[i]), streamIDs[i]);
                auto it = streamNodeMap.find(key);
                if (it == streamNodeMap.end()) {
                    // Add the new key and the node.
                    std::vector<int> value = {static_cast<int>(i)};
                    streamNodeMap.insert(std::make_pair(key, value));

                } else {
                    // Add the node to the exisiting key.
                    it->second.emplace_back(static_cast<int>(i));
                }
                retval = true;
            }
        }
    }

    // Log the information and return.
    for (auto it = streamNodeMap.begin(); it != streamNodeMap.end(); ++it) {
        // Get all the destination nodes of the stream.
        std::string nodeInfo = "node ";
        if (it->second.size() > 1) {
            nodeInfo = "nodes ";
        }
        for (const auto& node : it->second) {
            nodeInfo.append(std::to_string(node).c_str());
            nodeInfo.append(", ");
        }
        nodeInfo = nodeInfo.substr(0, nodeInfo.length() - 2);

        // Logg the information about the current stream.
        megamol::core::utility::log::Log::DefaultLog.WriteInfo(
            "Node %d receives data with ID %d of type %sstream (%d) from %s.", this->worldRank, it->first.second,
            streamTypeNames[static_cast<int>(it->first.first)].c_str(), static_cast<int>(it->first.first), nodeInfo.c_str());
    }
    return retval;
}


/*
 * megamol::network::MpiIntraCommunicator::MpiIntraCommunicator
 */
bool megamol::network::MpiIntraCommunicator::initMPI(void) {
    // Initialise the return value.
    bool retval = false;
    
    // Check if the intra comm has been initialised already.
    if (!this->initialiseMPI) {
        return true;
    }
    this->initialiseMPI = false;

    #ifdef WITH_MPI
    // Initialise MPI and the world communicator.
    if (this->megamolComm == MPI_COMM_NULL) {
        // Get the call.
        auto const call = this->requestMpi.CallAs<megamol::core::cluster::mpi::MpiCall>();
        if (call != nullptr) {
            // Let the MpiProvider do all the stuff.
            if ((*call)(core::cluster::mpi::MpiCall::IDX_PROVIDE_MPI)) {
                megamol::core::utility::log::Log::DefaultLog.WriteInfo("Got MPI world communicator.");
                this->megamolComm = call->GetComm();

            } else {
                megamol::core::utility::log::Log::DefaultLog.WriteError(
                    "Could not retrieve MPI world communicator from the registered provider module.");
            }

        } else {
            megamol::core::utility::log::Log::DefaultLog.WriteError(
                "MPI cannot be initialized lazily. Please initialize MPI before using this module.");
        }

        // Get the world rank and the size of the world.
        if (this->megamolComm != MPI_COMM_NULL) {
            megamol::core::utility::log::Log::DefaultLog.WriteInfo(
                "MPI is ready, retrieving world communicator properties ...");
            this->worldRank = call->GetRank();
            this->worldSize = call->GetCommSize();
            megamol::core::utility::log::Log::DefaultLog.WriteInfo(
                "MpiIntraCommunicator on %hs has world rank %d of %d.",
                vislib::sys::SystemInformation::ComputerNameA().PeekBuffer(), this->worldRank, this->worldSize);
        }
    }

    // Determine success of the whole operation.
    retval = (this->megamolComm != MPI_COMM_NULL);
    #endif
    return retval;
}


/*
 * megamol::network::MpiIntraCommunicator::MpiIntraCommunicator
 */
megamol::network::MpiIntraCommunicator::MpiIntraCommunicator(void)
        : droppData()
        , expectedStreams()
        , expectingStreams(INTRADATATYPECNT, false)
        , initialiseMPI(true)
        , isInitialised(false)
        , managementData()
        #ifdef WITH_MPI
        , megamolComm(MPI_COMM_NULL)
        #else
        , megamolComm(0x04000000)
        #endif
        , mpiIntraCall("mpiIntraCall", "Call to send and receive data via MPI")
        , nextSendBuffer()
        , providedStreams()
        , providingStreams(INTRADATATYPECNT, false)
        , requestMpi("requestMPI", "Requests initialization of MPI and the communicator")
        , roles(static_cast<int>(Role::NONE))
        , sendBuffer()
        , sendCount()
        , sendLocks()
        , singleNode(false)
        , worldRank(0)
        , worldSize(0) {
    // Initialise the callback functions for the MpiIntraCall.
    this->mpiIntraCall.SetCallback(MpiIntraCall::ClassName(),
        MpiIntraCall::FunctionName(MpiIntraCall::IDX_DELIVER_DATA), &MpiIntraCommunicator::onDeliverData);
    this->mpiIntraCall.SetCallback(MpiIntraCall::ClassName(), MpiIntraCall::FunctionName(MpiIntraCall::IDX_SEND_DATA),
        &MpiIntraCommunicator::onSendData);
    this->mpiIntraCall.SetCallback(MpiIntraCall::ClassName(),
        MpiIntraCall::FunctionName(MpiIntraCall::IDX_ADD_EXPECTED_DATA), &MpiIntraCommunicator::onAddExpectedData);
    this->mpiIntraCall.SetCallback(MpiIntraCall::ClassName(),
        MpiIntraCall::FunctionName(MpiIntraCall::IDX_ADD_PROVIDED_DATA), &MpiIntraCommunicator::onAddProvidedData);
    this->mpiIntraCall.SetCallback(MpiIntraCall::ClassName(), MpiIntraCall::FunctionName(MpiIntraCall::IDX_INIT_COMM),
        &MpiIntraCommunicator::onInitialise);
    this->mpiIntraCall.SetCallback(MpiIntraCall::ClassName(), MpiIntraCall::FunctionName(MpiIntraCall::IDX_START_COMM),
        &MpiIntraCommunicator::onStart);
    this->mpiIntraCall.SetCallback(MpiIntraCall::ClassName(), MpiIntraCall::FunctionName(MpiIntraCall::IDX_STOP_COMM),
        &MpiIntraCommunicator::onStop);
    this->MakeSlotAvailable(&this->mpiIntraCall);

    // Initialise the call to the MpiProvider.
    this->requestMpi.SetCompatibleCall<megamol::core::cluster::mpi::MpiCallDescription>();
    this->MakeSlotAvailable(&this->requestMpi);
}


/*
 * megamol::network::MpiIntraCommunicator::onAddExpectedData
 */
bool megamol::network::MpiIntraCommunicator::onAddExpectedData(megamol::core::Call& call) {
    // Get the MpiIntraCall and add the data type to the expected streams.
    auto& c = dynamic_cast<MpiIntraCall&>(call);
    this->SetExpectedStreamType(c.GetDataType(), c.GetDataID());
    return true;
}


/*
 * megamol::network::MpiIntraCommunicator::onAddProvidedData
 */
bool megamol::network::MpiIntraCommunicator::onAddProvidedData(megamol::core::Call& call) {
    // Get the MpiIntraCall and add the data type to the provided streams.
    auto& c = dynamic_cast<MpiIntraCall&>(call);
    this->SetProvidedStreamType(c.GetDataType(), c.GetDataID());
    return true;
}


/*
 * megamol::network::MpiIntraCommunicator::onDeliverData
 */
bool megamol::network::MpiIntraCommunicator::onDeliverData(megamol::core::Call& call) {
    // Check if the intra comm is ready.
    if (!this->isInitialised) {
        // Wait for a short time then return without data.
        ::Sleep(10);
        return true;
    }

    // Get the MpiIntraCall and the type of data the caller wants.
    auto& c = dynamic_cast<MpiIntraCall&>(call);

    // Check if the deliver event exists.
    if (this->deliverEvents.find(c.GetDataType()) == this->deliverEvents.end()) {
        megamol::core::utility::log::Log::DefaultLog.WriteError(
            "Failed to find receive buffer for data of type %d.", static_cast<int>(c.GetDataType()));
        return false;
    }

    // Wait until some data is ready, or 10 seconds are over.
    if (!this->deliverEvents[c.GetDataType()]->Wait(10000)) {
        // Timeout, no data is ready.
        c.SetDeliverDataCnt(0);
        megamol::core::utility::log::Log::DefaultLog.WriteWarn(
            "Timeout for data of type %d.", static_cast<int>(c.GetDataType()));
        return true;
    }

    // Get all receive buffer that contain data of the requested type.
    size_t idx = 0;
    auto range = this->receiveBufferDeliverMap.equal_range(c.GetDataType());
    for (auto it = range.first; it != range.second; ++it) {
        // Get the index of the buffer.
        auto index = it->second;

        // Check if the read index needs to be advanced.
        if (this->receiveBuffer[index]->CanTryPop()) {
            // Pop the element from the ring buffer.
            if (!this->receiveBuffer[index]->TryPop()) {
                megamol::core::utility::log::Log::DefaultLog.WriteWarn(
                    "Could not pop item from receive buffer 0x%p.", this->receiveBuffer[index].get());
                return false;
            }
        }

        // Get the current element
        auto element = this->receiveBuffer[index]->Peek();
        if (element != nullptr) {
            if (element->FrameSize > 0) {
                c.AddDeliverData(element->Data.data(), static_cast<size_t>(element->FrameSize), idx++);
            }

            // Pop the element from the ring buffer in the next call.
            this->receiveBuffer[index]->TryPopNext();
        }
    }
    c.SetDeliverDataCnt(idx);
    return true;
}


/*
 * megamol::network::MpiIntraCommunicator::onInitialise
 */
bool megamol::network::MpiIntraCommunicator::onInitialise(megamol::core::Call& call) {
    // Get the MpiIntraCall and initialise the MpiIntraCommunicator.
    auto& c = dynamic_cast<MpiIntraCall&>(call);
    return this->Initialise(c.GetRoles());
}


/*
 * megamol::network::MpiIntraCommunicator::onSendData
 */
bool megamol::network::MpiIntraCommunicator::onSendData(megamol::core::Call& call) {
    // Get the MpiIntraCall and add the data to the corresponding send buffer.
    auto& c = dynamic_cast<MpiIntraCall&>(call);
    this->sendData(c.GetData(), c.GetDataCnt(), c.GetDataID(), c.GetTimestamp(), c.GetHeight(), c.GetWidth());
    return true;
}


/*
 * megamol::network::MpiIntraCommunicator::onStart
 */
bool megamol::network::MpiIntraCommunicator::onStart(megamol::core::Call& call) {
    // Start the MpiIntraCommunicator.
    this->Start();
    return true;
}


/*
 * megamol::network::MpiIntraCommunicator::onStop
 */
bool megamol::network::MpiIntraCommunicator::onStop(megamol::core::Call& call) {
    // Stop the MpiIntraCommunicator.
    this->Stop();
    return true;
}


/*
 * megamol::network::MpiIntraCommunicator::release
 */
void megamol::network::MpiIntraCommunicator::release(void) {
    // Stop the communication thread.
    this->Stop();
}


/*
 * megamol::network::MpiIntraCommunicator::reset
 */
void megamol::network::MpiIntraCommunicator::reset(void) {
    // Log the call to reset.
    megamol::core::utility::log::Log::DefaultLog.WriteInfo(
        "Node %d received a reset command via the management channel.", this->worldRank);

    // Cancel all outstanding MPI_Isend operations.
    if (this->roles & static_cast<int>(Role::DATA_PROVIDER)) {
        for (auto& operation : this->sendOperations) {
            // Loop over all destination nodes and cancel the send operation.
            for (size_t i = 0; i < operation.destNodes.size(); ++i) {
                // Cancel the send operation.
                {
                    this->mpiLock.lock();
                    auto res = ::MPI_Cancel(&operation.requests[i]);
                    CHECK_MPI_LOG_AND_THROW(res, "MPI_Cancel failed with error \"%s\".", this->getMPIError(res));
                    this->mpiLock.unlock();
                }

                // Wait for the send operations to be cancled.
                {
                    this->mpiLock.lock();
                    auto res = ::MPI_Wait(&operation.requests[i], MPI_STATUSES_IGNORE);
                    CHECK_MPI_LOG_AND_THROW(res, "MPI_Wait failed with error \"%s\".", this->getMPIError(res));
                    this->mpiLock.unlock();
                }
            }
        }
    }

    // Stop the send operation threads.
    for (auto& operation : this->sendOperations) {
        operation.waitEvent->Set();
        if (operation.waitThread.joinable()) {
            operation.waitThread.join();
        }
    }

    // Clear the send and receive operations.
    this->receiveOperations.clear();
    this->sendOperations.clear();
    this->isInitialised = false;

    // Wait for all nodes.
    megamol::core::utility::log::Log::DefaultLog.WriteInfo(
        "Node %d canceled all outstanding operations, waiting for the other nodes.", this->worldRank);
    this->GlobalBarrier();

    // Initialise the node again.
    megamol::core::utility::log::Log::DefaultLog.WriteInfo("Node %d initialising again.", this->worldRank);
    this->Initialise(static_cast<Role>(this->roles));
}


/*
 * megamol::network::MpiIntraCommunicator::sendData
 */
void megamol::network::MpiIntraCommunicator::sendData(const void* data, const size_t cntData, const size_t dataID,
    const int64_t timestamp, const int height, const int width) {
    // Check if the MpiIntraCommunicator is initialised.
    if (!this->isInitialised) {
        return;
    }

    // Get the send buffer based on the unique ID.
    auto it = this->sendBufferMap.find(dataID);
    if (it == this->sendBufferMap.end()) {
        megamol::core::utility::log::Log::DefaultLog.WriteError(
            "There is no buffer for the ID %zu. The data will be dropped.", dataID);
        return;
    }
    auto sendBufferIdx = it->second;

    // Create the header of the data.
    Message header = Message(cntData, dataID, height, timestamp, width);

    // Add the data to the buffer.
    this->sendLocks[sendBufferIdx]->lock();
    auto idx = this->nextSendBuffer[sendBufferIdx];
    auto offset = this->sendCount[sendBufferIdx][idx];
    this->sendCount[sendBufferIdx][idx] += sizeof(Message) + cntData;
    if (this->sendBuffer[sendBufferIdx][idx].size() < this->sendCount[sendBufferIdx][idx]) {
        this->sendBuffer[sendBufferIdx][idx].resize(this->sendCount[sendBufferIdx][idx] * 2);
    }
    ::memcpy(this->sendBuffer[sendBufferIdx][idx].data() + (offset * sizeof(BYTE)), &header, sizeof(Message));
    ::memcpy(this->sendBuffer[sendBufferIdx][idx].data() + (offset * sizeof(BYTE)) + sizeof(Message), data,
        cntData * sizeof(BYTE));
    this->sendLocks[sendBufferIdx]->unlock();
}


/*
 * megamol::network::MpiIntraCommunicator::SetExpectedStreamType
 */
void megamol::network::MpiIntraCommunicator::SetExpectedStreamType(const IntraDataType type, const size_t dataID) {
    // Remember that the node expects data of the given type with the given unique ID.
    megamol::core::utility::log::Log::DefaultLog.WriteInfo(
        "The node expects data of type %d with the unique ID %zu.", static_cast<int>(type), dataID);
    this->expectingStreams[static_cast<int>(type)] = true;
    this->expectedStreams.emplace_back(std::make_pair(type, dataID));
}


/*
 * megamol::network::MpiIntraCommunicator::SetProvidedStreamType
 */
void megamol::network::MpiIntraCommunicator::SetProvidedStreamType(const IntraDataType type, const size_t dataID) {
    // Remember that the node provides data of the given type with the given unique ID.
    megamol::core::utility::log::Log::DefaultLog.WriteInfo(
        "The node provides data of type %d with the unique ID %zu.", static_cast<int>(type), dataID);
    this->providingStreams[static_cast<int>(type)] = true;
    this->providedStreams.emplace_back(std::make_pair(type, dataID));

    // Resize the send buffer and the corresponding data.
    this->nextSendBuffer.emplace_back(0);
    this->sendBuffer.emplace_back();
    this->sendBuffer.back().resize(3);
    this->sendCount.emplace_back();
    this->sendCount.back().resize(3, 0);
    this->sendLocks.emplace_back(std::make_shared<std::mutex>());

    // Add the unique ID to the sendBuffer map.
    if (this->sendBufferMap.find(dataID) != this->sendBufferMap.end()) {
        megamol::core::utility::log::Log::DefaultLog.WriteError(
            "The ID of the data must be unique. ID %zu is already in the send buffer map of MpiIntraCommunicator 0x%p.",
            dataID, this);
        throw std::exception("SetProvidedStreamType failed because ID was not unique.");
    }
    this->sendBufferMap.insert(std::make_pair(dataID, this->nextSendBuffer.size() - 1));
}


/*
 * megamol::network::MpiIntraCommunicator::Start
 */
void megamol::network::MpiIntraCommunicator::Start(void) {
    // Check if the MpiIntraCommunicator has been initialised already.
    if (this->initialiseMPI) {
        megamol::core::utility::log::Log::DefaultLog.WriteError("MpiIntraCommunicator is not yet initialised.");
        return;
    }

    // Set the flag that controls the lifetime of the communication thread.
    this->isRunning.store(true);

    // Start the communication thread.
    try {
        this->communicationThread = std::thread(std::bind(&MpiIntraCommunicator::communicate, std::ref(*this)));
    } catch (std::exception& ex) {
        megamol::core::utility::log::Log::DefaultLog.WriteError(
            "Failed to start the communication thread with error \"%s\".", ex.what());
    }
}


/*
 * megamol::network::MpiIntraCommunicator::Stop
 */
void megamol::network::MpiIntraCommunicator::Stop(void) {
    // Stop the communication thread.
    megamol::core::utility::log::Log::DefaultLog.WriteInfo(
        "Stopping MpiIntraCommunicator on node %d.", this->worldRank);
    this->isRunning.store(false);
    if (this->communicationThread.joinable()) {
        this->communicationThread.join();
    }

    // Stop the send operation threads.
    for (auto& operation : this->sendOperations) {
        operation.waitEvent->Set();
        if (operation.waitThread.joinable()) {
            operation.waitThread.join();
        }
    }

    // Clear the send and receive operations as well as the provided and expected streams.
    this->receiveOperations.clear();
    std::fill(this->expectingStreams.begin(), this->expectingStreams.end(), false);
    this->expectedStreams.clear();
    this->sendOperations.clear();
    std::fill(this->providingStreams.begin(), this->providingStreams.end(), false);
    this->providedStreams.clear();
    this->nextSendBuffer.clear();
    this->sendBuffer.clear();
    this->sendCount.clear();
    this->sendLocks.clear();
    this->sendBufferMap.clear();

    // Reset the role.
    this->roles = static_cast<int>(Role::NONE);
    this->isInitialised = false;

    // Tell the next node that the send and receive operations need to be reinitialised.
    megamol::core::utility::log::Log::DefaultLog.WriteInfo(
        "Informing node %d that a restart is necessary.", this->managementData.Dst);
    this->managementData.Data = -1;
    {
        this->mpiLock.lock();
        auto res = ::MPI_Send(&this->managementData.Data, 1, MPI_INT, this->managementData.Dst,
            this->managementData.Tag, this->megamolComm);
        CHECK_MPI_LOG_AND_THROW(res, "MPI_Send failed with error \"%s\".", this->getMPIError(res));
        this->mpiLock.unlock();
    }

    // Receive the new state.
    {
        this->mpiLock.lock();
        auto res = ::MPI_Recv(&this->managementData.Data, 1, MPI_INT, this->managementData.Src,
            this->managementData.Tag, this->megamolComm, &this->managementData.Status);
        this->mpiLock.unlock();
    }

    // Reset the management data.
    this->managementData.Completed = 0;
    this->managementData.Data = 0;

    // Wait for all nodes.
    megamol::core::utility::log::Log::DefaultLog.WriteInfo(
        "Node %d stoped everything, waiting for the other nodes.", this->worldRank);
    this->GlobalBarrier();
}


/*
 * megamol::network::MpiIntraCommunicator::waitForResponses
 */
void megamol::network::MpiIntraCommunicator::waitForResponses(const size_t idx) {
    #ifdef WITH_MPI
    // Wait for the operation to complete until the programme terminates.
    while (this->isRunning.load()) {
        // Wait for a send to occur.
        this->sendOperations[idx].waitEvent->Wait();

        // Complete the send operation for all nodes.
        if (this->sendOperations[idx].inUse && this->providingStreams[static_cast<int>(this->sendOperations[idx].type)]) {
            // Check if every destination node has recieved the send operation.
            int completionCnt = 0;
            for (size_t i = 0; i < this->sendOperations[idx].destNodes.size(); ++i) {
                if (this->sendOperations[idx].nodeCompleted[i] != 1) {
                    // Wait for the send operation to finish.
                    while (this->sendOperations[idx].nodeCompleted[i] != 1) {
                        // Call MPI_Test in order to check if the operation has completed.
                        this->mpiLock.lock();
                        auto res = ::MPI_Test(&this->sendOperations[idx].requests[i],
                            &this->sendOperations[idx].nodeCompleted[i], &this->sendOperations[idx].responseStatus[i]);
                        CHECK_MPI_LOG_AND_THROW(res, "MPI_Test failed with error \"%s\".", this->getMPIError(res));
                        this->mpiLock.unlock();

                        // Check if the programme already terminated.
                        if (!this->isRunning.load()) {
                            return;
                        }
                    }

                    // Post the receive operation.
                    {
                        this->mpiLock.lock();
                        this->sendOperations[idx].receivedFrameID = -1;
                        auto res = ::MPI_Irecv(&this->sendOperations[idx].receivedFrameID, 1, MPI_INT,
                            this->sendOperations[idx].destNodes[i], this->sendOperations[idx].ID, this->megamolComm,
                            &this->sendOperations[idx].responseRequests[i]);
                        CHECK_MPI_LOG_AND_THROW(res, "MPI_Irecv failed with error \"%s\".", this->getMPIError(res));
                        this->mpiLock.unlock();
                    }

                    // Wait for the receive operation to finish.
                    this->sendOperations[idx].responseFlag = 0;
                    while (this->sendOperations[idx].responseFlag == 0) {
                        // Call MPI_Test in order to check if the operation has completed.
                        this->mpiLock.lock();
                        auto res = ::MPI_Test(&this->sendOperations[idx].responseRequests[i],
                            &this->sendOperations[idx].responseFlag, &this->sendOperations[idx].responseStatus[i]);
                        CHECK_MPI_LOG_AND_THROW(res, "MPI_Test failed with error \"%s\".", this->getMPIError(res));
                        this->mpiLock.unlock();

                        // Check if the programme already terminated.
                        if (!this->isRunning.load()) {
                            return;
                        }
                    }

                    // Check if the ID is the one we expected.
                    if (std::abs(this->sendOperations[idx].receivedFrameID) != this->sendOperations[idx].frameID) {
                        megamol::core::utility::log::Log::DefaultLog.WriteError(
                            "Received frame ID %d but expected frame ID %d from node %d.",
                            this->sendOperations[idx].receivedFrameID, this->sendOperations[idx].frameID,
                            this->sendOperations[idx].destNodes[i]);
                    }
                }
                completionCnt += this->sendOperations[idx].nodeCompleted[i];
            }

            // Check if the operation is completed.
            auto isCompleted = static_cast<size_t>(completionCnt) == this->sendOperations[idx].destNodes.size();
            if (isCompleted) {
                // Reset the operation and the used send buffer.
                this->sendOperations[idx].frameID =
                    MpiIntraCommunicator::increaseOperationFrameID(this->sendOperations[idx].frameID);
                this->sendOperations[idx].inUse = false;
                std::fill(
                    this->sendOperations[idx].nodeCompleted.begin(), this->sendOperations[idx].nodeCompleted.end(), 0);
                this->sendCount[this->sendOperations[idx].sendBufferIdx][this->sendOperations[idx].sendIdx] = 0;
            }
        }
    }
    #endif
}
