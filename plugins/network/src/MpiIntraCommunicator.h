/*
 * MpiIntraCommunicator.h
 *
 * Author: Florian Frieß
 * Copyright (C) 2021 by Universitaet Stuttgart (VISUS).
 * All rights reserved.
 */

#ifndef NETWORK_MPIINTRACOMMUNICATOR_H_INCLUDED
#define NETWORK_MPIINTRACOMMUNICATOR_H_INCLUDED
#if (defined(_MSC_VER) && (_MSC_VER > 1000))
#pragma once
#endif /* (defined(_MSC_VER) && (_MSC_VER > 1000)) */

#include "mmcore/CalleeSlot.h"
#include "mmcore/CallerSlot.h"
#include "mmcore/Module.h"
#include "mmcore/cluster/mpi/MpiCall.h"
#include "mmcore/utility/log/Log.h"
#include "mmcore/utility/sys/SystemInformation.h"

#include <mutex>

#ifdef WITH_MPI
#include "mpi.h"
#endif

#include "MpiIntraCall.h"
#include "MPIRecvBuffer.h"
#include "types.h"

namespace megamol {
namespace network {

    /**
     * This module allows to communicate across machines via MPI.
     */
    class MpiIntraCommunicator : public megamol::core::Module {
    public:
        /**
         * Answer the name of this module.
         *
         * @return The name of this module.
         */
        static const char* ClassName(void) {
            return "MpiIntraCommunicator";
        }

        /**
         * Answer a human readable description of this module.
         *
         * @return A human readable description of this module.
         */
        static const char* Description(void) {
            return "MPI communicator to send and receive data from other nodes.";
        }

        /**
         * Wait for an execution barrier on all nodes in the MPI world.
         */
        void GlobalBarrier(void) const;

        /**
         * Initialise MPI, the role of the node and get the roles of all other nodes in the MPI world.
         */
        bool Initialise(const Role ownRole);

        /**
         * Answers whether this module is available on the current system.
         *
         * @return 'true' if the module is available, 'false' otherwise.
         */
        static bool IsAvailable(void) {
            return true;
        }

        /** Dtor */
        virtual ~MpiIntraCommunicator(void);

        /** Ctor */
        MpiIntraCommunicator(void);

        /**
         * Remeber that the current nodes expects data of the given type.
         *
         * @param type The type of the data.
         * @param dataID The unique ID of the expected data.
         */
        void SetExpectedStreamType(const IntraDataType type, const size_t dataID);

        /**
         * Remeber that the current nodes provides data of the given type.
         *
         * @param type The type of the data.
         * @param dataID The unique ID of the provided data.
         */
        void SetProvidedStreamType(const IntraDataType type, const size_t dataID);

        /**
         * Start the thread that handles the communication between nodes by calling doDataExchange until the
         * programme terminates.
         */
        void Start(void);

        /**
         * Stop the thread that handles the communication between nodes.
         */
        void Stop(void);

    protected:
        /**
         * Implementation of 'Create'.
         *
         * @return 'true' on success, 'false' otherwise.
         */
        virtual bool create(void);

        /**
         * Implementation of 'Release'.
         */
        virtual void release(void);

    private:
        /**
          * Checks whether status represents an MPI error and if so, logs the given error message and throws an
          * exception.
          */
        #define CHECK_MPI_LOG_AND_THROW(status, fmt, ...)                                  \
            if (status != MPI_SUCCESS) {                                                   \
                megamol::core::utility::log::Log::DefaultLog.WriteError(fmt, __VA_ARGS__); \
                throw std::exception(this->getMPIError(status).c_str());                   \
            }

        /** Typedef for the combination of data type and unique ID. */
        typedef std::pair<IntraDataType, size_t> StreamData;

        /**
         * Collect all expected and provided streams across the MPI world. Then initialise the send and received
         * operations for the node based on the collected information.
         *
         * @param streams Reference to either the streams the node provides or receives, depending on the
         * operationType.
         * @param streamCnt The maximum number of streams provided/expected by a node in the MPI world.
         * @param operationType Flag to switch between initialisen either send (1) or receive(0) operations.
         *
         * @returns 'true' in case the node provides/expects streams, depending on the operationType. 'false'
         * otherwise.
         */
        bool collectStreams(const std::vector<StreamData>& streams, const int streamCnt, const int operationType);

        /**
         * Used by the communication thread.
         */
        void communicate(void);

        /**
         * Performs all intra-communication queued for the current machine. This means
         * sending an received data.
         *
         * @param runAgain After the first call to this function the value will be either 'true' or 'false'. In
         * consecutive calls the value will not be changed. If the value is 'false' there is no need to call this
         * function again, if it is 'true' it needs to be called untill the programme is terminated.
         */
        void doDataExchange(bool& runAgain);

        /**
         * Get the description of the MPI error based on the error code.
         *
         * @param errorCode The error code returned by an MPI function.
         *
         * @returns The string describing the error.
         */
        std::string getMPIError(const int errorCode) const;

        /**
         * Compute a unique ID based on the input.
         *
         * @param streamType The IntraDataType.
         * @param worldRank The MPI rank of the node in the MPI world.
         *
         * @return A unique ID based on the two input values.
         */
        int getUniqueID(const int streamType, const int worldRank);

        /**
         * Take the old frame ID and increase it by one while making sure an overflow does not happen.
         *
         * @param oldFrameID The current frame ID of the operation.
         *
         * @returns The new frame ID for the operation.
         */
        static int increaseOperationFrameID(const int oldFrameID);

        /**
         * Initialise the send operations of the current node based on the given stream types and names.
         *
         * @param streamTypes The stream types of the expected streams of all nodes in the MPI world.
         * @param streamTypeNames The string representation of the four different stream types.
         *
         * @returns 'true' in case the node provides streams, 'false' otherwise.
         */
        bool initialiseExpectedOperations(const std::vector<int>& streamTypes, const std::vector<int>& streamIDs,
            const std::vector<std::string>& streamTypeNames);

        /**
         * Initialise the receive operations of the current node based on the given stream types and names.
         *
         * @param streamTypes The stream types of the provided streams of all nodes in the MPI world.
         * @param streamIDs The stream IDs of the provided streams of all nodes in the MPI world.
         * @param streamTypeNames The string representation of the four different stream types.
         *
         * @returns 'true' in case the node expects streams, 'false' otherwise.
         */
        bool initialiseReceiveOperations(const std::vector<int>& streamTypes, const std::vector<int>& streamIDs,
            const std::vector<std::string>& streamTypeNames);

        /**
         * Initialise MPI using the MpiProvider.
         *
         * @returns 'true' if successfull, 'false' otherwise.
         */
        bool initMPI(void);

        /**
         * Called in order to add the data type to the list of expected data.
         *
         * @param call Contains the type of data that the node expects.
         *
         * @returns 'true'.
         */
        bool onAddExpectedData(megamol::core::Call& call);

        /**
         * Called in order to add the data type to the list of provided data.
         *
         * @param call Contains the type of data that the node provides.
         *
         * @returns 'true'.
         */
        bool onAddProvidedData(megamol::core::Call& call);

        /**
         * Called in order to get the latest received data.
         *
         * @param call The call that will contain the received data. Contains the type of data that should be
         * delived.
         *
         * @returns 'true' in case there was data, 'false' in case of an error.
         */
        bool onDeliverData(megamol::core::Call& call);

        /**
         * Called in order to initialise the MpiIntraCommunicator.
         *
         * @param call Contains the roles of the node.
         *
         * @returns 'true' in case the MpiIntraCommunicator was initialisef, 'false' otherwise.
         */
        bool onInitialise(megamol::core::Call& call);

        /**
         * Called in order to send data.
         *
         * @param call The call that contains the data.
         *
         * @returns 'true'
         */
        bool onSendData(megamol::core::Call& call);

        /**
         * Called in order to start the MpiIntraCommunicator.
         *
         * @param call Can be ignored.
         *
         * @returns 'true'
         */
        bool onStart(megamol::core::Call& call);

        /**
         * Called in order to stop the MpiIntraCommunicator.
         *
         * @param call Can be ignored.
         *
         * @returns 'true'
         */
        bool onStop(megamol::core::Call& call);

        /**
         * Reset the send and receive operations.
         */
        void reset(void);

        /**
         * Adds the given data to the send buffer which will be send to all nodes that expect the data in the
         * next call to doDataExchange.
         *
         * @param data Pointer to the start of the data.
         * @param cntData The number of bytes.
         * @param dataID The uniqe ID of the data.
         * @param timestamp The timestamp of the frame.
         * @param height The height in pixel. Only required for frame data.
         * @param width The width in pixel. Only required for frame data.
         */
        void sendData(const void* data, const size_t cntData, const size_t dataID, const int64_t timestamp,
            const int height, const int width);

        /**
         * Used by each send operation in sendOperations to make sure that all destination nodes have received
         * the data and responded.
         */
        void waitForResponses(const size_t idx);

        /** The data used by MPI for the management communication. */
        ManagementData managementData;

        /** Call to send and receive data via MPI. */
        megamol::core::CalleeSlot mpiIntraCall;

        /** The thread that handles the MPI-based communication between the nodes. */
        std::thread communicationThread;

        std::map<IntraDataType, std::shared_ptr<vislib::sys::Event>> deliverEvents;

        /** The vector that will contain data that could not be received because the receiveBuffer was full. */
        std::vector<uint8_t> droppData;

        /** The unique ID of the data that is expected on the current node. */
        std::vector<StreamData> expectedStreams;

        /** Store, for the current node, that it expects data of the type. */
        std::vector<bool> expectingStreams;

        /** Flag that signals that MPI needs to be initialised. */
        bool initialiseMPI;

        /**
         * The state if the providing and expecting nodes, i.e. the state of DoDataExchange function. Is 'true'
         * after the DoDataExchange function was called once, 'false' until then.
         */
        bool isInitialised;

        /** Controls the lifetime of the deliverThread thread. */
        std::atomic<bool> isRunning;

        /** The MPI world communicator containing all nodes. */
#ifdef WITH_MPI
        MPI_Comm megamolComm;
#else
        int megamolComm;
#endif

        /**
         * The lock that protects the access to all MPI functions, since they might be called from different
         * threads.
         */
        mutable std::mutex mpiLock;

        /** The index of the current buffer for each data type that is send next. */
        std::vector<size_t> nextSendBuffer;

        /** The unique ID of the data that is provided by the current node. */
        std::vector<StreamData> providedStreams;

        /** Store, for the current node, that it provides data of the type. */
        std::vector<bool> providingStreams;

        /** The ring buffer the stores the received data before it is delivered. */
        std::vector<std::shared_ptr<MPIRecvBuffer>> receiveBuffer;

        /** The underlying container of the receiveBuffer ring buffer. */
        std::vector<std::vector<RecvData>> receiveBufferData;

        /** Maps the node rank of the source to the receiveBuffer ring buffer. */
        std::map<int, size_t> receiveBufferMap;

        /** Maps the data type of the operation to the receiveBuffer. */
        std::multimap<IntraDataType, size_t> receiveBufferDeliverMap;

        /** Contains the data for all asynchronous MPI_Iprobe operations. */
        std::vector<MPIRecvData> receiveOperations;

        /** The slot used to call the MpiProvider in order to initialise MPI. */
        megamol::core::CallerSlot requestMpi;

        /** The roles of this node. */
        std::underlying_type<Role>::type roles;

        /**
         * Accumulates all data that are to be sent, for each data type, to the nodes that expect it during the
         * next data exchange.
         */
        std::vector<std::vector<std::vector<BYTE>>> sendBuffer;

        /** The map that links the index of the sendBuffer to the ID of the data. */
        std::map<size_t, size_t> sendBufferMap;

        /** The amount of valid data that are to be sent from the sendBuffer. */
        std::vector<std::vector<size_t>> sendCount;

        /** The lock protecting access to the sendBuffer and the sendCount. */
        std::vector<std::shared_ptr<std::mutex>> sendLocks;

        /**
         * The flag indicates if there is only one node in the MPI world and DoDataExchange will not be called
         * again.
         */
        bool singleNode;

        /** The MPI rank of the nodes that provide data. */
        std::vector<int> sourceNodes;

        /** Contains the data for all asynchronous MPI_Isend operations. */
        std::vector<MPISendData> sendOperations;

        /** The MPI rank of the nodes that receive data. */
        std::vector<int> targetNodes;

        /** Rank of this node in the megamolComm world. */
        int worldRank;

        /** Size of the megamolComm world. */
        int worldSize;
    };

} // namespace megamol
} /* end namespace megamol */

#endif /* NETWORK_MPIINTRACOMMUNICATOR_H_INCLUDED */
