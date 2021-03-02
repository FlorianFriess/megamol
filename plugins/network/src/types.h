/*
 * types.h
 *
 * Author: Florian Frieß
 * Copyright (C) 2021 by Universitaet Stuttgart (VISUS).
 * All rights reserved.
 */

#ifndef NETWORK_TYPES_H_INCLUDED
#define NETWORK_TYPES_H_INCLUDED
#if (defined(_MSC_VER) && (_MSC_VER > 1000))
#pragma once
#endif /* (defined(_MSC_VER) && (_MSC_VER > 1000)) */

#include "vislib/sys/Event.h"

#include <thread>
#include <vector>

namespace megamol {
namespace network {

    /**
     * Numeric identifier of the data type. Used by the MpiIntraCommunicator.
     */
    #define INTRADATATYPECNT 4
    enum class IntraDataType {
        BITSTREAM = 0,
        BITSTREAMENC = 1,
        CAMERA = 2,
        FOVEATEDREGION = 3
    };

    /**
     * Identifies different roles a machine can have. Used by the MpiIntraCommunicator.
     */
    enum class Role {
        /** The nodes does not take part in any intra-communication. */
        NONE = 0x0000,

        /** The node receives data, such as bitstreams, encoded bitstreams or foveated regions. */
        DATA_RECEIVER = 0x0001,

        /** The node provides data, such as bitstreams, encoded bitstreams or foveated regions. */
        DATA_PROVIDER = 0x0002
    };

    /**
     * The struct that contains the MPI management data.
     */
    struct ManagementData {
        /** Ctor */
        ManagementData(void) : Completed(0), Data(0), Dst(0), Request(0), Src(0), Tag(0) {}

        int Completed;

        /** The state that is send/received. */
        int Data;

        /** The rank of the node that we inform about changes. */
        int Dst;

        /** Store the current MPI_Request for the last MPI_Irecv call. */
        #ifdef WITH_MPI
        MPI_Request Request;
        #else
        int Request;
        #endif

        /** The rank of the node that informs us about changes. */
        int Src;

        #ifdef WITH_MPI
        MPI_Status Status;
        #else
        int Status;
        #endif

        /** The tag used by MPI for the management communication. */
        int Tag;
    };

    /**
     * The header for the data that is transmitted via MPI.
     */
    struct Message {
        /** Ctor */
        Message(void) : DataCnt(0), DataID(0), Height(0), Timestamp(0), Width(0) {}

        /** Ctor */
        Message(const size_t DataCnt, const size_t DataID, const int Height, const int64_t Timestamp, const int Width)
                : DataCnt(DataCnt), DataID(DataID), Height(Height), Timestamp(Timestamp), Width(Width) {}

        /** The amount of data in the messagen. */
        size_t DataCnt;

        /** The unique ID of the data. */
        size_t DataID;

        /** The height of a frame. */
        int Height;

        /** The timestamp of the data. */
        int64_t Timestamp;

        /** The width of a frame. */
        int Width;
    };

    /**
     * The struct that contains the data needed for the MPI_Test function, that checks if the MPI_Isend operation
     * has completed.
     */
    struct MPISendData {
        /** The amount of data that is sent. */
        size_t dataCnt;

        /** The rank of the destination nodes. */
        std::vector<int> destNodes;

        /** The ID, i.e. the number, of the last send frame. */
        int frameID;

        /** The ID of the send operation. */
        int ID;

        /**
         * Flag that signals if the send is complete.
         *
         * @remarks If the flag is 'true' the MPI_Isend is not completed. If the flag is 'false' the send is done.
         */
        bool inUse;

        /**
         * The flag, for each node in destNodes, that is 1 once the send operation to the node has been
         * completed.
         */
        std::vector<int> nodeCompleted;

        /** Returns the frame ID that the destination node responded with. */
        int receivedFrameID;

        /**
         * Store the current MPI_Requests that were used for the last MPI_Isend operations of that stream type.
         */
        #ifdef WITH_MPI
        std::vector<MPI_Request> requests;
        #else
        std::vector<int> requests;
        #endif

        /** The number of expected responses after the send operation has been completed. */
        int responseCnt;

        /** Used to test if the response from the destination nodes has been fully received. */
        int responseFlag;

        /**
         * The received frame IDs after the send operation has been completed.
         */
        std::vector<int> responseFrameID;

        /** The asynchronous requests issued after the send operation has been completed. */
        #ifdef WITH_MPI
        std::vector<MPI_Request> responseRequests;
        #else
        std::vector<int> responseRequests;
        #endif

        /** The status of the asynchronous requests issued after the send operation has been completed. */
        #ifdef WITH_MPI
        std::vector<MPI_Status> responseStatus;
        #else
        std::vector<int> responseStatus;
        #endif

        /** The index of the sendBuffer that holds the data for this operation. */
        size_t sendBufferIdx;

        /** The index that was used for these send operations. */
        size_t sendIdx;

        /** Store the MPI_Status of the MPI_Test function. */
        #ifdef WITH_MPI
        std::vector<MPI_Status> status;
        #else
        std::vector<int> status;
        #endif

        /** Store the type of the stream these MPI_Isend operations belong to. */
        IntraDataType type;

        /** The unique ID of the data that is transmitted. */
        int uniqueID;

        /**
         * The event that is signaled after the asynchronous send operation has started.
         *
         * @remarks The waitThread waits for the event to be signaled.
         */
        std::shared_ptr<vislib::sys::Event> waitEvent;

        /**
         * The thread that makes sure the send operation is completed and the responses from all destination
         * nodes has been received.
         */
        std::thread waitThread;
    };

    /// <summary>
    /// The struct that contains the data needed for the MPI_Iprobe function,
    /// that checks if the MPI_Recv can be called.
    /// </summary>
    struct MPIRecvData {
        /**
         * Contais the answer of this node send to the source node after the data for this operation has been
         * received.
         */
        int answer;

        /** The completed flags for the MPI_Iprobe operation. */
        std::vector<int> completed;

        /** The ID, i.e. the number, of the last received frame. */
        int frameID;

        /** Contains the size of the data that can be received by this node. */
        int frameSize;

        /** The ID of the receive operation. */
        int ID;

        /**
         * Flag that signals if the receive is complete.
         *
         * @remarks If the flag is 'true' the MPI_Iprobe is not completed. If the flag is 'false' the data can be
         * recieved.
         */
        bool inUse;

        /** The rank of the source nodes. */
        std::vector<int> srcNodes;

        /** Store the MPI_Status of the MPI_Test function. */
        #ifdef WITH_MPI
        MPI_Status status;
        #else
        int status;
        #endif

        /** Store the type of the stream these MPI_Iprobe operations belong to. */
        IntraDataType type;

        /** The unique ID of the data that is transmitted. */
        int uniqueID;
    };

} // namespace network
} /* end namespace megamol */

#endif /* NETWORK_TYPES_H_INCLUDED */
