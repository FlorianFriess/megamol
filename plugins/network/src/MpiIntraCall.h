/*
 * MpiIntraCall.h
 *
 * Author: Florian Frieß
 * Copyright (C) 2021 Visualisierungsinstitut der Universität Stuttgart.
 * Alle Rechte vorbehalten.
 */

#ifndef NETWORK_MPIINTRACALL_H_INCLUDED
#define NETWORK_MPIINTRACALL_H_INCLUDED
#if (defined(_MSC_VER) && (_MSC_VER > 1000))
#pragma once
#endif /* (defined(_MSC_VER) && (_MSC_VER > 1000)) */

#include "mmcore/Call.h"
#include "mmcore/factories/CallAutoDescription.h"

#ifdef WITH_MPI
#include "mpi.h"
#endif

#include "types.h"

namespace megamol {
namespace network {

    /**
     * Class for binding site calls and data interfaces.
     */
    class MpiIntraCall : public megamol::core::Call {
    public:
        /**
         * Add the pointer to the deliver data at the given index.
         *
         * @param data The pointer to the data.
         * @param dataCnt The number of bytes in the data.
         * @param idx The index where to put the data pointer.
         */
        inline void AddDeliverData(void* data, const size_t dataCnt, const size_t idx) {
            // Check if the vector is big enough.
            if (this->deliverData.size() < (idx + 1)) {
                this->deliverData.resize(idx + 1);
            }

            // Add the pointer and the number of bytes to the vector.
            this->deliverData[idx].first = data;
            this->deliverData[idx].second = dataCnt;
        }

        /**
         * Answer the name of this call.
         *
         * @return The name of this call.
         */
        static inline const char* ClassName(void) {
            return "MpiIntraCall";
        }

        /**
         * Generate a unique ID based on the two input values a and b. This function must match the
         * implementation of the megamol::network::MpiIntraCommunicator::getUniqueID function.
         *
         * @param a The first value.
         * @param b The second value.
         *
         * @returns The unique value based on the two input values.
         */
        static inline int ComputeUniqueID(const int a, const int b) {
            return (((a + b) * (a + b + 1)) / 2) + b;
        }

        /**
         * Answer a human readable description of this call.
         *
         * @return A human readable description of this call.
         */
        static inline const char* Description(void) {
            return "Call to send and receive data via MPI.";
        }

        /**
         * Answer the number of functions used for this call.
         *
         * @return The number of functions used for this call.
         */
        static unsigned int FunctionCount(void);

        /**
         * Answer the name of the function used for this call.
         *
         * @param idx The index of the function to return it's name.
         *
         * @return The name of the requested function.
         */
        static const char* FunctionName(unsigned int idx);

        /**
         * Get the pointer to the data.
         */
        inline void* GetData(void) {
            return this->data;
        }

        /**
         * Get the number of bytes of data.
         */
        inline size_t GetDataCnt(void) {
            return this->dataCnt;
        }

        /**
         * Get the unique ID of the data.
         */
        inline size_t GetDataID(void) {
            return this->dataID;
        }

        /**
         * Get the type of the data.
         */
        inline IntraDataType GetDataType(void) {
            return this->dataType;
        }

        /**
         * Get the deliver data vector.
         */
        inline std::vector<std::pair<void*, size_t>>& GetDeliverData(void) {
            return this->deliverData;
        }

        /**
         * Get the number of valid pointers in the deliver data vector.
         */
        inline size_t GetDeliverDataCnt(void) {
            return this->deliverDataCnt;
        }

        /**
         * Get the height of a frame.
         */
        inline int GetHeight(void) {
            return this->height;
        }

        /**
         * Get the role(s) of the data.
         */
        inline Role GetRoles(void) {
            return this->roles;
        }

        /**
         * Get the timestamp of the data.
         */
        inline int64_t GetTimestamp(void) {
            return this->timestamp;
        }

        /**
         * Get the width of a frame.
         */
        inline int GetWidth(void) {
            return this->width;
        }

        /**
         * Answers whether this call is available on the current system.
         *
         * @return 'true' if the module is available, 'false' otherwise.
         */
        static bool IsAvailable(void);

        /**
         * Initialises a new instance.
         */
        MpiIntraCall(void);

        /**
         * Finalises the instance.
         */
        virtual ~MpiIntraCall(void);

        /**
         * Set the pointer to the data.
         */
        inline void SetData(void* data) {
            this->data = data;
        }

        /**
         * Set the number of bytes.
         */
        inline void SetDataCnt(const size_t dataCnt) {
            this->dataCnt = dataCnt;
        }

        /**
         * Set the unique ID of the data.
         */
        inline void SetDataID(const size_t dataID) {
            this->dataID = dataID;
        }

        /**
         * Set the type of the data.
         */
        inline void SetDataType(const IntraDataType dataType) {
            this->dataType = dataType;
        }

        /**
         * Set the number of valid pointers in the deliver data vector.
         */
        inline void SetDeliverDataCnt(const size_t deliverDataCnt) {
            this->deliverDataCnt = deliverDataCnt;
        }

        /**
         * Set the height of a frame.
         */
        inline void SetHeight(const int height) {
            this->height = height;
        }

        /**
         * Set the role(s) of the node.
         */
        inline void SetRoles(const Role roles) {
            this->roles = roles;
        }

        /**
         * Set the timestamp of the data.
         */
        inline void SetTimestamp(const int64_t timestamp) {
            this->timestamp = timestamp;
        }

        /**
         * Set the width of a frame.
         */
        inline void SetWidth(const int width) {
            this->width = width;
        }

        /** Index of the 'DeliverData' function */
        static const unsigned int IDX_DELIVER_DATA;

        /** Index of the 'SendData' function */
        static const unsigned int IDX_SEND_DATA;

        /** Init of the 'Initialise' function. */
        static const unsigned int IDX_INIT_COMM;

        /** Init of the 'SetProvidedStreamType' function. */
        static const unsigned int IDX_ADD_PROVIDED_DATA;

        /** Init of the 'SetExpectedStreamType' function. */
        static const unsigned int IDX_ADD_EXPECTED_DATA;

        /** Init of the 'Start' function. */
        static const unsigned int IDX_START_COMM;

        /** Init of the 'Stop' function. */
        static const unsigned int IDX_STOP_COMM;

    private:

        /** Pointer to the data.*/
        void* data;

        /** The number of bytes. */
        size_t dataCnt;

        /** The unique ID of the data. */
        size_t dataID;

        /** The type of the data. */
        IntraDataType dataType;

        std::vector<std::pair<void*, size_t>> deliverData;

        size_t deliverDataCnt;

        /** The height of a frame. */
        int height;

        /** The roles of the node. */
        Role roles;

        /** The timestamp of the data. */
        int64_t timestamp;

        /** The width of a frame. */
        int width;

        /** The functions that are provided by the call. */
        static const char* INTENTS[7];
    };

    /** Description class typedef */
    typedef megamol::core::factories::CallAutoDescription<MpiIntraCall> MpiIntraCallDescription;

} // namespace network
} /* end namespace megamol */

#endif /* NETWORK_MPIINTRACALL_H_INCLUDED */
