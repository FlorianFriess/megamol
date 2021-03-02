#include "MpiIntraCall.h"


/*
 * megamol::network::MpiIntraCall::IDX_DELIVER_DATA
 */
const unsigned int megamol::network::MpiIntraCall::IDX_DELIVER_DATA = 0;


/*
 * megamol::network::MpiIntraCall::IDX_SEND_DATA
 */
const unsigned int megamol::network::MpiIntraCall::IDX_SEND_DATA = 1;


/*
 * megamol::network::MpiIntraCall::IDX_INIT_COMM
 */
const unsigned int megamol::network::MpiIntraCall::IDX_INIT_COMM = 2;


/*
 * megamol::network::MpiIntraCall::IDX_ADD_PROVIDED_DATA
 */
const unsigned int megamol::network::MpiIntraCall::IDX_ADD_PROVIDED_DATA = 3;


/*
 * megamol::network::MpiIntraCall::IDX_ADD_EXPECTED_DATA
 */
const unsigned int megamol::network::MpiIntraCall::IDX_ADD_EXPECTED_DATA = 4;


/*
 * megamol::network::MpiIntraCall::IDX_START_COMM
 */
const unsigned int megamol::network::MpiIntraCall::IDX_START_COMM = 5;


/*
 * megamol::network::MpiIntraCall::IDX_STOP_COMM
 */
const unsigned int megamol::network::MpiIntraCall::IDX_STOP_COMM = 6;


/*
 * megamol::network::MpiIntraCall:INTENTS
 */
const char* megamol::network::MpiIntraCall::INTENTS[] = {
    "DeliverData", "SendData", "InitComm", "AddProvidedData", "AddExpectedData", "StartComm", "StopComm"};


/*
 * megamol::network::MpiIntraCall::~MpiIntraCall
 */
megamol::network::MpiIntraCall::~MpiIntraCall(void) {}


/*
 * megamol::network::MpiIntraCall::FunctionCount
 */
unsigned int megamol::network::MpiIntraCall::FunctionCount(void) {
    return (sizeof(MpiIntraCall::INTENTS) / sizeof(*MpiIntraCall::INTENTS));
}


/*
 * megamol::network::MpiIntraCall::FunctionName
 */
const char* megamol::network::MpiIntraCall::FunctionName(unsigned int idx) {
    if (idx < MpiIntraCall::FunctionCount()) {
        return MpiIntraCall::INTENTS[idx];
    } else {
        return "";
    }
}


/*
 * megamol::network::MpiIntraCall::IsAvailable
 */
bool megamol::network::MpiIntraCall::IsAvailable(void) {
    return true;
}


/*
 * megamol::network::MpiIntraCall::MpiIntraCall
 */
megamol::network::MpiIntraCall::MpiIntraCall(void)
        : megamol::core::Call()
        , data(nullptr)
        , dataCnt(0)
        , dataID(0)
        , dataType()
        , deliverData()
        , deliverDataCnt(0)
        , height(0)
        , roles(Role::NONE)
        , timestamp(0)
        , width(0) {}
