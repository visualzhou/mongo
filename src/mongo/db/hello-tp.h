#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER mongo

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "mongo/db/hello-tp.h"

#if !defined(_HELLO_TP_H) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _HELLO_TP_H

#include <lttng/tracepoint.h>

TRACEPOINT_EVENT_CLASS(
    mongo,
    txnClass,
    TP_ARGS(
        unsigned int, lsidArg,
        int, txnNumArg,
        const char*, stateArg
    ),
    TP_FIELDS(
        ctf_integer(unsigned int, lsid, lsidArg)
        ctf_integer(int, txnNum, txnNumArg)
        ctf_string(state, stateArg)
    )
)

#define DEFINE_TXN_INSTANCE(name) TRACEPOINT_EVENT_INSTANCE(mongo, txnClass, txn, \
    TP_ARGS(unsigned int, lsidArg, int, txnNumArg, const char*, stateArg))

TRACEPOINT_EVENT_CLASS(mongo, simpleEvent, TP_ARGS(), TP_FIELDS())
TRACEPOINT_EVENT_INSTANCE(mongo, simpleEvent, before_schedule_write_to_oplog, TP_ARGS())
TRACEPOINT_EVENT_INSTANCE(mongo, simpleEvent, after_schedule_write_to_oplog, TP_ARGS())
TRACEPOINT_EVENT_INSTANCE(mongo, simpleEvent, after_dispatch_writes, TP_ARGS())
TRACEPOINT_EVENT_INSTANCE(mongo, simpleEvent, after_oplog_write, TP_ARGS())
TRACEPOINT_EVENT_INSTANCE(mongo, simpleEvent, after_write_consistency_markers, TP_ARGS())
TRACEPOINT_EVENT_INSTANCE(mongo, simpleEvent, after_oplog_application, TP_ARGS())
DEFINE_TXN_INSTANCE(txn)

#endif /* _HELLO_TP_H */

#include <lttng/tracepoint-event.h>

