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

#define DEFINE_TXN_INSTANCE(name) TRACEPOINT_EVENT_INSTANCE(mongo, txnClass, name, \
    TP_ARGS(unsigned int, lsidArg, int, txnNumArg, const char*, stateArg))

TRACEPOINT_EVENT_CLASS(mongo, simpleEvent, TP_ARGS(), TP_FIELDS())
#define SIMPLE_EVENT(name) TRACEPOINT_EVENT_INSTANCE(mongo, simpleEvent, name, TP_ARGS())

TRACEPOINT_EVENT_CLASS(mongo, simpleNumberEvent, TP_ARGS(int, numArg), TP_FIELDS(ctf_integer(int, num, numArg)))
#define SIMPLE_NUM_EVENT(name) TRACEPOINT_EVENT_INSTANCE(mongo, simpleNumberEvent, name, TP_ARGS(int, numArg))

SIMPLE_EVENT(before_schedule_write_to_oplog)
SIMPLE_EVENT(after_schedule_write_to_oplog)
SIMPLE_EVENT(after_dispatch_writes)
SIMPLE_EVENT(after_oplog_write)
SIMPLE_EVENT(after_write_consistency_markers)
SIMPLE_EVENT(after_oplog_application)

// Txn application trace points.
SIMPLE_EVENT(start_commit_apply)
SIMPLE_EVENT(start_read_from_oplog_chain)
SIMPLE_EVENT(start_traverse_iterater)
SIMPLE_NUM_EVENT(start_reverse_oplog_from_disk)
SIMPLE_EVENT(start_build_cached_ops)
SIMPLE_EVENT(end_build_cached_ops)

DEFINE_TXN_INSTANCE(txn)

TRACEPOINT_EVENT(
    mongo,
    txn_apply,
    TP_ARGS(
        unsigned int, lsidArg,
        int, txnNumArg,
        unsigned int, countArg
    ),
    TP_FIELDS(
        ctf_integer(unsigned int, lsid, lsidArg)
        ctf_integer(int, txnNum, txnNumArg)
        ctf_integer(unsigned int, count, countArg)
    )
)

#endif /* _HELLO_TP_H */

#include <lttng/tracepoint-event.h>

