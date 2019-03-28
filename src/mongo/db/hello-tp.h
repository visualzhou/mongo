#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER mongo

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "mongo/db/hello-tp.h"

#if !defined(_HELLO_TP_H) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _HELLO_TP_H

#include <lttng/tracepoint.h>

TRACEPOINT_EVENT(
    mongo,
    txn,
    TP_ARGS(
        int, txnNumArg,
        const char*, stateArg
    ),
    TP_FIELDS(
        ctf_string(state, stateArg)
        ctf_integer(int, txnNum, txnNumArg)
    )
)

#endif /* _HELLO_TP_H */

#include <lttng/tracepoint-event.h>

