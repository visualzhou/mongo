
#define MONGO_LOGV2_DEFAULT_COMPONENT ::mongo::logv2::LogComponent::kDefault

#include "mongo/platform/basic.h"

#include "mongo/logv2/log.h"
#include "mongo/unittest/log_test.h"
#include "mongo/unittest/unittest.h"

namespace mongo {
namespace repl {

class Scheduler {
} scheduler;

class LockListener : public latch_detail::DiagnosticListener {
    void aboutToLock(const Identity& id) override {
        logd("XXX about to lock {}", id.name());
    }

    void onContendedLock(const Identity& id) override{};
    void onQuickLock(const Identity& id) override{};
    void onSlowLock(const Identity& id) override{};
    void onUnlock(const Identity& id) override{};
};

namespace {

class Initialization {
public:
    Initialization() {
        latch_detail::installDiagnosticListener<LockListener>();
    }
} globalInitializationInstance;

TEST(Scheduler, Simple) {
    auto& state = latch_detail::getDiagnosticListenerState();
    logd("XXX state: {}", state.listeners.size());

    Mutex mutex = MONGO_MAKE_LATCH("test_mutex");

    mutex.lock();
    logd("XXX 1");

    mutex.unlock();
    ASSERT(true);
}

}  // namespace
}  // namespace repl
}  // namespace mongo