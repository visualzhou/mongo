
#include "mongo/stdx/thread.h"
#include "mongo/unittest/barrier.h"
#define MONGO_LOGV2_DEFAULT_COMPONENT ::mongo::logv2::LogComponent::kDefault

#include "mongo/platform/basic.h"

#include "mongo/logv2/log.h"
#include "mongo/unittest/log_test.h"
#include "mongo/unittest/unittest.h"

namespace mongo {
namespace repl {

using latch_detail::Identity;
using unittest::Barrier;

class Scheduler {
public:
    void createBarrier(int numThread) {
        barrier = std::make_unique<Barrier>(numThread);
    }

    void wait(const Identity& id) {
        if (barrier) {
            std::stringstream stream;
            stream << stdx::this_thread::get_id();
            logd("Waiting for scheduler on {} by {}", id.name(), stream.str());
            barrier->countDownAndWait();
        }
    }
    std::unique_ptr<Barrier> barrier;
} scheduler;

class LockListener : public latch_detail::DiagnosticListener {
    void aboutToLock(const Identity& id) override {
        logd("XXX about to lock {}", id.name());
        scheduler.wait(id);
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

    scheduler.createBarrier(2);

    Mutex mutex = MONGO_MAKE_LATCH("test_mutex");


    auto t1 = stdx::thread([&] {
        stdx::lock_guard lk(mutex);
        logd("XXX 1");
    });


    mutex.lock();
    logd("XXX 2");

    mutex.unlock();

    t1.join();
    ASSERT(true);
}

}  // namespace
}  // namespace repl
}  // namespace mongo