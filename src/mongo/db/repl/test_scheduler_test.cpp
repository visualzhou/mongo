
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

struct Choice {
    std::vector<stdx::thread::id> threads;
    size_t currentIndex = 0;
    size_t actionIndex;

    bool isMyTurn() {
        return threads[currentIndex] == stdx::this_thread::get_id();
    }

    std::string toString() const {
        std::stringstream stream;
        stream << "Action " << actionIndex << " : [";
        for (auto t : threads)
            stream << t << "  ";
        stream << "]";
        stream << " current: " << currentIndex;
        return stream.str();
    }
};

// TODO:: each mutex has its own scheduler.
class Scheduler {
public:
    // Return true if this is the last thread hitting the barrier.
    bool attemptToTriggerBarrier() {
        _threadsWaiting--;
        if (_threadsWaiting != 0)
            return false;

        _generation++;
        _threadsWaiting = threads.size();
        if (!threads.empty()) {
            // Always choose the first one.
            behavior.push_back({threads, 0, behavior.size()});
            logd("behavior: {}", behavior.back());
        }
        _condition.notify_all();
        return true;
    }

    void wait(const Identity& id) {
        // For threads before the test runs.
        if (threads.empty())
            return;

        std::stringstream stream;
        stream << "0x" << std::hex << stdx::this_thread::get_id();
        auto me = stream.str();

        stdx::unique_lock<stdx::mutex> lk(mutex);
        bool myTurn = false;
        while (!myTurn) {
            logd("{} : Waiting for scheduler on {}, action: {}, count: {}, waiting: {}",
                 me,
                 id.name(),
                 behavior.size(),
                 threads.size(),
                 _threadsWaiting);

            if (!attemptToTriggerBarrier()) {
                uint64_t currentGeneration = _generation;
                while (currentGeneration == _generation) {
                    _condition.wait(lk);
                }
            }
            myTurn = behavior.back().isMyTurn();
        }
    }

    void onCreateThread() {
        stdx::lock_guard lk(mutex);
        threads.push_back(stdx::this_thread::get_id());
        _threadsWaiting++;
    }

    void onExitThread() {
        stdx::lock_guard lk(mutex);
        threads.erase(std::remove(threads.begin(), threads.end(), stdx::this_thread::get_id()),
                      threads.end());
        attemptToTriggerBarrier();
    }

    // TODO: we assume the new thread will not exit without waiting on the mutex so it won't
    // immediately exit.
    void waitForNewThread(stdx::thread::id newThread) {
        while (true) {
            std::lock_guard lk(mutex);
            if (std::find(threads.begin(), threads.end(), newThread) != threads.end())
                break;
        }
    }

    std::mutex mutex;
    // Records the thread index in threads scheduled so far.
    std::vector<Choice> behavior;
    std::vector<stdx::thread::id> threads;

    // Barrier
    size_t _threadsWaiting;
    uint64_t _generation = 0;
    stdx::condition_variable _condition;
} scheduler;

class LockListener : public latch_detail::DiagnosticListener {
    void aboutToLock(const Identity& id) override {
        logd("about to lock {}", id.name());
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
    // auto& state = latch_detail::getDiagnosticListenerState();
    // logd("XXX state: {}", state.listeners.size());

    Mutex mutex = MONGO_MAKE_LATCH("test_mutex");

    scheduler.onCreateThread();

    auto t1 = stdx::thread([&] {
        scheduler.onCreateThread();
        {
            stdx::lock_guard lk(mutex);
            logd("XXX 1");
        }
        {
            stdx::lock_guard lk(mutex);
            logd("XXX 3");
        }
        scheduler.onExitThread();
    });
    scheduler.waitForNewThread(t1.get_id());

    mutex.lock();
    logd("XXX 2");
    mutex.unlock();
    scheduler.onExitThread();

    t1.join();

    // Check the post condition of the interleaving.
    ASSERT(true);
}

}  // namespace
}  // namespace repl
}  // namespace mongo