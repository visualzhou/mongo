
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

std::string toString(stdx::thread::id id) {
        std::stringstream stream;
        stream << id;
        return stream.str();
}

struct Choice {
    std::vector<stdx::thread::id> threads;
    size_t currentChoice = 0;
    size_t actionIndex;

    bool isMyTurn() const {
        return threads[currentChoice] == stdx::this_thread::get_id();
    }

    void next() {
        currentChoice++;
    }
    bool exhausted() const {
        return currentChoice == threads.size() - 1;
    }

    std::string toString() const {
        std::stringstream stream;
        stream << "Action " << actionIndex << " : [";
        for (auto t : threads)
            stream << t << "  ";
        stream << "]";
        stream << " current: " << currentChoice;
        return stream.str();
    }
};

struct Behavior {
    // Records the thread index in threads scheduled so far.
    // This works as a stack to exhaust every choice in depth-first-seach manner.
    std::vector<Choice> choices;
    size_t nextChoiceIndex = 0;
    int totalCount = 1;

    bool resetToNext() {
        nextChoiceIndex = 0;
        // Pop all exhausted tailing choices.
        while (!choices.empty() && choices.back().exhausted()) {
            choices.pop_back();
        }

        if (choices.empty())
            return false;

        // Advance the leaf.
        choices.back().next();
        totalCount++;
        return true;
    }

    void advanceOrAddChoice(std::vector<stdx::thread::id>& threads) {
        if (nextChoiceIndex == choices.size()) {
            // Traversed exiting path, start to explore new behavior.
            choices.push_back({threads, 0, nextChoiceIndex});
        } else {
            if (threads != choices[nextChoiceIndex].threads) {
                logd("threads: ");
                for (auto t : threads) logd(toString(t));
                logd("stored: nextChoiceIndex {}", nextChoiceIndex);
                for (auto t : choices[nextChoiceIndex].threads) logd(toString(t));
            }
            // We are reproducing a prefix of an existing behavior.
            invariant(threads == choices[nextChoiceIndex].threads);
        }
        logd("current choice: {}", choices[nextChoiceIndex]);
        nextChoiceIndex++;
    }

    const Choice& currentChoice() {
        invariant(nextChoiceIndex > 0);
        return choices[nextChoiceIndex - 1];
    }

    void print() {
        logd("====================================");
        logd("Current total behavior number: {}", totalCount);
        for (auto choice : choices) {
            logd(choice.toString());
        }
        logd("====================================");
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
            globalBehavior.advanceOrAddChoice(threads);
        }
        _condition.notify_all();
        return true;
    }

    void wait(const Identity& id) {
        // For threads before the test runs.
        if (threads.empty())
            return;

        std::stringstream stream;
        stream << stdx::this_thread::get_id();
        auto me = stream.str();

        stdx::unique_lock<stdx::mutex> lk(mutex);
        bool myTurn = false;
        while (!myTurn) {
            logd("{} : Waiting for scheduler on {}, next action: {}, count: {}, waiting: {}",
                 me,
                 id.name(),
                 globalBehavior.nextChoiceIndex,
                 threads.size(),
                 _threadsWaiting);

            if (!attemptToTriggerBarrier()) {
                uint64_t currentGeneration = _generation;
                while (currentGeneration == _generation) {
                    _condition.wait(lk);
                }
            }
            myTurn = globalBehavior.currentChoice().isMyTurn();
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
    Behavior globalBehavior;
    std::vector<stdx::thread::id> threads;

    // Barrier
    size_t _threadsWaiting;
    uint64_t _generation = 0;
    stdx::condition_variable _condition;
} scheduler;

class LockListener : public latch_detail::DiagnosticListener {
    void aboutToLock(const Identity& id) override {
        // logd("about to lock {}", id.name());
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
    do {
        scheduler.onCreateThread();

        auto t1 = stdx::thread([&] {
            scheduler.onCreateThread();
            {
                stdx::lock_guard lk(mutex);
                logd("XXX 1");
            }
            scheduler.onExitThread();
        });
        scheduler.waitForNewThread(t1.get_id());

        {
            stdx::lock_guard lk(mutex);
            logd("XXX 2");
        }
        scheduler.onExitThread();

        t1.join();

        scheduler.globalBehavior.print();
    } while (scheduler.globalBehavior.resetToNext());
    // Check the post condition of the interleaving.
    ASSERT(true);
}

TEST(Scheduler, Two_Action_VS_One_Action) {
    Mutex mutex = MONGO_MAKE_LATCH("test_mutex");
    do {
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

        {
            stdx::lock_guard lk(mutex);
            logd("XXX 2");
        }
        scheduler.onExitThread();

        t1.join();

        scheduler.globalBehavior.print();
    } while (scheduler.globalBehavior.resetToNext());
}

// Delegate all work to threads.
TEST(Scheduler, Two_Action_VS_Two_Action) {
    Mutex mutex = MONGO_MAKE_LATCH("test_mutex");
    do {
        scheduler.onCreateThread();

        auto t1 = stdx::thread([&] {
            scheduler.onCreateThread();
            logd("t1");
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

        auto t2 = stdx::thread([&] {
            scheduler.onCreateThread();
            logd("t2");
            {
                stdx::lock_guard lk(mutex);
                logd("XXX 2");
            }
            {
                stdx::lock_guard lk(mutex);
                logd("XXX 4");
            }
            scheduler.onExitThread();
        });
        scheduler.waitForNewThread(t2.get_id());

        // Main thread just starts the threads and doesn't run tests.
        scheduler.onExitThread();

        t1.join();
        t2.join();

        scheduler.globalBehavior.print();
    } while (scheduler.globalBehavior.resetToNext());
}

}  // namespace
}  // namespace repl
}  // namespace mongo