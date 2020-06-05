
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

using ThreadId = int;
using ConditionVariableId = stdx::condition_variable*;

thread_local ThreadId localThreadId = 0;

std::string toString(ThreadId id) {
    std::stringstream stream;
    stream << id;
    return stream.str();
}

std::string toString(ConditionVariableId id) {
    std::stringstream stream;
    stream << (long long)id;
    return stream.str();
}

template <class T>
std::string toString(const std::vector<T>& v) {
    std::stringstream stream;
    stream << "[";
    for (auto& e : v) {
        stream << e << "  ";
    }
    stream << "]";
    return stream.str();
}

struct Choice {
    std::vector<ThreadId> threads;
    size_t currentChoice = 0;
    size_t actionIndex;

    bool isMyTurn() const {
        return threads[currentChoice] == localThreadId;
    }

    void next() {
        currentChoice++;
    }
    bool exhausted() const {
        return currentChoice == threads.size() - 1;
    }

    std::string toString() const {
        std::stringstream stream;
        stream << "Action " << actionIndex << " : " << repl::toString(threads)
               << " current index: " << currentChoice << " current thread: #"
               << threads[currentChoice];
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

    void advanceOrAddChoice(std::vector<ThreadId>& threads) {
        if (nextChoiceIndex == choices.size()) {
            // Traversed exiting path, start to explore new behavior.
            choices.push_back({threads, 0, nextChoiceIndex});
        } else {
            if (threads != choices[nextChoiceIndex].threads) {
                logd("threads: {}", threads);
                logd("stored : {}, nextChoiceIndex {}",
                     choices[nextChoiceIndex].threads,
                     nextChoiceIndex);
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

struct DisabledThread {
    ThreadId thread;
    ConditionVariableId cv;
    std::string toString() const {
        std::stringstream stream;
        stream << "<th: " << thread << " cv: " << (long long)cv << ">";
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
        // Only wait for enabled threads on the barrier.
        _threadsWaiting = threads.size() - disabledThreads.size();

        std::vector<ThreadId> disabledThreadIds, enabledThreads;
        std::transform(disabledThreads.begin(),
                       disabledThreads.end(),
                       std::back_inserter(disabledThreadIds),
                       [](DisabledThread& d) { return d.thread; });
        std::set_difference(threads.begin(),
                            threads.end(),
                            disabledThreadIds.begin(),
                            disabledThreadIds.end(),
                            std::back_inserter(enabledThreads));
        logd("threads: {}, disabled: {}, enabled: {}", threads, disabledThreadIds, enabledThreads);
        if (!enabledThreads.empty()) {
            // Always choose the first one.
            globalBehavior.advanceOrAddChoice(enabledThreads);
        } else if (!threads.empty()) {
            logd("Deadlock detected : threads {}", threads);
        }
        _condition.notify_all();
        return true;
    }

    void wait(const Identity& id) {
        stdx::unique_lock<stdx::mutex> lk(mutex);

        // For threads before the test runs.
        if (threads.empty())
            return;

        do {
            logd("#{} : Waiting for scheduler on {}, next action: {}, count: {}, waiting: {}",
                 localThreadId,
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
        } while (!globalBehavior.currentChoice().isMyTurn());
    }

    void onConditionVariableWait(const Identity& id, ConditionVariableId cv) {
        logd("#{} : Blocking on cv {} with mutex {}, next action: {}",
             localThreadId,
             (long long)cv,
             id.name(),
             globalBehavior.nextChoiceIndex);
        disabledThreads.push_back({localThreadId, cv});
        // This thread is blocked so it hits the barrier.
        attemptToTriggerBarrier();
    }

    // If condition_variable::wait() passes on the first check of "pred", it doesn't wait on the cv.
    // Thus, we need to enable it immediately.
    void onConditionVariableFinishWait(ConditionVariableId cv) {
        logd("#{} : Finish waiting on cv {}", localThreadId, (long long)cv);
        disabledThreads.erase(std::remove_if(disabledThreads.begin(),
                                             disabledThreads.end(),
                                             [&](auto& d) { return d.thread == localThreadId; }),
                              disabledThreads.end());
    }

    void onConditionVariableSignalAll(ConditionVariableId cv) {
        std::vector<ThreadId> v;
        for (auto& d : disabledThreads) {
            if (d.cv == cv)
                v.push_back(d.thread);
        }
        auto pred = [&](auto& d) { return d.cv == cv; };
        disabledThreads.erase(std::remove_if(disabledThreads.begin(), disabledThreads.end(), pred),
                              disabledThreads.end());
        logd("Signal all blocked threads on cv {}: {}", (long long)cv, v);
    }
    // void onConditionVariableSignalOne() {}

    void onCreateThread(ThreadId threadId) {
        localThreadId = threadId;
        stdx::lock_guard lk(mutex);
        threads.push_back(threadId);
        _threadsWaiting++;
    }

    void onExitThread() {
        stdx::lock_guard lk(mutex);
        threads.erase(std::remove(threads.begin(), threads.end(), localThreadId), threads.end());
        attemptToTriggerBarrier();
    }

    // TODO: we assume the new thread will not exit without waiting on the mutex so it won't
    // immediately exit.
    void waitForNewThread(ThreadId newThread) {
        while (true) {
            std::lock_guard lk(mutex);
            if (std::find(threads.begin(), threads.end(), newThread) != threads.end())
                break;
        }
    }

    void reset() {
        threads.clear();
        disabledThreads.clear();
        _threadsWaiting = 0;
        globalBehavior = Behavior();
    }

    std::mutex mutex;
    Behavior globalBehavior;
    std::vector<ThreadId> threads;
    std::vector<DisabledThread> disabledThreads;

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

    scheduler.reset();

    Mutex mutex = MONGO_MAKE_LATCH("test_mutex");
    do {
        scheduler.onCreateThread(0);

        auto t1 = stdx::thread([&] {
            scheduler.onCreateThread(1);
            {
                stdx::lock_guard lk(mutex);
                logd("XXX 1");
            }
            scheduler.onExitThread();
        });
        scheduler.waitForNewThread(1);

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
    scheduler.reset();

    Mutex mutex = MONGO_MAKE_LATCH("test_mutex");
    do {
        scheduler.onCreateThread(0);

        auto t1 = stdx::thread([&] {
            scheduler.onCreateThread(1);
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
        scheduler.waitForNewThread(1);

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
// The total behavior number is C(4, 2) = 6.
TEST(Scheduler, Two_Action_VS_Two_Action) {
    scheduler.reset();
    Mutex mutex = MONGO_MAKE_LATCH("test_mutex");
    do {
        scheduler.onCreateThread(0);

        auto t1 = stdx::thread([&] {
            scheduler.onCreateThread(1);
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
        scheduler.waitForNewThread(1);

        auto t2 = stdx::thread([&] {
            scheduler.onCreateThread(2);
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
        scheduler.waitForNewThread(2);

        // Main thread just starts the threads and doesn't run tests.
        scheduler.onExitThread();

        t1.join();
        t2.join();

        scheduler.globalBehavior.print();
    } while (scheduler.globalBehavior.resetToNext());
}

// This test case fails now.
TEST(Scheduler, Two_Action_VS_Two_Action_ConditionVariable) {
    scheduler.reset();
    Mutex mutex = MONGO_MAKE_LATCH("test_mutex");
    stdx::condition_variable cv;

    do {
        bool flag = false;
        scheduler.onCreateThread(0);

        auto t1 = stdx::thread([&] {
            scheduler.onCreateThread(1);
            {
                stdx::lock_guard lk(mutex);
                logd("XXX 1");
            }
            {
                stdx::lock_guard lk(mutex);
                logd("XXX 3");

                // Set the flag and signal.
                flag = true;
                scheduler.onConditionVariableSignalAll(&cv);
                cv.notify_all();
            }
            scheduler.onExitThread();
        });
        scheduler.waitForNewThread(1);

        auto t2 = stdx::thread([&] {
            scheduler.onCreateThread(2);
            {
                stdx::lock_guard lk(mutex);
                logd("XXX 2");
            }
            {
                stdx::unique_lock lk(mutex);
                // Wait until flag is set. This forces 3 to happen before 4.
                // Thus this excludes 2 behaviors out of 6 - (1 2 4 3) / (2 1 4 3)
                logd("Before cv wait");
                while (!flag) {
                    scheduler.onConditionVariableWait(latch_detail::Identity(mutex.getName()), &cv);
                    cv.wait(lk);
                }
                // scheduler.onConditionVariableFinishWait(&cv);
                logd("XXX 4");
            }
            scheduler.onExitThread();
        });
        scheduler.waitForNewThread(2);

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