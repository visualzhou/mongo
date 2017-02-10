/**
 *    Copyright 2015 MongoDB Inc.
 *
 *    This program is free software: you can redistribute it and/or  modify
 *    it under the terms of the GNU Affero General Public License, version 3,
 *    as published by the Free Software Foundation.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Affero General Public License for more details.
 *
 *    You should have received a copy of the GNU Affero General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *    As a special exception, the copyright holders give permission to link the
 *    code of portions of this program with the OpenSSL library under certain
 *    conditions as described in each individual source file and distribute
 *    linked combinations including the program with the OpenSSL library. You
 *    must comply with the GNU Affero General Public License in all respects for
 *    all of the code used other than as permitted herein. If you modify file(s)
 *    with this exception, you may extend this exception to your version of the
 *    file(s), but you are not obligated to do so. If you do not wish to do so,
 *    delete this exception statement from your version. If you delete this
 *    exception statement from all source files in the program, then also delete
 *    it in the license file.
 */

#include "mongo/platform/basic.h"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "mongo/db/catalog/collection_options.h"
#include "mongo/db/catalog/database.h"
#include "mongo/db/catalog/database_holder.h"
#include "mongo/db/catalog/document_validation.h"
#include "mongo/db/client.h"
#include "mongo/db/concurrency/d_concurrency.h"
#include "mongo/db/concurrency/write_conflict_exception.h"
#include "mongo/db/curop.h"
#include "mongo/db/db_raii.h"
#include "mongo/db/jsobj.h"
#include "mongo/db/repl/bgsync.h"
#include "mongo/db/repl/oplog.h"
#include "mongo/db/repl/oplog_interface_local.h"
#include "mongo/db/repl/replication_coordinator_global.h"
#include "mongo/db/repl/replication_coordinator_mock.h"
#include "mongo/db/repl/storage_interface.h"
#include "mongo/db/repl/storage_interface_mock.h"
#include "mongo/db/repl/sync_tail.h"
#include "mongo/db/service_context.h"
#include "mongo/db/service_context_d_test_fixture.h"
#include "mongo/stdx/mutex.h"
#include "mongo/unittest/unittest.h"
#include "mongo/util/concurrency/old_thread_pool.h"
#include "mongo/util/string_map.h"

#define MONGO_LOG_DEFAULT_COMPONENT ::mongo::logger::LogComponent::kReplication
#include "mongo/util/log.h"

namespace {

using namespace mongo;
using namespace mongo::repl;

class SyncTailTest : public ServiceContextMongoDTest {
protected:
    void _testSyncApplyInsertDocument(LockMode expectedMode);
    ServiceContext::UniqueOperationContext _txn;
    unsigned int _opsApplied;
    SyncTail::ApplyOperationInLockFn _applyOp;
    SyncTail::ApplyCommandInLockFn _applyCmd;
    SyncTail::IncrementOpsAppliedStatsFn _incOps;
    StorageInterfaceMock* _storageInterface = nullptr;

    // Implements the MultiApplier::ApplyOperationFn interface and does nothing.
    static Status noopApplyOperationFn(MultiApplier::OperationPtrs*) {
        return Status::OK();
    }

private:
    void setUp() override;
    void tearDown() override;
};

/**
 * Testing-only SyncTail that returns user-provided "document" for getMissingDoc().
 */
class SyncTailWithLocalDocumentFetcher : public SyncTail {
public:
    SyncTailWithLocalDocumentFetcher(const BSONObj& document);
    BSONObj getMissingDoc(OperationContext* txn, Database* db, const BSONObj& o) override;

private:
    BSONObj _document;
};

/**
 * Testing-only SyncTail that checks the operation context in shouldRetry().
 */
class SyncTailWithOperationContextChecker : public SyncTail {
public:
    SyncTailWithOperationContextChecker();
    bool shouldRetry(OperationContext* txn, const BSONObj& o) override;
};

void SyncTailTest::setUp() {
    ServiceContextMongoDTest::setUp();
    ReplSettings replSettings;
    replSettings.setOplogSizeBytes(5 * 1024 * 1024);

    auto service = getServiceContext();
    ReplicationCoordinator::set(
        service, stdx::make_unique<ReplicationCoordinatorMock>(service, replSettings));
    auto storageInterface = stdx::make_unique<StorageInterfaceMock>();
    _storageInterface = storageInterface.get();
    storageInterface->insertDocumentsFn = [](OperationContext*,
                                             const NamespaceString&,
                                             const std::vector<BSONObj>&) { return Status::OK(); };
    StorageInterface::set(service, std::move(storageInterface));

    _txn = cc().makeOperationContext();
    _opsApplied = 0;
    _applyOp = [](OperationContext* txn,
                  Database* db,
                  const BSONObj& op,
                  bool inSteadyStateReplication,
                  stdx::function<void()>) { return Status::OK(); };
    _applyCmd = [](OperationContext* txn, const BSONObj& op, bool) { return Status::OK(); };
    _incOps = [this]() { _opsApplied++; };
}

void SyncTailTest::tearDown() {
    _txn.reset();
    ServiceContextMongoDTest::tearDown();
    _storageInterface = nullptr;
}


using Mutex = stdx::mutex;
using GuardLock = stdx::lock_guard<Mutex>;

struct AllLockStates {
    Mutex m;
};

void runThread1(AllLockStates* allLocks) {
    Client::initThread(getThreadName().c_str());
    auto txn = cc().makeOperationContext();

    GuardLock lk(allLocks->m);
    sleepmillis(1000);
    log() << "t1 is going to wait";
    Lock::DBLock dbLock(txn->lockState(), "foo", MODE_X);
}


void runThread2(AllLockStates* allLocks) {
    Client::initThread(getThreadName().c_str());
    auto txn = cc().makeOperationContext();

    Lock::DBLock dbLock(txn->lockState(), "foo", MODE_IX);
    sleepmillis(1000);
    GuardLock lk(allLocks->m);
}

class SkunkworksTest : public SyncTailTest {};

TEST_F(SkunkworksTest, Dummy) {
    AllLockStates allLocks;

    stdx::thread t1(runThread1, &allLocks);
    stdx::thread t2(runThread2, &allLocks);
    stdx::thread t3(runThread2, &allLocks);


    t1.join();
    t2.join();
    t3.join();
}
}  // namespace
