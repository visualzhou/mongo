from __future__ import print_function

import gdb
import gdb.printing

import re
import struct

class Node:
  def __init__(self, systag, tid, description = None):
    self.systag = systag
    self.tid = tid
    self.description = description
    self.next_nodes = []

class Graph:
  def __init__(self):
    self.nodes = []

  def add_node(self, node):
    self.nodes.append(node)
    pass

  def find_node(self, systag, tid):
    for node in self.nodes:
      if systag is not None and node.systag == systag:
        return node

      if tid is not None and node.tid == tid:
        return node
    return None

  def add_edge(self, from_node, to_node):
    f = self.find_node(from_node.systag, from_node.tid)
    if f is None:
      print("Adding new node when adding its edge")
      self.add_node(from_node)
      f = from_node

    t = self.find_node(to_node.systag, to_node.tid)
    if t is None:
      print("Adding new node when adding its edge")
      self.add_node(to_node)
      t = to_node
 
    for n in f.next_nodes:
      if n.systag == t.systag or n.tid == t.tid:
        return
    f.next_nodes.append(t)


  def print(self):
    for node in self.nodes:
      print("Node", node.systag, node.tid, node.description)
      for to in node.next_nodes:
        print(" ->", to.systag, to.tid, to.description)

  def get_node_type(self, node):
    node_type = "Lock "
    if node.tid is not None:
        node_type = "Thread "
    return node_type

  def to_graph(self):
    sb = []
    sb.append("digraph \"mongod+lock-status\" {")
    for node in self.nodes:
        node_id = node.systag or node.tid
        for next_node in node.next_nodes:
            next_node_id = next_node.systag or next_node.tid
            sb.append("    \"%s\" -> \"%s\";" % (node_id, next_node_id))
    for node in self.nodes:
        node_id = node.systag or node.tid
        node_type = "Thread" if node.tid else "Lock"
        sb.append("    \"%s\" [label=\"%s\"]" % (node_id, node.description))
    sb.append("}")
    return "\n".join(sb)


global graph

def find_func_block(block):
  if not block:
    return None

  if block.function:
    return block

  return find_func_block(block.superblock)

def find_frame(function_name_pattern, frame = None, times = 10):
  if (times == 0):
    return None

  if not frame:
    frame = gdb.selected_frame()
  
  try:
    block = find_func_block(frame.block())
    if block:
      if re.search(function_name_pattern, block.function.name):
        print("found", function_name_pattern, block.function)
        return frame
  except RuntimeError as err:
    # print("ignoring ", err)
    pass

  if frame.older():
    return find_frame(function_name_pattern, frame.older(), times - 1)


def get_mutex_holder(sym, frame):
  value = sym.value(frame)
  holder = value["_M_mutex"]["__data"]["__owner"]
  return holder, value.address

def find_mutex_holder():
  frame = find_frame(r'std::mutex::lock\(\)')
  if not frame:
    return
 
  # Waiting for mutex locking!
  mutex_this, _ = gdb.lookup_symbol("this", frame.block())
  mutex_value = mutex_this.value(frame)
  mutex_holder = mutex_value["_M_mutex"]["__data"]["__owner"]

  # Process ID/PID,  Lightweight Process ID (LWPID), Thread ID (TID)
  me = gdb.selected_thread()
  tid = me.ptid[1] if me.ptid[1] > 0 else me.ptid[2]
  # print("Mutex:", tid, "is waiting on mutex at", mutex_value.address, "held by", mutex_holder)
  print("Mutex at", mutex_value, "held by thread tid", mutex_holder)

  global graph
  graph.add_edge(Node(None, tid), Node(str(mutex_value), None, "Mutex"))
  graph.add_edge(Node(str(mutex_value), "Mutex"), Node(None, int(str(mutex_holder))))
  return

  
def find_lock_manager_holders():
  # frame = find_frame(r'lockComplete')
  frame = find_frame(r'mongo::LockerImpl\<.*\>::')
  if not frame:
    return

  old_frame = gdb.selected_frame()
  frame.select()

  me = gdb.selected_thread()
  tid = me.ptid[1] if me.ptid[1] > 0 else me.ptid[2]

  locker_ptr_type = gdb.lookup_type("mongo::LockerImpl<false>").pointer()
  # grantedList = gdb.parse_and_eval("mongo::getGlobalLockManager()->_getBucket(resId)->findOrInsert(resId).grantedList")
  lock_head = gdb.parse_and_eval("mongo::getGlobalLockManager()->_getBucket(resId)->findOrInsert(resId)")
  # print(lock_head)
  grantedList = lock_head.dereference()["grantedList"]
  lock_request_ptr = grantedList["_front"]
  while lock_request_ptr != 0:
    lock_request = lock_request_ptr.dereference()
    locker_ptr = lock_request["locker"]
    locker_ptr = locker_ptr.cast(locker_ptr_type)
    locker = locker_ptr.dereference()
    thread_id = locker["_threadId"]["_M_thread"]
    print("MongoDB Lock at", lock_head, "held by thread id 0x{0:x}".format(int("{0}".format(thread_id))), lock_request["mode"])
    global graph
    graph.add_edge(Node(None, tid), Node(str(lock_head), None, "MongoDB lock"))
    graph.add_edge(Node(str(lock_head), None, "MongoDB lock"), Node("0x{0:x}".format(int(thread_id)), None))

    lock_request_ptr = lock_request["next"]

  # resume old frame
  old_frame.select()

def show_lock_holders():
  find_mutex_holder()
  find_lock_manager_holders()

def get_threads_info():
    """
    threads   Id   Target Id         Frame
    * 1    Thread 0x7f1d7206fcc0 (LWP 12657) "sync_tail_test" 0x00007f1d70fb198d in pthread_join (threadid=139764417513216, thread_return=0x0) at pthread_join.c:90
      2    Thread 0x7f1d70754700 (LWP 12658) "sync_tail_test" pthread_cond_timedwait@@GLIBC_2.3.2 () at ../sysdeps/unix/sysv/linux/x86_64/pthread_cond_timedwait.S:225
      3    Thread 0x7f1d6ff53700 (LWP 12659) "sync_tail_test" __lll_lock_wait () at ../sysdeps/unix/sysv/linux/x86_64/lowlevellock.S:135
      4    Thread 0x7f1d6f752700 (LWP 12660) "sync_tail_test" __lll_lock_wait () at ../sysdeps/unix/sysv/linux/x86_64/lowlevellock.S:135
    """
    INFO_PATTERN = "\d*\s*Thread (?P<pid>0x\w*)\s*\(LWP (?P<tid>\d*)\)"
    threads_info = gdb.execute("info threads", from_tty=False, to_string=True)
    global graph
    graph = Graph()
    for m in re.finditer(INFO_PATTERN, threads_info):
      print(m.group("pid"), m.group("tid"), m.group())
      graph.add_node(Node(m.group("pid"), int(m.group("tid")), m.group()))
    return graph


class MongoShowLocks(gdb.Command):
    """Analyze MongoDB process"""

    def __init__(self):
        register_mongo_command(self, "mongodb-show-locks", gdb.COMMAND_STATUS)

    def invoke(self, arg, _from_tty):
        if len(gdb.objfiles()) == 0:
            print("Skipping, not attached to an inferior")
            return

        main_binary_name = gdb.objfiles()[0].filename
        self.analyze_mongod()

    def analyze_mongod(self):
        """GDB in-process python supplement"""

        try:
            show_lock_holders()
        except gdb.Error as gdberr:
            print("Ignoring error '%s'" % str(gdberr))

MongoShowLocks()

class MongoDeadlockDetect(gdb.Command):
    """Analyze MongoDB process"""

    def __init__(self):
        register_mongo_command(self, "mongodb-deadlock-detect", gdb.COMMAND_STATUS)

    def invoke(self, arg, _from_tty):
        if len(gdb.objfiles()) == 0:
            print("Skipping, not attached to an inferior")
            return

        main_binary_name = gdb.objfiles()[0].filename
        self.analyze_mongod()

    def analyze_mongod(self):
        """GDB in-process python supplement"""

        try:
            get_threads_info()
            # Call into mongod, and dump the state of lock manager
            # Note that output will go to mongod's standard output, not the debugger output window
            gdb.execute("thread apply all mongodb-show-locks",
                        from_tty=False, to_string=False)
            global graph
            graph.print()
            print(graph.to_graph())

        except gdb.Error as gdberr:
            print("Ignoring error '%s'" % str(gdberr))

MongoDeadlockDetect()
