# -------------------------------------------------------------------------------------------------------------------- #
# File Name    : MultiProcessBase.py
# Project Name : ExaRobotCtrl
# Author       : Raim.Delgado
# Organization : SeoulTech
# Description  :
# [Revision History]
# >> 2022.02.17 - First Commit
# -------------------------------------------------------------------------------------------------------------------- #
import multiprocessing
import os
import sys
import logging
import threading
import time
from multiprocessing import Process, Queue, Pipe
from typing import Union

FILE_PATH = os.path.dirname(os.path.realpath(__file__))  # %PROJECT_ROOT%/Library/Socket
ROOT_PATH = os.path.dirname(os.path.dirname(FILE_PATH))
INCLUDE_PATH = os.path.join(ROOT_PATH, "Include")
MDI_PATH = os.path.join(INCLUDE_PATH, "MdiBackground")
DOCK_PATH = os.path.join(INCLUDE_PATH, "DockWidgets")
MISC_PATH = os.path.join(INCLUDE_PATH, "Misc")
RESOURCES_PATH = os.path.join(INCLUDE_PATH, "Resources")
LIBRARY_PATH = os.path.join(ROOT_PATH, "Library")
SERIAL_PATH = os.path.join(LIBRARY_PATH, "Serial")
sys.path.extend([FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH, LIBRARY_PATH, SERIAL_PATH])
sys.path = list(set(sys.path))
del FILE_PATH, ROOT_PATH, INCLUDE_PATH, MDI_PATH, DOCK_PATH, RESOURCES_PATH, LIBRARY_PATH, SERIAL_PATH

from Commons import PySignal

# https://stackoverflow.com/questions/8463008/multiprocessing-pipe-vs-queue
# Sending 10000 numbers to Pipe() took 0.0369849205017 seconds
# Sending 100000 numbers to Pipe() took 0.328398942947 seconds
# Sending 1000000 numbers to Pipe() took 3.17266988754 seconds
# Sending 10000 numbers to Queue() took 0.105256080627 seconds
# Sending 100000 numbers to Queue() took 0.980564117432 seconds
# Sending 1000000 numbers to Queue() took 10.1611330509 seconds

class ThreadQueueBroadcaster(threading.Thread):
    keepAlive = True

    # todo: do we need mutex?
    def __init__(self, pQueue: Queue):
        super(ThreadQueueBroadcaster, self).__init__()
        self.setDaemon(True)
        self.sig_terminated = PySignal()
        self.sig_queue_bcast = PySignal(object)
        self.q = pQueue

    def stop(self):
        self.keepAlive = False

    def run(self) -> None:
        while self.keepAlive:
            if not self.q.empty():
                msg = self.q.get()
                self.sig_queue_bcast.emit(msg)
            else:
                time.sleep(0.001)

        self.sig_terminated.emit()


class CMultiProcessBase:
    procDescriptor: Process
    threadQueueBcast: Union[ThreadQueueBroadcaster, None] = None
    threadQueueBcastBk: Union[ThreadQueueBroadcaster, None] = None

    def __init__(self, astrName: str = "BASE", apRoutine=None, abQueueBkup: bool = False, abDaemon: bool = True):
        routine = self.routine_proc
        if apRoutine is not None:
            routine = apRoutine

        self.sig_queue_bcast = PySignal(object)
        self.sig_queue_bcast_bk = PySignal(object)
        self.sig_error = PySignal(str)

        # todo: check if we need bidirectional for ACK
        self.pipeParent, self.pipeChild = Pipe()
        self.queueRecv = Queue()
        if abQueueBkup:
            self.queueRecvBk = Queue()
        else:
            self.queueRecvBk = None

        proc_args = (self.pipeChild, self.queueRecv, self.queueRecvBk)
        self.procDescriptor = Process(target=routine, name=astrName, args=proc_args)
        self.procDescriptor.daemon = abDaemon
        self.start_thread_queue_bcast()
        if abQueueBkup:
            self.start_thread_queue_bcast_bk()

    def start(self):
        self.procDescriptor.start()

    def release(self):
        self.procDescriptor.terminate()
        self.procDescriptor.join()
        self.stop_thread_queue_bcast()
        self.stop_thread_queue_bcast_bk()
        self.pipeParent.close()

    def send_command(self, adictCmd: dict):
        if not self.procDescriptor.is_alive():
            self.sig_error.emit("MP ERROR")
            return

        while not self.pipeParent.writable:
            time.sleep(1e-3)
            pass

        self.pipeParent.send(adictCmd)

    # aPipe: is an instance of PipeConnection on Windows. In Linux, this is different (unknown)
    def routine_proc(self, aPipe, aQueue: Queue, aQueueBk: Union[Queue, None]):
        pass

    def start_thread_queue_bcast(self):
        if self.threadQueueBcast is None:
            self.threadQueueBcast = ThreadQueueBroadcaster(self.queueRecv)
            self.threadQueueBcast.sig_terminated.connect(self.on_terminate_thread_queue_bcast)
            self.threadQueueBcast.sig_queue_bcast.connect(self.on_bcast_thread_queue_bcast)
            self.threadQueueBcast.start()

    def on_bcast_thread_queue_bcast(self, msg: object):
        self.sig_queue_bcast.emit(msg)

    def stop_thread_queue_bcast(self):
        if self.threadQueueBcast is not None:
            self.threadQueueBcast.stop()

    def on_terminate_thread_queue_bcast(self):
        del self.threadQueueBcast
        self.threadQueueBcast = None

    def start_thread_queue_bcast_bk(self):
        if self.threadQueueBcastBk is None:
            self.threadQueueBcastBk = ThreadQueueBroadcaster(self.queueRecvBk)
            self.threadQueueBcastBk.sig_terminated.connect(self.on_terminate_thread_queue_bcast_bk)
            self.threadQueueBcastBk.sig_queue_bcast.connect(self.on_bcast_thread_queue_bcast_bk)
            self.threadQueueBcastBk.start()

    def stop_thread_queue_bcast_bk(self):
        if self.threadQueueBcastBk is not None:
            self.threadQueueBcastBk.stop()

    def on_bcast_thread_queue_bcast_bk(self, msg: object):
        self.sig_queue_bcast_bk.emit(msg)

    def on_terminate_thread_queue_bcast_bk(self):
        del self.threadQueueBcastBk
        self.threadQueueBcastBk = None

    @property
    def PID(self):
        return self.procDescriptor.pid

    @property
    def NAME(self):
        return self.procDescriptor.name

    @property
    def NICENESS(self):
        import psutil
        this_proc = psutil.Process(self.PID)
        return this_proc.nice()


def f(connection):
    print('parent process:', os.getppid())
    print('process id:', os.getpid())
    connection.send([42, None, 'hello'])
    connection.close()


if __name__ == '__main__':
    parent_conn, child_conn = Pipe()
    p = Process(target=f, args=(child_conn,))
    p.start()
    print(parent_conn.recv())  # prints "[42, None, 'hello']"
    p.join()
