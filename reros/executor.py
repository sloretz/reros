# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from concurrent.futures import ThreadPoolExecutor as _ThreadPoolExecutor
from threading import Lock
from threading import Thread

import time

from typing import Callable
from typing import Optional


from .context import Context
from .context import DefaultContext

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


class SetableBool:
    """A mutable boolean that can be set inside a tuple."""

    __slots__ = ('value')

    def __init__(self, initial_value):
        self.value= initial_value

    def __bool__(self):
        return self.value


class Mediator:
    """
    Mediate between ROS entities, a task executor, and the wait set.

    This base class handles
    """
    # TODO this class can probably handle storing entities and callables

    def __init__(
        self,
        *,
        context: Context = None,
        executor: None,
    ):
        if context is None:
            context = DefaultContext()

        if executor is None:
            executor = _ThreadPoolExecutor()

        # Use a dedidcated thread to notify ready entities
        self.__rcl_wait_thread = Thread(daemon=True, target=self.__rcl_wait)

        # Use a guard condition to wake when entities are added or removed
        self.__gc = _rclpy.GuardCondition(self._context.handle)

        # Single underscore - intended to be used by direct subclasses only
        self._context = context
        self.__wait_set = _rclpy.WaitSet(
            0,
            1, # Always has the executor's guard condition
            0,
            0,
            0,
            0,
            self._context.handle)

        self.__services = {}
        self.__subscribers = {}
        self.__clients = {}
        self.__guard_conditions = {
            self.__gc.pointer(): (self.__gc, None, SetableBool())
        }
        self.__timers = {}
        # TODO Action clients?
        # TODO Action Servers?

        self.register_entity(self.__gc, lambda: pass)
        self.__rcl_wait_thread.start()

    def register_entity(self, entity, ready_callback: Optional[Callable]):
        """
        The ready_callback may choose to take the data right away, in which
        case it must return a callable with the work to be done with the data.
        Otherwise, the executor will wait for the entity to tell it 
        """
        # TODO - add entity to the wait set
        pass

    def __notify_all_ready(self, ready_pointers, entity_map):
        for ptr in ready_ptrs:
            entity_callback, took_last_data = entity_map[ptr][1]
            maybe_work = entity_ready_callback(took_last_data)
            if maybe_work is not None:
                # If there is work to do, ask the executor to do it
                # This also means the entity is ready
                self.__executor.submit(maybe_work)

    def __resize_wait_set(self):
        # Resize the wait set
        self.__wait_set = _rclpy.WaitSet(
            len(self.__subscribers),
            len(self.__guard_conditions),
            len(self.__timers),
            len(self.__clients),
            len(self.__services),
            self._context.handle)

    def __rcl_wait(self):
        while self._context.ok():
            # Add entities to the wait set

            # TODO could do a watchdog in this thread
            # Wait on the wait set ... forever
            self.__wait_set.wait(-1)

            ready_gcs = self.__wait_set.get_ready_entities('guard_condition'),

            # Notify all the relevant entities that they're ready
            self.__notify_all_ready(
                self.__wait_set.get_ready_entities('timer'),
                self.__timers)
            self.__notify_all_ready(
                ready_gcs,
                self.__guard_conditions)
            self.__notify_all_ready(
                self.__wait_set.get_ready_entities('service'),
                self.__services)
            self.__notify_all_ready(
                self.__wait_set.get_ready_entities('client'),
                self.__clients)
            self.__notify_all_ready(
                self.__wait_set.get_ready_entities('subscription'),
                self.__subscriptions)

            if self.__gc.pointer() in ready_gcs:
                self.__resize_wait_set()


#class ThreadPoolMediator(Mediator):
#
#    def __init__(
#        self,
#        *,
#        watchdog_period: float = 0.5,
#        max_workers: Optional[int] = None,
#        **kwargs
#    ):
#        super().__init__(**kwargs)
#        # Watchdog thread warns if threads have been exhausted
#        self.__watchdog_thread = Thread(daemon=True, target=self.__watchdog)
#
#        self.__thread_pool_executor = _ThreadPoolExecutor(max_workers)
#
#        self.__watchdog_thread.start()
#
#
#    def register_entity(self, entity, ready_callback: Callable):
#        # TODO - base class could manage this
#        pass
#
#    def __watchdog(self):
#        # TODO
#        return
#        while self._context.ok():
#            pass
#            last_feeding = time.monotonic()
#            time.sleep(watchdog_period)
#            # Feed watchdog by giving noop task


class DefaultMediator(Mediator):
    _lock: Lock = Lock()
    _executor = None

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._executor is None or not cls._executor.ok():
                cls._executor = object.__new__(DefaultMediator, *args, **kwargs)
            return cls._executor
