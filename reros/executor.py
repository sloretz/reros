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


class _MediatorHandle:

    __slots__ = (
        '_has_untaken_data',
        '_ready_callback',
        '_ready_callback',
        '_mediator_gc',
    )

    def __init__(self, gc, ready_callback):
        # This is only meant to be called by the Mediator
        self._ready_callback = ready_callback
        self._has_untaken_data = False
        self._ready_callback = ready_callback
        self._mediator_gc = gc

    def notify_data_ready(self):
        self._has_untaken_data = True

        if self._ready_callback:
            self._ready_callback()

    def notify_took_data(self):
        """
        Called by an entity to tell the mediator it took the data that was
        ready for it.

        The entity won't be executed again until this call is called.
        """
        # print('Notifying that data was taken')
        self._has_untaken_data = False
        if self._mediator_gc:
            self._mediator_gc.trigger_guard_condition()

    def has_untaken_data(self):
        return self._has_untaken_data


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
        executor = None,
    ):
        if context is None:
            context = DefaultContext()

        if executor is None:
            executor = _ThreadPoolExecutor()

        self._context = context

        # Use a dedidcated thread to notify ready entities
        self.__rcl_wait_thread = Thread(daemon=True, target=self.__rcl_wait)

        # Use a guard condition to wake when entities are added or removed
        self.__gc = _rclpy.GuardCondition(self._context.handle)

        self.__services = {}
        self.__subscribers = {}
        self.__clients = {}
        self.__guard_conditions = {
            self.__gc.pointer: (self.__gc, _MediatorHandle(None, None))
        }
        self.__timers = {}
        # TODO Action clients?
        # TODO Action Servers?

        self.__resize_wait_set()
        self.__rcl_wait_thread.start()

    def register_entity(self, entity, ready_callback: Optional[Callable]):
        """
        The ready_callback may choose to take the data right away, in which
        case it must return a callable with the work to be done with the data.
        Otherwise, the executor will wait for the entity to tell it 
        """
        # print(f'Registering entity {entity.pointer}')
        handle = _MediatorHandle(self.__gc, ready_callback)

        if isinstance(entity, _rclpy.Subscription):
            self.__subscribers[entity.pointer] = (entity, handle)

        self.__gc.trigger_guard_condition()
        return handle

    def __notify_all_ready(self, ready_pointers, entity_map):
        for ptr in ready_pointers:
            # print(f'{ptr} is ready!')
            handle = entity_map[ptr][1]
            maybe_work = handle.notify_data_ready()
            if maybe_work is not None:
                # If there is work to do, ask the executor to do it
                # This also means the entity is ready
                self.__executor.submit(maybe_work)

    def __resize_wait_set(self):
        # print('Resizing the wait set!')
        # Resize the wait set
        self.__wait_set = _rclpy.WaitSet(
            len(self.__subscribers),
            len(self.__guard_conditions),
            len(self.__timers),
            len(self.__clients),
            len(self.__services),
            0,  # TODO events?
            self._context.handle)

        # Add entities to the wait set
        for tmr, handle in self.__timers.values():
            if handle.has_untaken_data():
                # print('has untaken data', tmr.pointer)
                continue
            self.__wait_set.add_timer(tmr)
        for srv, handle in self.__services.values():
            if handle.has_untaken_data():
                # print('has untaken data', srv.pointer)
                continue
            self.__wait_set.add_service(srv)
        for cli, handle in self.__clients.values():
            if handle.has_untaken_data():
                # print('has untaken data', cli.pointer)
                continue
            self.__wait_set.add_client(cli)
        for sub, handle in self.__subscribers.values():
            if handle.has_untaken_data():
                # print('has untaken data', sub.pointer)
                continue
            self.__wait_set.add_subscription(sub)
        for gc, handle in self.__guard_conditions.values():
            if handle.has_untaken_data():
                # print('has untaken data', gc.pointer)
                continue
            self.__wait_set.add_guard_condition(gc)

    def __rcl_wait(self):
        # print('Starting wait loop')
        while self._context.ok():
            # TODO - redo this only when needed?
            self.__resize_wait_set()

            # print('About to wait')
            # TODO could do a watchdog in this thread
            # Wait on the wait set ... forever
            self.__wait_set.wait(-1)
            # print('Just woke up')

            ready_gcs = self.__wait_set.get_ready_entities('guard_condition')

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
                self.__subscribers)

            if self.__gc.pointer in ready_gcs:
                self.__guard_conditions[self.__gc.pointer][1].notify_took_data()
            #     self.__resize_wait_set()


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
