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

from threading import Event

from typing import Callable
from typing import TypeVar
from typing import Union
from typing import Optional

from .executor import DefaultMediator

from .node import DefaultNode
from .node import Node

# Using non-public rclpy API that may break any time!
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSProfile
from rclpy.type_support import check_is_valid_msg_type


MsgType = TypeVar('MsgType')


class Subscriber:

    def __init__(
        self,
        msg_type: MsgType,
        topic: str,
        qos_profile: Union[QoSProfile, int],
        *,
        node: Node = None,
        callback: Optional[Callable] = None,
        execution_mediator = None
    ):
        check_is_valid_msg_type(msg_type)
        self.__msg_type = msg_type
        if node is None:
            node = DefaultNode()

        if execution_mediator is None:
            execution_mediator = DefaultMediator()

        self.__executor = executor
        self.__callback = callback
        self.__data_ready = Event()

        qos_profile = self._validate_qos_or_depth_parameter(qos_profile)

        with node.handle:
            self.__subscriber = _rclpy.Subscription(
                node.handle, msg_type, topic, qos_profile.get_c_qos_profile())

        execution_mediator.register_entity(
            self.__subscriber,
            ready_callback=self.__notify_data_ready)

    def __notify_data_ready(self):
        """
        Notify the subscriber that it has data ready to be taken.

        If a potentially long-running function needs to be run, it is returned.
        """
        if self.__callback:
            return self.__call_callback

        # Notify the synchronous iterator that data is ready
        self.__data_ready.set()

        # TODO Notify an asynchronous iterator that data is ready

    def __take_data(self):
        """Take data from the subscription and return the message."""
        # Get data from the lower level
        raw = False
        msg = self.__subscriber.take_message(self.__msg_type, raw)

        # Tell synchronous iterator data is no longer ready
        self.__data_ready.clear()
        
        return msg

    def __call_callback(self):
        self.__call_callback(self.__take_data())

    def __iter__(self):
        """Synchronous message iterator."""
        if self.__callback is not None:
            raise RuntimeError('Cannot iterate because this subscription is'
                               ' using the callback interface.')
        return self

    def __next__(self):
        # Wait for data to be available, then take it!
        # TODO raise StopIteration if the context is shutdown
        self.__data_ready.wait()
        return self.__take_data()

    # TODO(sloretz) this belongs elsewhere
    def _validate_qos_or_depth_parameter(self, qos_or_depth) -> QoSProfile:
        if isinstance(qos_or_depth, QoSProfile):
            return qos_or_depth
        elif isinstance(qos_or_depth, int):
            if qos_or_depth < 0:
                raise ValueError('history depth must be greater than or equal to zero')
            return QoSProfile(depth=qos_or_depth)
        else:
            raise TypeError(
                'Expected QoSProfile or int, but received {!r}'.format(type(qos_or_depth)))

    @property
    def handle(self):
        return self.__subscriber
