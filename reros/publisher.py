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

from typing import TypeVar, Union

from .node import DefaultNode
from .node import Node

# Using non-public rclpy API that may break any time!
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSProfile
from rclpy.type_support import check_is_valid_msg_type


MsgType = TypeVar('MsgType')


class Publisher:

    def __init__(
        self,
        msg_type: MsgType,
        topic: str,
        qos_profile: Union[QoSProfile, int],
        *,
        node: Node = None,
    ):
        check_is_valid_msg_type(msg_type)
        self.__msg_type = msg_type
        if node is None:
            node = DefaultNode()

        qos_profile = self._validate_qos_or_depth_parameter(qos_profile)

        with node.handle:
            self.__publisher = _rclpy.Publisher(
                node.handle, msg_type, topic, qos_profile.get_c_qos_profile())


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
        return self.__publisher

    def publish(self, msg: Union[MsgType, bytes]):
        with self.handle:
            if isinstance(msg, self.__msg_type):
                self.__publisher.publish(msg)
            elif isinstance(msg, bytes):
                self.__publisher.publish_raw(msg)
            else:
                raise TypeError('Expected {}, got {}'.format(self.__msg_type, type(msg)))
