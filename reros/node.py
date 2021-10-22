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

from threading import Lock
from typing import List
from typing import Optional

import uuid

from .context import Context
from .context import DefaultContext

# Using non-public rclpy API that may break any time!
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


class Node:

    def __init__(self,
        name: str = None,
        *,
        context: Context = None,
        cli_args: List[str] = None,
        namespace: str = '',
        enable_rosout: bool = True,
        use_global_arguments: bool = True,
        # start_parameter_services: bool = True,
    ):
        if name is None:
            name = 'reros_' + str(uuid.uuid4()).replace('-', '')
        if context is None:
            context = DefaultContext()

        self._context = context

        self.__node = _rclpy.Node(
            name,
            namespace,
            self._context.handle,
            cli_args,
            use_global_arguments,
            enable_rosout
        )

    @property
    def handle(self):
        return self.__node


class DefaultNode(Node):
    _lock: Lock = Lock()
    _node = None

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._node is None or not cls._node.ok():
                cls._node = object.__new__(DefaultNode, *args, **kwargs)
            return cls._node
