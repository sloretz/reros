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

from rclpy.context import Context as RclpyContext

# Using non-public rclpy API that may break any time!
# from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy


class Context(RclpyContext):

    def __init__(
        self,
        *,
        args: Optional[List[str]] = None,
        domain_id: Optional[int] = None 
    ):
        super().__init__()
        super().init(args=args, domain_id=domain_id)

    def __enter__(self):
        return self

    def __exit__(self, t, v, tb):
        self.shutdown()


class DefaultContext(Context):
    _lock: Lock = Lock()
    _context = None

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._context is None or not cls._context.ok():
                cls._context = object.__new__(DefaultContext, *args, **kwargs)
            return cls._context
