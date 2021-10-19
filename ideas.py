# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import reros

from std_msgs.msg import String


class Talker(reros.Node):

    def __init__(self):
        super().__init__('talker')
        self.i = 0
        self.pub = reros.Publisher(String, 'chatter', 10, node=self)
        timer_period = 1.0
        self.tmr = reros.Timer(timer_period, self.timer_callback, node=self)
        self.logger = reros.Logger(node=self)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: {0}'.format(self.i)
        self.i += 1
        self.logger.info('Publishing: "{0}"'.format(msg.data))
        self.pub.publish(msg)


def main():
    node = Talker()
    reros.spin()


if __name__ == '__main__':
    main()



# ----------------------------




# Default node ...

import time
import reros
from std_msgs.msg import String

pub = reros.Publisher(String, 'chatter', 10)
while reros.ok():
    time.sleep(1.0)
    pub.publish(msg)


# -------------------------------

def is_valid(value: reros.int64):
    return True


reros.Parameter('foo.bar', reros.int64, is_valid, node=self)





# -----------------------

sub = reros.Subscription(String, 'chatter', 10)

message = await sub.next_message_asyncio()
# OR
while sub.next_message() as message:
    print(message)
# OR
for message in sub:
    print(message)


# Implies there is a background thread reading from the wait set
# Have a ticket system that adds / removes things from background waitset


# --------------------------


# When shutting down the library matters, it REALLY matters

with reros.Context() as ctx:
    node = reros.Node('talker', context=ctx)
    # ...
