#!/usr/bin/env python
import rospy
from neupan_core import neupan_core

if __name__ == '__main__':
    neupan_node = neupan_core()
    neupan_node.run()
