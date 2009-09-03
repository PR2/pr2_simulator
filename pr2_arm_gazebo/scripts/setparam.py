#!/usr/bin/env python
import roslib.scriptutil as s
s.get_param_server().setParam('/', 'robot_description', open('pr2_arm.xml').read())
