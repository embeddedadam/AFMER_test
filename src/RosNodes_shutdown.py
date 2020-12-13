#!/usr/bin/env python
import os
import time

nodes = os.popen("rosnode list").readlines()
for i in range(len(nodes)):
    nodes[i] = nodes[i].replace("\n","")

for node in nodes:
    time.sleep(1)
    os.system("rosnode kill "+ node)
