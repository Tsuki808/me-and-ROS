#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Stop the current user's previous local ROS runtime before a new launch."""
from __future__ import print_function
import errno
import os
import signal
import sys
import time


ROS_PROGRAMS = set(['roslaunch', 'roscore', 'rosmaster', 'rosout', 'rosrun',
                    'rosbag', 'rostopic', 'rosservice', 'rosnode', 'rosparam',
                    'rosmsg', 'rossrv', 'roswtf', 'rviz', 'gzserver', 'gzclient',
                    'rqt', 'rqt_gui'])
READ_ONLY_OPTIONS = set(['--files', '--args', '--nodes', '--find-node',
                         '--dump-params', '--ros-args'])


def processes():
    result = {}
    for name in os.listdir('/proc'):
        if not name.isdigit():
            continue
        directory = '/proc/' + name
        try:
            if os.stat(directory).st_uid != os.getuid():
                continue
            with open(directory + '/cmdline', 'rb') as stream:
                argv = [item.decode('utf-8', 'replace') for item in stream.read().split(b'\0') if item]
            with open(directory + '/stat') as stream:
                fields = stream.read().rsplit(') ', 1)[1].split()
            if argv and fields[0] != 'Z':
                result[int(name)] = dict(argv=argv, parent=int(fields[1]), start=fields[19])
        except (IOError, OSError):
            continue  # A process may exit while /proc is being read.
    return result


def targets(table, protected, workspace):
    selected = set()
    for pid, item in table.items():
        argv = item['argv']
        program = argv[:2]  # executable, or interpreter plus script
        if (any(os.path.basename(arg) in ROS_PROGRAMS or arg.startswith('/opt/ros/') for arg in program) or
                any(arg.startswith('__name:=') or arg.startswith('__log:=') for arg in argv[1:]) or
                any(arg.startswith(workspace + '/devel/lib/') or
                    (arg.startswith(workspace + '/src/') and arg.endswith('.py')) for arg in program)):
            selected.add(pid)
    selected.difference_update(protected)
    while True:
        children = set(pid for pid, item in table.items()
                       if item['parent'] in selected and pid not in protected)
        if children.issubset(selected):
            return selected
        selected.update(children)


def main():
    table = processes()
    protected, pid = set(), os.getpid()
    while pid in table and pid not in protected:
        protected.add(pid)
        pid = table[pid]['parent']
    # Launch inspection also evaluates parameter commands; it must remain read-only.
    launch_found = False
    for pid in protected:
        argv = table[pid]['argv']
        if any(os.path.basename(arg) == 'roslaunch' for arg in argv[:2]):
            launch_found = True
            if any(arg.split('=', 1)[0] in READ_ONLY_OPTIONS for arg in argv[1:]):
                return
    if not launch_found:
        return  # Loading a launch/model through a Python library is also read-only.
    package = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    workspace = os.path.dirname(os.path.dirname(package))
    selected = targets(table, protected, workspace)
    if not selected:
        return
    sys.stderr.write('[mode2 startup] Stopping previous ROS/Gazebo processes: %s\n' % sorted(selected))
    for signum, seconds in [(signal.SIGINT, 15.), (signal.SIGTERM, 5.), (signal.SIGKILL, 2.)]:
        table = processes()
        selected = targets(table, protected, workspace)
        for pid in sorted(selected):
            try:
                with open('/proc/%d/stat' % pid) as stream:
                    start = stream.read().rsplit(') ', 1)[1].split()[19]
                if start != table[pid]['start']:
                    continue
                os.kill(pid, signum)
            except (IOError, OSError) as exc:
                if exc.errno not in [errno.ENOENT, errno.ESRCH]:
                    raise
        deadline = time.time() + seconds
        while time.time() < deadline:
            if not targets(processes(), protected, workspace):
                sys.stderr.write('[mode2 startup] Previous ROS background processes stopped.\n')
                return
            time.sleep(.1)
    raise RuntimeError('Previous ROS processes did not exit; the new simulation was not started')


if __name__ == '__main__':
    main()
