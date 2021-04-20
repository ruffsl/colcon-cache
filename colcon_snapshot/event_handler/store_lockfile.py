# Copyright 2019 Dirk Thomas
# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

from colcon_core.event.job import JobEnded
from colcon_core.event.test import TestFailure
from colcon_core.event_handler import EventHandlerExtensionPoint
from colcon_core.plugin_system import satisfies_version
from colcon_core.verb.build import BuildPackageArguments
from colcon_core.verb.test import TestPackageArguments
from colcon_snapshot.event_handler \
    import get_previous_lockfile, set_lockfile
from colcon_snapshot.subverb.capture \
    import CaptureSnapshotPackageArguments


class StoreLockfileEventHandler(EventHandlerExtensionPoint):
    """
    Persist the lockfile of a job in a file in its build directory.

    The extension handles events of the following types:
    - :py:class:`colcon_core.event.job.JobEnded`
    - :py:class:`colcon_core.event.test.TestFailure`
    """

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            EventHandlerExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')
        self._test_failures = set()

    def __call__(self, event):  # noqa: D102
        data = event[0]

        if isinstance(data, TestFailure):
            job = event[1]
            self._test_failures.add(job)

        elif isinstance(data, JobEnded):
            job = event[1]

            if isinstance(job.task_context.args,
                          CaptureSnapshotPackageArguments):
                verb_name = 'snapshot'
                lockfile = data.rc
            elif isinstance(job.task_context.args,
                            BuildPackageArguments):
                verb_name = 'build'
                lockfile = get_previous_lockfile(
                    job.task_context.args.build_base,
                    'snapshot')
            elif isinstance(job.task_context.args,
                            TestPackageArguments):
                verb_name = 'test'
                lockfile = get_previous_lockfile(
                    job.task_context.args.build_base,
                    'build')
            else:
                return

            if job in self._test_failures:
                return
            if str(data.rc) != '0':
                if verb_name != 'snapshot':
                    return

            if lockfile is not None:
                set_lockfile(
                    job.task_context.args.build_base, verb_name, lockfile)
