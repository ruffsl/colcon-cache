# Copyright 2019 Dirk Thomas
# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

from colcon_cache.event_handler \
    import set_lockfile
from colcon_cache.verb_handler \
    import get_verb_handler_extensions
from colcon_core.event.job import JobEnded
from colcon_core.event.test import TestFailure
from colcon_core.event_handler import EventHandlerExtensionPoint
from colcon_core.plugin_system import satisfies_version


class LockfileEventHandler(EventHandlerExtensionPoint):
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

            verb_name = self.context.args.verb_name
            verb_handler_extensions = get_verb_handler_extensions()

            if verb_name not in verb_handler_extensions:
                return

            verb_handler_extension = verb_handler_extensions[verb_name]
            lockfile = verb_handler_extension.get_job_lockfile(job)

            if job in self._test_failures:
                return
            if str(data.rc) != '0':
                return

            if lockfile:
                set_lockfile(
                    job.task_context.args.build_base, verb_name, lockfile)
