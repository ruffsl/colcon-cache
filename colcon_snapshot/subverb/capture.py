# Copyright 2016-2018 Dirk Thomas
# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

from collections import OrderedDict
import os
import os.path
from pathlib import Path

from colcon_core.argument_parser.destination_collector \
    import DestinationCollectorDecorator
from colcon_core.event.job import JobUnselected
from colcon_core.event_handler import add_event_handler_arguments
from colcon_core.executor import add_executor_arguments
from colcon_core.executor import execute_jobs
from colcon_core.executor import Job
from colcon_core.executor import OnError
from colcon_core.package_identification.ignore import IGNORE_MARKER
from colcon_core.package_selection import add_arguments \
    as add_packages_arguments
from colcon_core.package_selection import get_packages
from colcon_core.plugin_system import satisfies_version
from colcon_core.task import add_task_arguments
from colcon_core.task import get_task_extension
from colcon_core.task import TaskContext
from colcon_core.verb import check_and_mark_build_tool
from colcon_core.verb import logger
from colcon_core.verb import update_object

from colcon_snapshot.subverb import SnapshotSubverbExtensionPoint


class CaptureSnapshotPackageArguments:
    """Arguments to snapshot capture a specific package."""

    def __init__(self, pkg, args, *, additional_destinations=None):
        """
        Construct a CaptureSnapshotPackageArguments.

        :param pkg: The package descriptor
        :param args: The parsed command line arguments
        :param list additional_destinations: The destinations of additional
          arguments
        """
        super().__init__()
        self.path = os.path.abspath(
            os.path.join(os.getcwd(), str(pkg.path)))
        self.build_base = os.path.abspath(os.path.join(
            os.getcwd(), args.build_base, pkg.name))

        # set additional arguments
        for dest in (additional_destinations or []):
            # from the command line
            if hasattr(args, dest):
                update_object(
                    self, dest, getattr(args, dest),
                    pkg.name, 'snapshot capture', 'command line')
            # from the package metadata
            if dest in pkg.metadata:
                update_object(
                    self, dest, pkg.metadata[dest],
                    pkg.name, 'snapshot capture', 'package metadata')


class CaptureSnapshotSubverb(SnapshotSubverbExtensionPoint):
    """Capture current snapshot for packages."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            SnapshotSubverbExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102
        parser.add_argument(
            '--build-base',
            default='build',
            help='The base path for all build directories (default: build)')
        add_executor_arguments(parser)
        add_event_handler_arguments(parser)
        add_packages_arguments(parser)

        decorated_parser = DestinationCollectorDecorator(parser)
        add_task_arguments(decorated_parser, 'colcon_snapshot.task.capture')
        self.task_argument_destinations = decorated_parser.get_destinations()

    def main(self, *, context):  # noqa: D102
        check_and_mark_build_tool(context.args.build_base)

        self._create_paths(context.args)

        decorators = get_packages(
            context.args,
            additional_argument_names=self.task_argument_destinations,
            recursive_categories=('run', ))

        jobs, unselected_packages = self._get_jobs(
            context.args, decorators)

        # TODO: OnError.continue_ is a workaround given rc need not be 0
        # on_error = OnError.interrupt \
        #     if not context.args.continue_on_error \
        #     else OnError.skip_downstream
        on_error = OnError.continue_

        def post_unselected_packages(*, event_queue):
            nonlocal unselected_packages
            names = [pkg.name for pkg in unselected_packages]
            for name in sorted(names):
                event_queue.put(
                    (JobUnselected(name), None))

        rc = execute_jobs(
            context, jobs, on_error=on_error,
            pre_execution_callback=post_unselected_packages)

        return rc

    def _create_paths(self, args):
        self._create_path(args.build_base)

    def _create_path(self, path):
        path = Path(os.path.abspath(path))
        if not path.exists():
            path.mkdir(parents=True, exist_ok=True)
        ignore_marker = path / IGNORE_MARKER
        if not os.path.lexists(str(ignore_marker)):
            with ignore_marker.open('w'):
                pass

    def _get_jobs(self, args, decorators):
        jobs = OrderedDict()
        unselected_packages = set()
        for decorator in decorators:
            pkg = decorator.descriptor

            if not decorator.selected:
                unselected_packages.add(pkg)
                continue

            extension = get_task_extension(
                'colcon_snapshot.task.capture', pkg.metadata['vcs_type'])
            if not extension:
                logger.warning(
                    "No task extension to 'snapshot capture' "
                    "a '{pkg.type}' package"
                    .format_map(locals()))
                continue

            package_args = CaptureSnapshotPackageArguments(
                pkg, args, additional_destinations=self
                .task_argument_destinations.values())
            ordered_package_args = ', '.join([
                ('%s: %s' % (repr(k), repr(package_args.__dict__[k])))
                for k in sorted(package_args.__dict__.keys())
            ])
            logger.debug(
                "Staging package '{pkg.name}' with the following arguments: "
                '{{{ordered_package_args}}}'.format_map(locals()))
            task_context = TaskContext(
                pkg=pkg, args=package_args,
                dependencies=None)

            job = Job(
                identifier=pkg.name,
                dependencies=[],
                task=extension, task_context=task_context)

            jobs[pkg.name] = job
        return jobs, unselected_packages
