# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

from colcon_core.package_selection import add_arguments \
    as add_packages_arguments
from colcon_core.package_selection import get_package_descriptors
from colcon_core.plugin_system import satisfies_version
from colcon_core.verb import VerbExtensionPoint


class SnapshotVerb(VerbExtensionPoint):
    """Snapshot packages."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(VerbExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102
        # only added so that package selection arguments can be used
        # which use the build directory to store state information
        parser.add_argument(
            '--build-base',
            default='build',
            help='The base path for all build directories (default: build)')

        add_packages_arguments(parser)

    def main(self, *, context):  # noqa: D102
        args = context.args

        # descriptors = get_package_descriptors(args)
        _ = get_package_descriptors(args)
