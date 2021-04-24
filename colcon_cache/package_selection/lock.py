# Copyright 2019 Dirk Thomas
# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

import os

from colcon_core.package_selection import logger
from colcon_core.package_selection import PackageSelectionExtensionPoint
from colcon_core.plugin_system import satisfies_version
from colcon_cache.event_handler \
    import get_previous_lockfile


class LockPackageSelectionExtension(PackageSelectionExtensionPoint):
    """Skip a set of packages based on lockfiles from current capture."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            PackageSelectionExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102
        group = parser.add_mutually_exclusive_group()
        group.add_argument(
            '--packages-select-lock-changed', action='store_true',
            help='Only process a subset of packages whose lockfile '
                 'denote package changes (packages without lockfiles '
                 'are not considered as changed)')
        group.add_argument(
            '--packages-select-lock-unchanged', action='store_true',
            help='Only process a subset of packages whose lockfile '
                 'denote no package changes (packages without lockfiles '
                 'are not considered as unchanged)')

    def select_packages(self, args, decorators):  # noqa: D102
        if not any((
            args.packages_select_lock_changed,
            args.packages_select_lock_unchanged,
        )):
            return

        if not hasattr(args, 'build_base'):
            if args.packages_select_lock_changed:
                argument = '--packages-select-lock-changed'
            elif args.packages_select_lock_unchanged:
                argument = '--packages-select-lock-unchanged'
            else:
                assert False
            logger.warning(
                "Ignoring '{argument}' since the invoked verb doesn't have a "
                "'--build-base' argument and therefore can't access "
                'information about the previous state of a package'
                .format_map(locals()))
            return

        for decorator in decorators:
            # skip packages which have already been ruled out
            if not decorator.selected:
                continue

            pkg = decorator.descriptor

            verb_name = 'cache'

            package_build_base = os.path.join(
                args.build_base, pkg.name)
            verb_lockfile = get_previous_lockfile(
                package_build_base, verb_name)

            package_kind = None
            if verb_lockfile is None:
                package_kind = ('without lockfile')
            else:
                if args.packages_select_lock_changed:
                    if not verb_lockfile.is_changed():
                        package_kind = ('unchanged')

                if args.packages_select_lock_unchanged:
                    if verb_lockfile.is_changed():
                        package_kind = ('changed')

            if package_kind is not None:
                logger.info(
                    "Skipping {package_kind} package '{pkg.name}' in "
                    "'{pkg.path}'".format_map(locals()))
                decorator.selected = False
