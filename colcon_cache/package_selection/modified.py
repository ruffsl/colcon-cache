# Copyright 2019 Dirk Thomas
# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

import os

from colcon_cache.verb_handler import get_verb_handler_extensions
from colcon_core.package_selection import logger
from colcon_core.package_selection import PackageSelectionExtensionPoint
from colcon_core.plugin_system import satisfies_version


class ModifiedPackageSelection(PackageSelectionExtensionPoint):
    """Skip a set of packages based on lockfiles from current lock."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            PackageSelectionExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102
        group = parser.add_mutually_exclusive_group()
        group.add_argument(
            '--packages-select-cache-modified', action='store_true',
            help='Only process a subset of packages whose cache '
                 'denote package modifications (packages without lockfiles '
                 'are not considered as modified)')
        group.add_argument(
            '--packages-select-cache-unmodified', action='store_true',
            help='Only process a subset of packages whose cache '
                 'denote no package modifications (packages without lockfiles '
                 'are not considered as unmodified)')

    def select_packages(self, args, decorators):  # noqa: D102
        if not any((
            args.packages_select_cache_modified,
            args.packages_select_cache_unmodified,
        )):
            return

        if args.packages_select_cache_modified:
            argument = '--packages-select-cache-modified'
        elif args.packages_select_cache_unmodified:
            argument = '--packages-select-cache-unmodified'
        else:
            assert False

        if not hasattr(args, 'build_base'):
            logger.warning(
                "Ignoring '{argument}' since the invoked verb doesn't have a "
                "'--build-base' argument and therefore can't access "
                'information about the previous state of a package'
                .format_map(locals()))
            return

        verb_name = args.packages_select_cache_key
        if not verb_name:
            verb_name = args.verb_name
        verb_handler_extensions = get_verb_handler_extensions()

        if verb_name in verb_handler_extensions:
            verb_handler_extension = verb_handler_extensions[verb_name]
        else:
            logger.warning(
                "Ignoring '{argument}' since the respective verb "
                "'{verb_name}' doesn't have a colcon cache verb "
                "handler extension and therefore can't access "
                'information about the relative state of a package'
                .format_map(locals()))
            return

        for decorator in decorators:
            # skip packages which have already been ruled out
            if not decorator.selected:
                continue

            pkg = decorator.descriptor

            verb_lockfile = verb_handler_extension\
                .get_current_lockfile(args, pkg.name)

            package_kind = None
            if verb_lockfile is None:
                package_kind = ('without lockfile')
            else:
                if args.packages_select_cache_modified:
                    if not verb_lockfile.is_modified():
                        package_kind = ('unmodified')

                if args.packages_select_cache_unmodified:
                    if verb_lockfile.is_modified():
                        package_kind = ('modified')

            if package_kind:
                logger.info(
                    "Skipping {package_kind} package '{pkg.name}' in "
                    "'{pkg.path}'".format_map(locals()))
                decorator.selected = False
