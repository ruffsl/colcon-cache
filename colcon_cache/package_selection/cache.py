# Copyright 2019 Dirk Thomas
# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

import os

from colcon_core.package_selection import logger
from colcon_core.package_selection import PackageSelectionExtensionPoint
from colcon_core.plugin_system import satisfies_version
from colcon_cache.event_handler \
    import get_previous_lockfile


class CachePackageSelectionExtension(PackageSelectionExtensionPoint):
    """Skip a set of packages based on lockfiles from previous caches."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            PackageSelectionExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102
        group = parser.add_mutually_exclusive_group()
        group.add_argument(
            '--packages-select-cache-miss', action='store_true',
            help='Only process a subset of packages which have failed to '
                 'build previously (aborted packages are not '
                 'considered errors)')
        group.add_argument(
            '--packages-skip-cache-hit', action='store_true',
            help='Skip a set of packages which have finished to build '
                 'previously')

    def select_packages(self, args, decorators):  # noqa: D102
        if not any((
            args.packages_select_cache_miss,
            args.packages_skip_cache_hit,
        )):
            return

        if not hasattr(args, 'build_base'):
            if args.packages_select_build_cache_miss:
                argument = '--packages-select-cache-miss'
            elif args.packages_skip_build_cache_hit:
                argument = '--packages-skip-cache-hit'
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

            verb_name = args.verb_name
            if verb_name == 'cache':
                reference_name = None
            elif verb_name == 'build':
                reference_name = 'cache'
            elif verb_name == 'test':
                reference_name = 'build'
            else:
                assert False

            package_build_base = os.path.join(
                args.build_base, pkg.name)
            verb_lockfile = get_previous_lockfile(
                package_build_base, verb_name)
            reference_lockfile = get_previous_lockfile(
                package_build_base, reference_name)

            package_kind = None
            missing_kind = None
            if verb_lockfile is None:
                missing_kind = verb_name
            if reference_lockfile is None:
                missing_kind = reference_name

            if args.packages_select_cache_miss:
                if missing_kind == reference_name:
                    package_kind = ('missing {reference_name} lockfile'
                                    .format_map(locals()))

            if missing_kind is None:
                if reference_lockfile == verb_lockfile:
                    package_kind = ('matching {reference_name} and '
                                    '{verb_name} lockfiles'
                                    .format_map(locals()))

            if package_kind is not None:
                logger.info(
                    "Skipping {package_kind} package '{pkg.name}' in "
                    "'{pkg.path}'".format_map(locals()))
                decorator.selected = False
