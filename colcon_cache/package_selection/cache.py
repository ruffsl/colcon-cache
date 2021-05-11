# Copyright 2019 Dirk Thomas
# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

import os

from colcon_cache.verb_handler import get_verb_handler_extensions
from colcon_core.package_selection import logger
from colcon_core.package_selection import PackageSelectionExtensionPoint
from colcon_core.plugin_system import satisfies_version


class CachePackageSelectionExtension(PackageSelectionExtensionPoint):
    """Skip a set of packages based on lockfiles from previous caches."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            PackageSelectionExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102
        group = parser.add_mutually_exclusive_group()
        group.add_argument(
            '--packages-select-cache-void', action='store_true',
            help='Only process a subset of packages that miss its '
                 'reference cache (packages without a reference cache '
                 'are not considered as cache miss)')
        group.add_argument(
            '--packages-skip-cache-valid', action='store_true',
            help='Skip a set of packages which hit its '
                 'reference cache (packages without a verb cache '
                 'are not considered as cache hit)')
        parser.add_argument(
            '--packages-respective-cache-verb',
            choices=get_verb_handler_extensions().keys(),
            default=None,
            help='Only process caches for respective verb. '
                 'Defaults to invoked verb if unspecified.')

    def select_packages(self, args, decorators):  # noqa: D102
        if not any((
            args.packages_select_cache_void,
            args.packages_skip_cache_valid,
        )):
            return

        if args.packages_select_cache_void:
            argument = '--packages-select-cache-void'
        elif args.packages_skip_cache_valid:
            argument = '--packages-skip-cache-valid'
        else:
            assert False

        if not hasattr(args, 'build_base'):
            logger.warning(
                "Ignoring '{argument}' since the invoked verb doesn't have a "
                "'--build-base' argument and therefore can't access "
                'information about the relative state of a package'
                .format_map(locals()))
            return

        verb_name = args.packages_respective_cache_verb
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

            package_build_base = os.path.join(
                args.build_base, pkg.name)

            verb_lockfile = verb_handler_extension\
                .get_current_lockfile(package_build_base)
            reference_lockfile = verb_handler_extension\
                .get_reference_lockfile(package_build_base)
            reference_name = verb_handler_extension.reference_name

            package_kind = None
            missing_kind = None
            if reference_lockfile is None:
                missing_kind = reference_name
            elif verb_lockfile is None:
                missing_kind = verb_name

            if args.packages_select_cache_void:
                if missing_kind == reference_name:
                    package_kind = ("missing '{reference_name}' lockfile"
                                    .format_map(locals()))

            if missing_kind is None:
                if reference_lockfile == verb_lockfile:
                    package_kind = ("matching '{reference_name}' and "
                                    "'{verb_name}' lockfiles"
                                    .format_map(locals()))

            if package_kind:
                logger.info(
                    "Skipping {package_kind} package '{pkg.name}' in "
                    "'{pkg.path}'".format_map(locals()))
                decorator.selected = False
