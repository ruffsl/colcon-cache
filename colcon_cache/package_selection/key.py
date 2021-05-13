# Copyright 2019 Dirk Thomas
# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

from colcon_cache.verb_handler import get_verb_handler_extensions
from colcon_core.package_selection import PackageSelectionExtensionPoint
from colcon_core.plugin_system import satisfies_version


class KeyPackageSelection(PackageSelectionExtensionPoint):
    """Skip a set of packages based on lockfiles from previous caches."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            PackageSelectionExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102
        parser.add_argument(
            '--packages-select-cache-key',
            choices=get_verb_handler_extensions().keys(),
            default=None,
            help='Only process packages using considered cache key. '
                 'Fallbacks using invoked verb handler if unspecified.')

    def select_packages(self, args, decorators):  # noqa: D102
        # Pass given added arguments here are considered elsewhere by other
        # package selection extensions in colcon_cache
        pass
