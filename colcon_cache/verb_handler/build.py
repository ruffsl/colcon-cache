# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

from colcon_core.plugin_system import satisfies_version
from colcon_cache.event_handler \
    import get_previous_lockfile
from colcon_cache.verb_handler \
    import VerbHandlerExtensionPoint

VERB_NAME = 'build'
REFERENCE_NAME = 'cache'


class BuildVerbHandler(VerbHandlerExtensionPoint):
    """Determin how lockfiles for the build verb should be handled."""

    def __init__(self):  # noqa: D107
        super().__init__(VERB_NAME, REFERENCE_NAME)
        satisfies_version(
            VerbHandlerExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def get_current_lockfile(self, package_build_base):  # noqa: D102
        return get_previous_lockfile(package_build_base, self.verb_name)

    def get_reference_lockfile(self, package_build_base):  # noqa: D102
        return get_previous_lockfile(package_build_base, self.reference_name)
