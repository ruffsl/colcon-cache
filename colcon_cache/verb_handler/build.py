# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

from colcon_cache.verb_handler import VerbHandlerExtensionPoint
from colcon_core.plugin_system import satisfies_version

VERB_NAME = 'build'
REFERENCE_NAME = 'cache'


class BuildVerbHandler(VerbHandlerExtensionPoint):
    """Determin how lockfiles for the build verb should be handled."""

    def __init__(self):  # noqa: D107
        super().__init__(VERB_NAME, REFERENCE_NAME)
        satisfies_version(
            VerbHandlerExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')
