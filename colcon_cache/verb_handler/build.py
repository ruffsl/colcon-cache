# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

from colcon_cache.verb_handler import VerbHandlerExtensionPoint
from colcon_core.plugin_system import satisfies_version

BASE_PATH = 'build'
VERB_NAME = 'build'
REFERENCE_NAME = 'cache'


class BuildVerbHandler(VerbHandlerExtensionPoint):
    """Determin how lockfiles for the build verb should be handled."""

    def __init__(self):  # noqa: D107
        super().__init__(BASE_PATH, VERB_NAME, REFERENCE_NAME)
        satisfies_version(
            VerbHandlerExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102
        parser.add_argument(
            '--build-base',
            default=self.base_path,
            help='The base path for all build directories '
                 '(default: {self.base_path})'.format_map(locals()))
