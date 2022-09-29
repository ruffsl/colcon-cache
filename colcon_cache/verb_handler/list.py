# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

from colcon_cache.verb_handler import VerbHandlerExtensionPoint
from colcon_core.plugin_system import satisfies_version

BASE_PATH = None
VERB_NAME = None
REFERENCE_NAME = None


class ListVerbHandler(VerbHandlerExtensionPoint):
    """Determin how lockfiles for the list verb should be handled."""

    def __init__(self):  # noqa: D107
        super().__init__(BASE_PATH, VERB_NAME, REFERENCE_NAME)
        satisfies_version(
            VerbHandlerExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102
        pass

    def get_reference_lockfile(self, package_build_base):  # noqa: D102
        return None

    def get_job_lockfile(self, job):  # noqa: D102
        return None
