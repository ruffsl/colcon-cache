# Copyright 2016-2018 Dirk Thomas
# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

# from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import instantiate_extensions
from colcon_core.plugin_system import order_extensions_by_name

# logger = colcon_logger.getChild(__name__)


class VerbHandlerExtensionPoint:
    """
    The interface for verb handler extensions.

    A verb handler extension determines how a verb
    should cache the subset of packages to be processed.

    For each instance the attribute `VERB_HANDLER_NAME` is being set to
    the basename of the entry point registering the extension.
    """

    """The version of the package selection extension interface."""
    EXTENSION_POINT_VERSION = '1.0'

    def __init__(self, verb_name, reference_name):  # noqa: D107
        # TODO: find better alternative than perhaps using params
        self.verb_name = verb_name
        self.reference_name = reference_name

    def get_current_lockfile(self, package_build_base):
        """
        Get current lockfile for verb.

        This method must be overridden in a subclass.

        :param package_build_base: Base build path for package
        :returns: A lockfile, or None
        """
        raise NotImplementedError()

    def get_reference_lockfile(self, package_build_base):
        """
        Get reference lockfile for verb.

        This method must be overridden in a subclass.

        :param package_build_base: Base build path for package
        :returns: A lockfile, or None
        """
        raise NotImplementedError()

    def get_job_lockfile(self, job):
        """
        Get job lockfile for verb.

        This method can be overridden in a subclass.

        :param jobs: The job from `event[1]`
        :returns: A lockfile, or None
        """
        return self.get_reference_lockfile(job.task_context.args.build_base)


def get_verb_handler_extensions():
    """
    Get the available verb handler extensions.

    The extensions are ordered by their entry point name.

    :rtype: OrderedDict
    """
    extensions = instantiate_extensions(__name__)
    for name, extension in extensions.items():
        extension.VERB_HANDLER_NAME = name
    return order_extensions_by_name(extensions)
