# Copyright 2016-2018 Dirk Thomas
# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

from colcon_cache.event_handler import get_previous_lockfile
from colcon_core.plugin_system import instantiate_extensions
from colcon_core.plugin_system import order_extensions_by_name


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

    def __init__(self, base_path, verb_name, reference_name):  # noqa: D107
        # TODO: find better alternative than perhaps using params
        self.base_path = base_path
        self.verb_name = verb_name
        self.reference_name = reference_name

    def add_arguments(self, *, parser):
        """
        Add command line arguments specific to the workspace base.

        This method must be overridden in a subclass.

        :param parser: The argument parser
        """
        raise NotImplementedError()

    def get_current_lockfile(self, package_build_base):
        """
        Get current lockfile for verb.

        This method can be overridden in a subclass.

        :param package_build_base: Base build path for package
        :returns: A lockfile, or None
        """
        return get_previous_lockfile(package_build_base, self.verb_name)

    def get_reference_lockfile(self, package_build_base):
        """
        Get reference lockfile for verb.

        This method can be overridden in a subclass.

        :param package_build_base: Base build path for package
        :returns: A lockfile, or None
        """
        return get_previous_lockfile(package_build_base, self.reference_name)

    def get_job_lockfile(self, job):
        """
        Get job lockfile for verb.

        This method can be overridden in a subclass.

        :param jobs: The job from `event[1]`
        :returns: A lockfile, or None
        """
        return self.get_reference_lockfile(job.task_context.args.build_base)


def add_verb_handler_arguments(parser):
    """
    Add the command line arguments for the verb handler extensions.

    :param parser: The argument parser
    """
    group = parser.add_argument_group(title='Verb handler arguments')
    extensions = get_verb_handler_extensions()

    for key in sorted(extensions.keys()):
        extension = extensions[key]
        extension.add_arguments(parser=group)


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
