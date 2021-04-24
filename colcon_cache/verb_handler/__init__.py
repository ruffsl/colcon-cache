# Copyright 2016-2018 Dirk Thomas
# Licensed under the Apache License, Version 2.0

from collections import defaultdict
import traceback

from colcon_core.logging import colcon_logger
from colcon_core.package_augmentation import augment_packages
from colcon_core.package_discovery import add_package_discovery_arguments
from colcon_core.package_discovery import discover_packages
from colcon_core.package_identification \
    import get_package_identification_extensions
from colcon_core.plugin_system import instantiate_extensions
from colcon_core.plugin_system import order_extensions_by_name
from colcon_core.topological_order import topological_order_packages

logger = colcon_logger.getChild(__name__)


class PackageSelectionExtensionPoint:
    """
    The interface for package selection extensions.

    A package selection extension determines the subset of packages to be
    processed.

    For each instance the attribute `PACKAGE_SELECTION_NAME` is being set to
    the basename of the entry point registering the extension.
    """

    """The version of the package selection extension interface."""
    EXTENSION_POINT_VERSION = '1.0'

    def add_arguments(self, *, parser):
        """
        Add command line arguments specific to the package selection.

        The method is intended to be overridden in a subclass.

        :param parser: The argument parser
        """
        pass

    def check_parameters(self, *, args, pkg_names):
        """
        Check is the passed arguments have valid values.

        The method is intended to be overridden in a subclass.
        It should either warn about invalid values and gracefully continue or
        raise a `SystemExit` exception.

        :param args: The parsed command line arguments
        :param pkg_names: The set of package names
        """
        pass

    def select_packages(self, *, args, decorators):
        """
        Identify the packages which should be skipped.

        By default all package decorators are marked as "selected".

        This method must be overridden in a subclass.

        :param args: The parsed command line arguments
        :param list decorators: The package decorators in topological order
        """
        raise NotImplementedError()


def get_package_selection_extensions():
    """
    Get the available package selection extensions.

    The extensions are ordered by their entry point name.

    :rtype: OrderedDict
    """
    extensions = instantiate_extensions(__name__)
    for name, extension in extensions.items():
        extension.PACKAGE_SELECTION_NAME = name
    return order_extensions_by_name(extensions)
