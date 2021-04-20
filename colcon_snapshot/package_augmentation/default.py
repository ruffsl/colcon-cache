# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

# from colcon_core.package_augmentation import logger
from colcon_core.package_augmentation import PackageAugmentationExtensionPoint
from colcon_core.package_augmentation import update_descriptor
from colcon_core.plugin_system import satisfies_version

VCS_TYPE = 'default'


class DefaultPackageAugmentation(PackageAugmentationExtensionPoint):
    """
    Augment packages using no version control system.
    """

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            PackageAugmentationExtensionPoint.EXTENSION_POINT_VERSION,
            '^1.0')

    def augment_package(
        self, desc, *, additional_argument_names=None
    ):  # noqa: D102
        # deliberately ignore the package type
        # since this extension can contribute meta information to any package
        data = {
            'vcs_type': VCS_TYPE,
        }
        update_descriptor(desc, data, additional_argument_names=['*'])
