# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

from tempfile import TemporaryDirectory

from colcon_cache.package_augmentation.cache_dirhash \
    import DirhashPackageAugmentation
from colcon_core.package_descriptor import PackageDescriptor


def test_augmentation():
    augmentation_extension = DirhashPackageAugmentation()

    with TemporaryDirectory(prefix='test_colcon_') as basepath:
        desc = PackageDescriptor(basepath)
        assert not desc.metadata

        augmentation_extension.augment_package(desc)
        assert desc.metadata['vcs_type'] == 'dirhash'


def test_no_augmentation():
    augmentation_extension = DirhashPackageAugmentation()

    with TemporaryDirectory(prefix='test_colcon_') as basepath:
        desc = PackageDescriptor(basepath)
        desc.metadata['vcs_type'] = 'default'

        augmentation_extension.augment_package(desc)
        assert desc.metadata['vcs_type'] == 'default'
