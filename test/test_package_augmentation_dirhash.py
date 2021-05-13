# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

from tempfile import TemporaryDirectory

from colcon_cache.package_augmentation.git \
    import GitPackageAugmentation
from colcon_core.package_descriptor import PackageDescriptor
from git import Repo


def test_augmentation():
    augmentation_extension = GitPackageAugmentation()

    with TemporaryDirectory(prefix='test_colcon_') as basepath:
        repo = Repo.init(basepath)
        assert repo

        desc = PackageDescriptor(basepath)
        assert not desc.metadata

        augmentation_extension.augment_package(desc)
        assert desc.metadata['vcs_type'] == 'git'


def test_no_augmentation():
    augmentation_extension = GitPackageAugmentation()

    with TemporaryDirectory(prefix='test_colcon_') as basepath:
        desc = PackageDescriptor(basepath)
        assert not desc.metadata

        augmentation_extension.augment_package(desc)
        assert 'vcs_type' not in desc.metadata
