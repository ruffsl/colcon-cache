# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

from colcon_cache.package_selection.modified import ModifiedPackageSelection
# from colcon_core.package_selection import logger


class Object(object):
    pass


def test_modified():
    modified_package_selection = ModifiedPackageSelection()

    decorator = Object()
    decorator.selected = False
    decorators = [decorator]

    # TODO: check event log for warning by mocking logger
    args = Object()
    args.packages_select_cache_modified = True
    args.packages_select_cache_unmodified = False
    modified_package_selection.select_packages(args, decorators)

    args = Object()
    args.packages_select_cache_modified = False
    args.packages_select_cache_unmodified = True
    modified_package_selection.select_packages(args, decorators)

    args.build_base = 'foo'
    args.packages_select_cache_key = 'cache'
    modified_package_selection.select_packages(args, decorators)

    args.packages_select_cache_key = 'foo'
    modified_package_selection.select_packages(args, decorators)
