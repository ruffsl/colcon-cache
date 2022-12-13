# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

from colcon_cache.package_selection.valid import ValidPackageSelection
# from colcon_core.package_selection import logger


class Object(object):
    pass


def test_valid():
    valid_package_selection = ValidPackageSelection()

    decorator = Object()
    decorator.selected = False
    decorators = [decorator]

    # TODO: check event log for warning by mocking logger
    args = Object()
    args.packages_select_cache_key = 'cache'
    args.packages_select_cache_invalid = True
    args.packages_skip_cache_valid = False
    valid_package_selection.select_packages(args, decorators)

    args = Object()
    args.packages_select_cache_key = 'cache'
    args.packages_select_cache_invalid = False
    args.packages_skip_cache_valid = True
    valid_package_selection.select_packages(args, decorators)

    args.build_base = 'foo'
    args.packages_select_cache_key = 'cache'
    valid_package_selection.select_packages(args, decorators)

    args.packages_select_cache_key = 'foo'
    valid_package_selection.select_packages(args, decorators)
