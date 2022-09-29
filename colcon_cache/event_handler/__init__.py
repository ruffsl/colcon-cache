# Copyright 2019 Dirk Thomas
# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

from colcon_cache.cache import CacheLockfile
from colcon_cache.cache import get_lockfile_path


def get_previous_lockfile(package_base_path, verb_name):
    """
    Get the lockfile of a verb from the package build directory.

    :param str package_base_path: The build directory of a package
    :param str verb_name: The invoked verb name
    :returns: The previously persisted lockfile, otherwise None
    :rtype: CacheLockfile
    """
    path = get_lockfile_path(package_base_path, verb_name)
    if not path.exists():
        return None
    lockfile = CacheLockfile()
    lockfile.load(path)
    return lockfile


def set_lockfile(package_base_path, verb_name, lockfile):
    """
    Persist the lockfile of a verb in the package build directory.

    :param str package_base_path: The build directory of a package
    :param str verb_name: The invoked verb name
    :param CacheLockfile lockfile: The lockfile of the invocation
    """
    path = get_lockfile_path(package_base_path, verb_name)
    path.parent.mkdir(parents=True, exist_ok=True)
    lockfile.dump(path)
