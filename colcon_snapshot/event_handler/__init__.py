# Copyright 2019 Dirk Thomas
# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

from colcon_snapshot.snapshot import SnapshotLockfile
from colcon_snapshot.snapshot import get_lockfile_path


def get_previous_lockfile(package_build_base, verb_name):
    """
    Get the lockfile of a verb from the package build directory.

    :param str package_build_base: The build directory of a package
    :param str verb_name: The invoked verb name
    :returns: The previously persisted lockfile, otherwise None
    :rtype: SnapshotLockfile
    """
    path = get_lockfile_path(package_build_base, verb_name)
    if not path.exists():
        return None
    return SnapshotLockfile(path)


def set_lockfile(package_build_base, verb_name, lockfile):
    """
    Persist the lockfile of a verb in the package build directory.

    :param str package_build_base: The build directory of a package
    :param str verb_name: The invoked verb name
    :param SnapshotLockfile lockfile: The lockfile of the invocation
    """
    path = get_lockfile_path(package_build_base, verb_name)
    path.parent.mkdir(parents=True, exist_ok=True)
    lockfile.dump(path)
