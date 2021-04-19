# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

import pathlib
import yaml

LOCKFILE_FILENAME = 'colcon_{verb_name}.yaml'


class SnapshotLockfile:
    """Capture current snapshot for packages."""

    def __init__(self, path):  # noqa: D107
        if path.exists():
            lockdata = path.read_text()
            lockdata = yaml.safe_load(lockdata)
        else:
            lockdata = {'entry': {}}

        self._lockdata = lockdata
        self._path = path

    def get_entry(self, entry_type):  # noqa: D102
        if entry_type in self._lockdata['entry']:
            return self._lockdata['entry'][entry_type]
        else:
            return {'current_checksum': None,
                    'reference_checksum': None}

    def set_entry(self, entry_type, entry_data):  # noqa: D102
        self._lockdata['entry'][entry_type] = entry_data

    def dump(self, path=None):  # noqa: D102
        if path is None:
            path = self._path
        path.write_text(yaml.dump(self._lockdata, sort_keys=True))


def get_lockfile_path(package_build_base, verb_name):
    """
    Get the lockfile path of a verb from the package build directory.

    :param str package_build_base: The build directory of a package
    :param str verb_name: The invoked verb name
    :returns: The path for the lockfile
    :rtype: Path
    """
    return pathlib.Path(
        package_build_base,
        'snapshot',
        LOCKFILE_FILENAME.format_map(locals()))
