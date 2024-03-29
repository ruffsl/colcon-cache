# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

import json
import pathlib

from colcon_core.plugin_system import satisfies_version

LOCKFILE_FILENAME = 'colcon_{verb_name}.json'
LOCKFILE_VERSION = '0.0.1'


class CacheChecksums:
    """Cache checksums for package."""

    def __init__(  # noqa: D107
            self,
            current=None,
            reference=None):
        self.current = current
        self.reference = reference

    def __eq__(self, other):  # noqa: D105
        if not isinstance(other, CacheChecksums):
            return False
        return self.current == other.current

    def is_modified(self):  # noqa: D10s
        return self.current != self.reference


class CacheLockfile:
    """Cache lockfile for package."""

    def __init__(  # noqa: D107
            self,
            lock_type=None,
            checksums=None,
            dependencies=None,
            metadata=None):
        self.version = LOCKFILE_VERSION
        self.lock_type = lock_type

        if checksums:
            assert (isinstance(checksums, CacheChecksums))
            self.checksums = checksums
        else:
            self.checksums = CacheChecksums()

        if dependencies:
            assert (isinstance(dependencies, dict))
            self.dependencies = dependencies
        else:
            self.dependencies = {}

        if metadata:
            assert (isinstance(metadata, dict))
            self.metadata = metadata
        else:
            self.metadata = {}

    def __eq__(self, other):  # noqa: D105
        if not isinstance(other, CacheLockfile):
            return False
        elif self.lock_type != other.lock_type:
            return False
        return self.checksums == other.checksums

    def is_modified(self):  # noqa: D10s
        return self.checksums.is_modified()

    def update_dependencies(self, dep_lockfiles):  # noqa: D10s
        self.dependencies.clear()
        for dep_name, dep_lockfile in dep_lockfiles.items():
            assert isinstance(dep_lockfile, CacheLockfile)
            self.dependencies[dep_name] = \
                dep_lockfile.checksums.current

    def load(self, path):  # noqa: D102
        with open(path, 'r') as f:
            data = json.load(f)
        satisfies_version(data['version'], '^0.0.1')

        self.lock_type = data['lock_type']
        self.checksums = CacheChecksums(**data['checksums'])
        if data['dependencies']:
            self.dependencies = data['dependencies']
        if data['metadata']:
            self.metadata = data['metadata']

    def dump(self, path):  # noqa: D102
        with open(path, 'w') as f:
            json.dump(
                obj=self,
                fp=f,
                indent=2,
                default=lambda o: o.__dict__,
                sort_keys=True)


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
        'cache',
        LOCKFILE_FILENAME.format_map(locals()))
