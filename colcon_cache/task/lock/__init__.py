# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

from collections import OrderedDict

from colcon_cache.event_handler import get_previous_lockfile

_cached_lockfiles = {}


def get_dependencies_lockfiles(args, dependencies):  # noqa: D103
    global _cached_lockfiles
    dependencies_checksums = OrderedDict()
    if not args.ignore_dependencies:
        for dep_name, dep_path in dependencies.items():
            if dep_path in _cached_lockfiles:
                lockfile = _cached_lockfiles[dep_path]
            else:
                lockfile = get_previous_lockfile(
                    package_base_path=dep_path,
                    verb_name='cache')
                _cached_lockfiles[dep_path] = lockfile
            dependencies_checksums[dep_name] = lockfile
    return dependencies_checksums
