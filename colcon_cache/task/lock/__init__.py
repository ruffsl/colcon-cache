# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

from collections import OrderedDict

from colcon_cache.event_handler import get_previous_lockfile


def get_dependencies_lockfiles(args, dependencies):  # noqa: D103
    dependencies_checksums = OrderedDict()
    if not args.ignore_dependencies:
        for dep_name, dep_path in dependencies.items():
            lockfile = get_previous_lockfile(
                package_build_base=dep_path,
                verb_name='cache')
            dependencies_checksums[dep_name] = lockfile
    return dependencies_checksums
