# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

import copy
import pathlib
from tempfile import TemporaryDirectory

from colcon_cache.cache import \
    CacheChecksums, CacheLockfile, get_lockfile_path


def test_cache_lockfile():
    lockfile_a = CacheLockfile()
    lockfile_a.lock_type = 'foo'
    lockfile_a.checksums = CacheChecksums(
        current='123', reference='123')
    lockfile_a.dependencies = {}
    lockfile_a.metadata = {}
    assert lockfile_a != CacheChecksums()
    assert CacheChecksums() != lockfile_a

    lockfile_b = CacheLockfile(
        lock_type=copy.deepcopy(lockfile_a.lock_type),
        checksums=copy.deepcopy(lockfile_a.checksums),
        dependencies={'baz': '789'},
        metadata={'baz': 'spam'}
    )
    lockfile_b.update_dependencies(dep_lockfiles={})
    assert lockfile_a == lockfile_b

    lockfile_b.lock_type = 'bar'
    assert lockfile_a != lockfile_b

    lockfile_b.lock_type = lockfile_a.lock_type
    lockfile_b.checksums.current = '456'
    assert lockfile_a != lockfile_b
    assert lockfile_b.is_modified()

    dep_lockfiles = {
        'foo': lockfile_a,
        'bar': lockfile_b,
    }
    lockfile_a.update_dependencies(dep_lockfiles=dep_lockfiles)
    for dep_name, dep_lock in lockfile_a.dependencies.items():
        assert dep_lockfiles[dep_name].checksums.current == dep_lock

    with TemporaryDirectory(prefix='test_colcon_') as basepath:
        lockfile_a_path = get_lockfile_path(basepath, 'example_1')
        lockfile_a_path.parent.mkdir(parents=True, exist_ok=True)
        lockfile_c_path = pathlib.Path(
            pathlib.Path(__file__).parent.absolute(),
            'cache', 'colcon_cache_example_1.json')
        lockfile_a.dump(lockfile_a_path)
        assert lockfile_a_path.read_text() == lockfile_c_path.read_text()

        lockfile_c = CacheLockfile()
        lockfile_c.load(lockfile_c_path)
        assert lockfile_a.__dict__ == lockfile_c.__dict__

        lockfile_b_path = get_lockfile_path(basepath, 'example_2')
        lockfile_b_path.parent.mkdir(parents=True, exist_ok=True)
        lockfile_d_path = pathlib.Path(
            pathlib.Path(__file__).parent.absolute(),
            'cache', 'colcon_cache_example_2.json')
        lockfile_b.dump(lockfile_b_path)
        assert lockfile_b_path.read_text() == lockfile_d_path.read_text()

        lockfile_d = CacheLockfile()
        lockfile_d.load(lockfile_d_path)
        assert lockfile_b.__dict__ == lockfile_d.__dict__
