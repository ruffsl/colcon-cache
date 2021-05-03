# Copyright 2019 Rover Robotics
# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

import asyncio
from contextlib import suppress
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace

from colcon_cache.subverb.capture import \
    CaptureCachePackageArguments
from colcon_cache.task.capture.dirhash import DirhashCaptureTask
from colcon_core.package_descriptor import PackageDescriptor
from colcon_core.plugin_system import SkipExtensionException
import colcon_core.shell
from colcon_core.shell.bat import BatShell
from colcon_core.shell.sh import ShShell
from colcon_core.subprocess import new_event_loop
from colcon_core.task import TaskContext
import pytest


@pytest.fixture(autouse=True)
def monkey_patch_get_shell_extensions(monkeypatch):
    a_shell = None
    for shell_extension_class in [ShShell, BatShell]:
        with suppress(SkipExtensionException):
            a_shell = shell_extension_class()
            break

    if a_shell is None:
        pytest.fail('No valid shell extension found.')

    monkeypatch.setattr(
        colcon_core.shell,
        'get_shell_extensions',
        lambda: {
            200: {'mock': a_shell}
        }
    )


@pytest.fixture(autouse=True)
def monkey_patch_put_event_into_queue(monkeypatch):
    monkeypatch.setattr(
        TaskContext, 'put_event_into_queue', lambda *args: None
    )


@pytest.mark.skip(reason='WIP: fix package_args to include dirhash args')
def test_cache_package():
    event_loop = new_event_loop()
    asyncio.set_event_loop(event_loop)
    try:
        with TemporaryDirectory(prefix='test_colcon_') as tmp_path_str:
            tmp_path = Path(tmp_path_str)
            dirhash_capture_task = DirhashCaptureTask()
            package = PackageDescriptor(tmp_path / 'src')
            package.name = 'test_package'
            package.type = 'python'

            additional_destinations = [
                'dirhash_algorithm',
                'dirhash_match',
                'dirhash_ignore',
                'dirhash_empty_dirs',
                'dirhash_linked_dirs',
                'dirhash_linked_files',
                'dirhash_entry_properties',
                'dirhash_allow_cyclic_links',
                'dirhash_chunk_size',
                'dirhash_jobs',
                'dirhash_ratchet',
                'dirhash_reset',
                'git_diff_filter',
                'git_reference_revision']
            args = SimpleNamespace(
                    path=str(tmp_path / 'src'),
                    build_base=str(tmp_path / 'build'),
                    ignore_dependencies=False,
                )
            package_args = CaptureCachePackageArguments(
                package, args, additional_destinations=additional_destinations)

            context = TaskContext(
                pkg=package,
                args=package_args,
                dependencies={}
            )
            dirhash_capture_task.set_context(context=context)
            pkg = dirhash_capture_task.context.pkg

            pkg.path.mkdir()
            (pkg.path / 'setup.py').write_text(
                'from setuptools import setup\n'
                'setup(\n'
                '    name="test_package",\n'
                '    packages=["my_module"],\n'
                ')\n'
            )
            (pkg.path / 'my_module').mkdir()
            (pkg.path / 'my_module' / '__init__.py').touch()

            rc = event_loop.run_until_complete(dirhash_capture_task.capture())
            assert not rc

            build_base = Path(dirhash_capture_task.context.args.build_base)
            assert Path(build_base, 'cache/colcon_capture.yaml').exists()

    finally:
        event_loop.close()
