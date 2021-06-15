# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

import os
from pathlib import Path
import shutil
from tempfile import mkdtemp

from colcon_core.command import main
from git import Repo


def test_main():
    ws_base = Path(mkdtemp(prefix='test_colcon_'))
    resources_base = Path('test', 'resources').absolute()
    shutil.copytree(resources_base / 'test_src', ws_base / 'src')

    os.chdir(ws_base)
    argv = []

    try:
        main(argv=argv + ['cache', 'lock', '--ignore-dependencies'])
        main(argv=argv + ['cache', 'lock', '--dirhash-ratchet'])
        main(argv=argv + ['cache', 'lock', '--dirhash-reset'])
        main(argv=argv + ['cache', 'lock', '--dirhash-jobs=1'])
        main(argv=argv + ['cache', 'lock'])
        main(argv=argv + ['build'])
        main(argv=argv + ['test'])
        main(argv=argv + ['list', '--packages-select-cache-modified'])
        main(argv=argv + ['list', '--packages-select-cache-unmodified'])
        main(argv=argv + ['list', '--packages-select-cache-invalid'])
        main(argv=argv + ['list', '--packages-skip-cache-valid'])
        print('ws_base: ', ws_base)
    finally:
        # the logging subsystem might still have file handles pending
        # therefore only try to delete the temporary directory
        # shutil.rmtree(ws_base, ignore_errors=True)
        pass

    shutil.rmtree(ws_base / 'build')
    shutil.rmtree(ws_base / 'install')
    shutil.rmtree(ws_base / 'log')

    repo = Repo.init(ws_base / 'src' / 'test-repo')
    repo.config_writer().set_value('user', 'name', 'foo').release()
    repo.config_writer().set_value('user', 'email', 'bar').release()
    repo.git.add(all=True)
    repo.git.commit(message='initial commit')

    try:
        main(argv=argv + ['cache', 'lock'])
        main(argv=argv + ['build'])
        main(argv=argv + ['test'])
        main(argv=argv + ['list', '--packages-select-cache-modified'])
        main(argv=argv + ['list', '--packages-select-cache-unmodified'])
        main(argv=argv + ['list', '--packages-select-cache-invalid'])
        main(argv=argv + ['list', '--packages-skip-cache-valid'])
        print('ws_base: ', ws_base)
    finally:
        # the logging subsystem might still have file handles pending
        # therefore only try to delete the temporary directory
        # shutil.rmtree(ws_base, ignore_errors=True)
        pass
