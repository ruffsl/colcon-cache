# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

import os
from pathlib import Path
import shutil
from tempfile import mkdtemp

from colcon_core.command import main


def test_main():
    ws_base = Path(mkdtemp(prefix='test_colcon_'))
    resources_base = Path('test', 'resources').absolute()
    shutil.copytree(resources_base / 'test_src', ws_base / 'src')

    os.chdir(ws_base)
    argv = []

    try:
        main(argv=argv + ['cache', 'lock'])
        main(argv=argv + ['build'])
        main(argv=argv + ['test'])
        print('ws_base: ', ws_base)
    finally:
        # the logging subsystem might still have file handles pending
        # therefore only try to delete the temporary directory
        # shutil.rmtree(ws_base, ignore_errors=True)
        pass
