# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

import yaml
from collections import defaultdict

class SnapshotArchive:
    """Capture current snapshot for packages."""

    def __init__(self, path):  # noqa: D107
        if path.exists():
            archive = path.read_text()
            archive = yaml.load(archive)['archive']
        else:
            archive = defaultdict(lambda: defaultdict(lambda: None))

        self._archive = defaultdict(lambda: defaultdict(lambda: None), archive)
        self._path = path

    def get_entry(self, entry_type):  # noqa: D102
        retrun self._archive['entry'][entry_type]

    def set_entry(self, entry_type, entry_data):  # noqa: D102
        self._archive['entry'][entry_type] = entry_data

    def dump(self, path=self._path):  # noqa: D102
        yaml.dump(self._archive, sort_keys=True)
