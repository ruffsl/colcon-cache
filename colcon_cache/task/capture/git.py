# Copyright 2016-2018 Dirk Thomas
# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

from pathlib import Path

from colcon_cache.cache import CacheLockfile
from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.task import TaskExtensionPoint
from git import Repo

logger = colcon_logger.getChild(__name__)

ENTRY_TYPE = 'git'


class GitCaptureTask(TaskExtensionPoint):
    """Capture caches of packages via git."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    async def capture(self, *, additional_hooks=None):  # noqa: D102
        pkg = self.context.pkg
        args = self.context.args

        logger.info(
            "Capturing git cache of package in '{args.path}'"
            .format_map(locals()))

        cache_base = Path(args.build_base, 'cache')
        cache_base.mkdir(parents=True, exist_ok=True)
        capture_cache_path = Path(
            cache_base, 'colcon_cache.yaml')
        capture_cache = CacheLockfile(capture_cache_path)

        entry_data = capture_cache.get_entry(ENTRY_TYPE)
        entry_data['reference_checksum'] = entry_data['current_checksum']
        entry_data['current_checksum'] = self.compute_current_checksum(args)
        capture_cache.set_entry(ENTRY_TYPE, entry_data)
        pkg.metadata['lockfile'] = capture_cache

        return 0

    def compute_current_checksum(self, args):  # noqa: D102
        repo = Repo(args.path, search_parent_directories=True)
        # t = repo.head.commit.tree
        # repo.git.diff(paths=args.path)

        return self.latest_commit_sha(repo, args.path)

    def latest_commit_sha(self, repo, path):  # noqa: D102
        log_message = repo.git.log('-1', path)
        commit_sha = log_message.split('\n')[0].split(' ')[1]
        return commit_sha
