# Copyright 2016-2018 Dirk Thomas
# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

import hashlib
from pathlib import Path

from colcon_cache.cache import CacheLockfile
from colcon_cache.event_handler \
    import get_previous_lockfile, set_lockfile
from colcon_cache.task.lock import get_dependencies_lockfiles
from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.task import TaskExtensionPoint
from git import Repo

logger = colcon_logger.getChild(__name__)

ENTRY_TYPE = 'git'


class GitLockTask(TaskExtensionPoint):
    """Lock caches of packages via git."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102
        parser.add_argument(
            '--git-diff-filter',
            default='ACDMR',
            help=(
                'Select only files that are Added (A), Copied (C), '
                'Deleted (D), Modified (M), Renamed (R), have their type '
                '(i.e. regular file, symlink, submodule, …​) changed (T), '
                'are Unmerged (U), are Unknown (X), or have had their '
                'pairing Broken (B). Any combination of the filter '
                'characters (including none) can be used. When * '
                '(All-or-none) is added to the combination, all paths are '
                'selected if there is any file that matches other criteria '
                'in the comparison; if there is no file that matches other '
                'criteria, nothing is selected. Also, these upper-case '
                'letters can be downcased to exclude. View docs for info: '
                'https://git-scm.com/docs/git-diff#Documentation/git-diff.txt'
                '---diff-filterACDMRTUXB82308203'
                )
            )

        parser.add_argument(
            '--git-reference-revision',
            type=str, default=None,
            help=(
                'Optional choose revision used as a reference. If unset, '
                'the reference from the previous lockfile will be reused. '
                'If nether provide references, revision defaults to HEAD.'
                ' View docs for info: https://git-scm.com/docs/gitrevisions'
                )
            )

    async def lock(self, *, additional_hooks=None):  # noqa: D102
        pkg = self.context.pkg
        args = self.context.args

        logger.info(
            "Capturing git cache of package in '{args.path}'"
            .format_map(locals()))

        cache_base = Path(args.build_base, 'cache')
        cache_base.mkdir(parents=True, exist_ok=True)
        lockfile = get_previous_lockfile(args.build_base, 'cache')
        if lockfile is None:
            lockfile = CacheLockfile(lock_type=ENTRY_TYPE)
        assert lockfile.lock_type == ENTRY_TYPE

        dep_lockfiles = \
            get_dependencies_lockfiles(args, self.context.dependencies)
        lockfile.update_dependencies(dep_lockfiles)

        if args.git_reference_revision is None:
            args.git_reference_revision = \
                lockfile.metadata.get('reference_revision', None)

        self.compute_current_checksums(args, lockfile)

        pkg.metadata['lockfile'] = lockfile
        set_lockfile(args.build_base, 'cache', lockfile)

        return 0

    def compute_current_checksums(self, args, lockfile):  # noqa: D102
        repo = Repo(args.path, search_parent_directories=True)

        if args.git_reference_revision is None:
            reference_commit = repo.commit()
        else:
            reference_commit = repo.commit(args.git_reference_revision)
        lockfile.metadata['reference_revision'] = reference_commit.hexsha

        diff_args = []
        diff_args.append('--diff-filter={}'.format(args.git_diff_filter))
        diff_args.append(reference_commit.hexsha)
        diff_args.append(args.path)
        diff = repo.git.diff(diff_args)

        h = hashlib.sha1()
        for _, checksum in lockfile.dependencies.items():
            h.update(bytes.fromhex(checksum))

        h.update(bytes.fromhex(reference_commit.hexsha))
        lockfile.checksums.reference = h.hexdigest()

        if diff:
            h.update(diff.encode('utf-8'))
        lockfile.checksums.current = h.hexdigest()