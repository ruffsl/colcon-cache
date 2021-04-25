# Copyright 2016-2018 Dirk Thomas
# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

import hashlib

from colcon_cache.cache import CacheLockfile, get_lockfile_path
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

    async def capture(self, *, additional_hooks=None):  # noqa: D102
        pkg = self.context.pkg
        args = self.context.args

        logger.info(
            "Capturing git cache of package in '{args.path}'"
            .format_map(locals()))

        # Get current lockfile
        lockfile_path = get_lockfile_path(args.build_base, 'cache')
        lockfile = CacheLockfile(lockfile_path)
        entry_data = lockfile.get_entry(ENTRY_TYPE)

        if args.git_reference_revision is None:
            args.git_reference_revision = \
                entry_data['reference_checksum']

        entry_data['reference_checksum'], \
            entry_data['current_checksum'] = \
            self.compute_current_checksums(args)

        lockfile.set_entry(ENTRY_TYPE, entry_data)
        pkg.metadata['lockfile'] = lockfile

        return 0

    def compute_current_checksums(self, args):  # noqa: D102
        repo = Repo(args.path, search_parent_directories=True)

        if args.git_reference_revision is None:
            reference_commit = repo.commit()
        else:
            reference_commit = repo.commit(args.git_reference_revision)

        diff_args = []
        diff_args.append('--diff-filter={}'.format(args.git_diff_filter))
        diff_args.append(reference_commit.hexsha)
        diff_args.append(args.path)
        diff = repo.git.diff_index(diff_args)

        if not diff:
            return reference_commit.hexsha, reference_commit.hexsha

        h = hashlib.sha1()
        h.update(bytes.fromhex(reference_commit.hexsha))
        h.update(diff.encode('utf-8'))
        return reference_commit.hexsha, h.hexdigest()
