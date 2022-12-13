# Copyright 2016-2018 Dirk Thomas
# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

from contextlib import suppress
import hashlib
import os
from pathlib import Path

from colcon_cache.cache import CacheLockfile
from colcon_cache.event_handler \
    import get_previous_lockfile, set_lockfile
from colcon_cache.task.lock import get_dependencies_lockfiles
from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.task import TaskExtensionPoint
import dirhash

logger = colcon_logger.getChild(__name__)

ENTRY_TYPE = 'dirhash'
META_ARGS = ['dirhash_ratchet', 'dirhash_reset']


class DirhashLockTask(TaskExtensionPoint):
    """Lock caches of packages via dirhash."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102
        modifier_group = parser.add_mutually_exclusive_group()
        modifier_group.add_argument(
            '--dirhash-ratchet', action='store_true',
            help='Ratchet reference checksum from previous value')
        modifier_group.add_argument(
            '--dirhash-reset', action='store_true',
            help='Reset reference checksum to current value')

        parser.add_argument(
            '--dirhash-algorithm',
            choices=dirhash.algorithms_available,
            default='md5',
            help='Hashing algorithm to use, by default "md5". Always '
            'available: {}. Additionally available on current platform: {}. '
            'Note that the same algorithm may appear multiple times in this '
            'set under different names (thanks to OpenSSL) '
            '[https://docs.python.org/2/library/hashlib.html]'.format(
                sorted(dirhash.algorithms_guaranteed),
                sorted(dirhash.algorithms_available -
                       dirhash.algorithms_guaranteed)
                ),
            metavar=''
        )

        # TODO: Expose this somehow
        # filter_options = parser.add_argument_group(
        #     title='Filtering options',
        #     description='Specify what files and directories to include. '
        #     'All files and directories (including symbolic links) are '
        #     'included by default. The --dirhash-match/--dirhash-ignore '
        #     'arguments allows for selection using glob/wildcard '
        #     '(".gitignore style") path matching. Paths relative to the '
        #     'root `directory` (i.e. excluding the name of the root '
        #     'directory itself) are matched against the provided patterns. '
        #     'For example, to only include python source files, use: '
        #     '`colcon cache lock --dirhash-match "*.py"` '
        #     'or to exclude hidden files and directories use: '
        #     '`colcon cache lock --dirhash-ignore ".*" ".*/"` '
        #     'which is short for '
        #     '`colcon cache lock --dirhash-match "*" "!.*" "!.*/"`. '
        #     'For further details see '
        #     'https://github.com/andhus/dirhash/README.md#filtering'
        # )
        filter_options = parser

        filter_options.add_argument(
            '--dirhash-match',
            nargs='+',
            default=['*'],
            help='One or several patterns for paths to include. NOTE: '
            'patterns with an asterisk must be in quotes ("*") or the '
            'asterisk preceded by an escape character (\\*).',
            metavar=''
        )
        filter_options.add_argument(
            '--dirhash-ignore',
            nargs='+',
            default=['.*'],
            help='One or several patterns for paths to exclude. NOTE: '
            'patterns with an asterisk must be in quotes ("*") or the '
            'asterisk preceded by an escape character (\\*).',
            metavar=''
        )
        filter_options.add_argument(
            '--dirhash-empty-dirs',
            action='store_true',
            default=False,
            help='Include empty directories (containing no files that meet '
            'the matching criteria and no non-empty sub directories).'
        )
        filter_options.add_argument(
            '--dirhash-no-linked-dirs',
            dest='dirhash_linked_dirs',
            action='store_false',
            default=True,
            help='Do not include symbolic links to other directories.'
        )
        filter_options.add_argument(
            '--dirhash-no-linked-files',
            dest='dirhash_linked_files',
            action='store_false',
            default=True,
            help='Do not include symbolic links to files.'
        )

        # TODO: Expose this somehow
        # protocol_options = parser.add_argument_group(
        #     title='Protocol options',
        #     description='Specify what properties of files and directories '
        #     'to include and to allow cyclic links. For further details see '
        #     'https://github.com/andhus/dirhash/DIRHASH_STANDARD.md#protocol'
        # )
        protocol_options = parser
        protocol_options.add_argument(
            '--dirhash-properties',
            nargs='+',
            dest='dirhash_entry_properties',
            default=['data', 'name'],
            help='List of file/directory properties to include in the hash. '
            'Available properties are: {} and at least one of name and data '
            'must be included. Default is [data name] which means that both '
            'the name/paths and content (actual data) of files and '
            'directories will be included.'.format(
                list(dirhash.Protocol.EntryProperties.options)),
            metavar=''
        )
        protocol_options.add_argument(
            '--dirhash-allow-cyclic-links',
            default=False,
            action='store_true',
            help='Allow presence of cyclic links (by hashing the relative '
            'path to the target directory).'
        )

        # TODO: Expose this somehow
        # implementation_options = parser.add_argument_group(
        #     title='Implementation options',
        #     description=''
        # )
        implementation_options = parser
        implementation_options.add_argument(
            '--dirhash-chunk-size',
            default=2**20,
            type=int,
            help='The chunk size (in bytes) for reading of files.'
        )
        implementation_options.add_argument(
            '--dirhash-jobs',
            type=int,
            default=-1,  # TODO make default number of cores?
            help='Number of jobs (parallel processes) to use.'
        )

    async def lock(self, *, additional_hooks=None):  # noqa: D102
        pkg = self.context.pkg
        args = self.context.args

        logger.info(
            "Capturing dirhash cache of package in '{args.path}'"
            .format_map(locals()))

        cache_base = Path(args.cache_base, 'cache')
        cache_base.mkdir(parents=True, exist_ok=True)
        lockfile = get_previous_lockfile(args.cache_base, 'cache')
        if lockfile is None:
            lockfile = CacheLockfile(lock_type=ENTRY_TYPE)
        assert lockfile.lock_type == ENTRY_TYPE

        dep_lockfiles = \
            get_dependencies_lockfiles(args, self.context.dependencies)
        lockfile.update_dependencies(dep_lockfiles)

        if args.dirhash_ratchet:
            lockfile.checksums.reference = \
                lockfile.checksums.current

        self.compute_current_checksum(args, lockfile)

        if args.dirhash_reset:
            lockfile.checksums.reference = \
                lockfile.checksums.current

        pkg.metadata['lockfile'] = lockfile
        set_lockfile(args.cache_base, 'cache', lockfile)

        return 0

    def compute_current_checksum(self, args, lockfile):  # noqa: D102

        if args.dirhash_jobs < 0:
            # Use the number of CPU cores
            jobs = os.cpu_count()
            with suppress(AttributeError):
                # consider restricted set of CPUs if applicable
                jobs = min(jobs, len(os.sched_getaffinity(0)))
            # if the number of cores can't be determined
            jobs = max(filter(None.__ne__, [1, jobs]))
            args.dirhash_jobs = jobs

        kwargs = vars(args).copy()
        kwargs['dirhash_directory'] = args.path
        for k in META_ARGS:
            kwargs.pop(k)

        for key in list(kwargs.keys()):
            if key.startswith('dirhash_'):
                kwargs[key[len('dirhash_'):]] = kwargs.pop(key)
            else:
                kwargs.pop(key)
        dir_hash = dirhash.dirhash(**kwargs)

        h = hashlib.md5()
        for _, checksum in lockfile.dependencies.items():
            h.update(bytes.fromhex(checksum))

        h.update(bytes.fromhex(dir_hash))
        lockfile.checksums.current = h.hexdigest()
