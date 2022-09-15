# colcon-cache

[![GitHub Workflow Status](https://github.com/ruffsl/colcon-cache/actions/workflows/ci.yaml/badge.svg?branch=master&event=push)](https://github.com/ruffsl/colcon-cache/actions/workflows/ci.yaml?query=branch%3Amaster+event%3Apush)
[![Codecov](https://codecov.io/gh/ruffsl/colcon-cache/branch/master/graph/badge.svg)](https://codecov.io/gh/ruffsl/colcon-cache)

An extension for [colcon-core](https://github.com/colcon/colcon-core) to cache the processing of packages. Enables caching of various colcon tasks, such as building or testing packages, by associating successful jobs with the respective state of package source files. In conjunction with [colcon-package-selection](https://github.com/colcon/colcon-package-selection), this extension can accelerate developer or continuous integration workflows by allowing users to finely cache valid workspace artifacts and skip processing of unaltered or unaffected packages during software development. For example, when pulling various changes into a local workspace to review pull requests, this extension can be used to track which packages need to be rebuilt or retested, maximizing the caching of the existing artifacts.

The extension works by generating lockfiles that incorporate the respective state of package source files, either directly via hashing source directories or indirectly via detected revision control. Upon successful task completion for a package job, as when evoking colcon verbs like build, test, etc, these lockfiles are updated for the evoked verb, thereby delineating the provenance of the job’s results. For package selection, these lockfiles are then used to assess whether a verb’s cached outcome for a package remains relevant or valid.


## Quick start

Setup an example colcon workspace:
```
mkdir -p ~/ws/src && cd ~/ws
wget https://raw.githubusercontent.com/colcon/colcon.readthedocs.org/main/colcon.repos
vcs import src < colcon.repos
```

First lock cache, then build and test workspace:
```
colcon cache lock
colcon build
colcon test
```

Modify workspace packages, then update cache lockfiles:
```
echo "#foo" >> src/colcon-cmake/setup.py
echo "#bar" >> src/colcon-package-information/setup.py
colcon cache lock
```

Rebuild and retest, skipping packages with valid cache:
```
colcon build --packages-skip-cache-valid
colcon test --packages-skip-cache-valid
```


## Subverbs

### `lock` - Lock Package Cache

The `lock` subverb generates or updates lockfiles for selected packages by capturing the current state of package source files. The subverb provides basic arguments to change the build base path where lockfiles are recorded, as well as the option to ignore dependencies when capturing package state. More advance arguments specific `lock` tasks used to capture the package state include:

- `--build-base`
  - The base path for all build directories (default: build)
- `--ignore-dependencies`
  - Ignore dependencies when capturing caches (default: false)


## Package Selection

This extension provides additional package selection arguments that can filter by modified package source or by validity of workspace cache with respect to the most recent invocation of the `lock` subverb. By default, the  internal cache key is selected by the colcon verb that invokes the package selection argument, but can be manually overridden:

- `--packages-select-cache-key`
  - Only process packages using considered cache key. Fallbacks using invoked verb handler if unspecified.

### Modified Package Selection

Check if the `current` checksum in a package's lockfile matches it's `reference` checksum.

- `--packages-select-cache-modified`
  - Only process a subset of packages whose cache denote package modifications (packages without lockfiles are not considered as modified)
- `--packages-select-cache-unmodified`
  - Only process a subset of packages whose cache denote no package modifications (packages without lockfiles are not considered as unmodified)

### Valid Package Selection

Check if the `current` checksum in the verb's lockfile matches that in the package's lockfile.

- `--packages-select-cache-invalid`
  - Only process a subset of packages with an invalid cache (packages without a reference cache are not considered)
- `--packages-skip-cache-valid`
  - Skip a set of packages with a valid cache (packages without a reference cache are not considered)


## Extension points

This extension makes use of a number of colcon-core extension points for registering verbs, subverbs, and package selection arguments with colcon CLI, an event handler to update lockfiles for successful jobs, as well auto detect revision control with package augmentation. This extension also provides a number of it's own extension points for additional support of alternative revision control systems or package caching strategies.

### `VerbExtensionPoint`

This extension point determines how lockfiles are propagated for jobs invoked by a given verb. As tasks may or may not require prerequisite processing, this extension point provides the means to express the relational provenance of cached artifacts generated when using colcon. Default verb extensions provided include:

- `cache`
  - Do not propagate lockfile, as `lock` subverb handles this explicitly
- `list`
  - Do not propagate lockfile, using `cache` lockfile as a reference
- `build`
  - Propagate lockfile, using `cache` lockfile as a reference
- `test`
  - Propagate lockfile, using `build` lockfile as a reference

### `PackageAugmentationExtensionPoint`

This extension point determines if or what revision control is in effect for package source files, modifying the package's metadata to use the appropriate task extension for the `lock` subverb.

- `DirhashPackageAugmentation`
  - If no revision control is detected, the default dirhash extension is configured.
- `GitPackageAugmentation`
  - If git revision control is detected via a git repo, the git extension is configured.

### `TaskExtensionPoint`

This extension point determines how lockfiles are derived, given the package's detected revision control.

#### `DirhashLockTask`

This extension derives the lockfile by computing the hash of package source files using [Dirhash](https://github.com/andhus/dirhash-python). While most Dirhash options are exposed, such as customizing match and ignore expressions to include dot file paths (ignored by default), several specific arguments provide control in updating the `reference` checksum for a package's lockfile.

- `--dirhash-ratchet`
  - Ratchet reference checksum from previous value
- `--dirhash-reset`
  - Reset reference checksum to current value

```
Arguments for 'dirhash' packages:
  --dirhash-ratchet     Ratchet reference checksum from previous value
  --dirhash-reset       Reset reference checksum to current value
  --dirhash-algorithm   Hashing algorithm to use, by default "md5". Always
                        available: ['md5', 'sha1', 'sha224', 'sha256',
                        'sha384', 'sha512']. Additionally available on current
                        platform: ['blake2b', 'blake2s', 'md4', 'md5-sha1',
                        'ripemd160', 'sha3_224', 'sha3_256', 'sha3_384',
                        'sha3_512', 'sha512_224', 'sha512_256', 'shake_128',
                        'shake_256', 'sm3', 'whirlpool']. Note that the same
                        algorithm may appear multiple times in this set under
                        different names (thanks to OpenSSL)
                        [https://docs.python.org/2/library/hashlib.html]
  --dirhash-match  [ ...]
                        One or several patterns for paths to include. NOTE:
                        patterns with an asterisk must be in quotes ("*") or
                        the asterisk preceded by an escape character (\*).
  --dirhash-ignore  [ ...]
                        One or several patterns for paths to exclude. NOTE:
                        patterns with an asterisk must be in quotes ("*") or
                        the asterisk preceded by an escape character (\*).
  --dirhash-empty-dirs  Include empty directories (containing no files that
                        meet the matching criteria and no non-empty sub
                        directories).
  --dirhash-no-linked-dirs
                        Do not include symbolic links to other directories.
  --dirhash-no-linked-files
                        Do not include symbolic links to files.
  --dirhash-properties  [ ...]
                        List of file/directory properties to include in the
                        hash. Available properties are: ['is_link', 'data',
                        'name'] and at least one of name and data must be
                        included. Default is [data name] which means that both
                        the name/paths and content (actual data) of files and
                        directories will be included.
  --dirhash-allow-cyclic-links
                        Allow presence of cyclic links (by hashing the
                        relative path to the target directory).
  --dirhash-chunk-size DIRHASH_CHUNK_SIZE
                        The chunk size (in bytes) for reading of files.
  --dirhash-jobs DIRHASH_JOBS
                        Number of jobs (parallel processes) to use.
```

#### `GitLockTask`

This extension derives the lockfile by computing the hash of tracked source files using [Git](https://git-scm.com). Several specific arguments provide control in specifying the reference revision and fallback used for diffing the package source file when computing the `current` hash for a package lockfile. This not only enables tracking of package source files with respect to the most recent invocation of the `lock` subverb, but also with respect to a particular git branch, tag or commit. The default match criteria for the diff filter comparison can also be overridden.

```
Arguments for 'git' packages:
  --git-diff-filter GIT_DIFF_FILTER
                        Select only files that are Added (A), Copied (C),
                        Deleted (D), Modified (M), Renamed (R), have their
                        type (i.e. regular file, symlink, submodule, …​)
                        changed (T), are Unmerged (U), are Unknown (X), or
                        have had their pairing Broken (B). Any combination of
                        the filter characters (including none) can be used.
                        When * (All-or-none) is added to the combination, all
                        paths are selected if there is any file that matches
                        other criteria in the comparison; if there is no file
                        that matches other criteria, nothing is selected.
                        Also, these upper-case letters can be downcased to
                        exclude. View docs for info:
                        https://git-scm.com/docs/git-diff#Documentation/
                        git-diff.txt---diff-filterACDMRTUXB82308203
  --git-reference-revision GIT_REFERENCE_REVISION
                        Optionally specify revision used as a reference. If
                        unset, the reference from the previous lockfile will
                        be reused. If nether provide references, the fallback
                        will be used. View docs for info:
                        https://git-scm.com/docs/gitrevisions
  --git-reference-fallback GIT_REFERENCE_FALLBACK
                        Override fallback revision used as a reference. If
                        nether the user and the previous lockfile specify a
                        revision, or if reference is unresolvable, this
                        fallback will be used. View docs for info:
                        https://git-scm.com/docs/gitrevisions
```
