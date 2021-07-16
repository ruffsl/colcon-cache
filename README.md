# colcon-cache

[![GitHub Workflow Status](https://github.com/ruffsl/colcon-cache/actions/workflows/test.yml/badge.svg)](https://github.com/ruffsl/colcon-cache/actions/workflows/test.yml)
[![Codecov](https://codecov.io/gh/ruffsl/colcon-cache/branch/master/graph/badge.svg)](https://codecov.io/gh/ruffsl/colcon-cache)

An extension for [colcon-core](https://github.com/colcon/colcon-core) to cache the processing of packages.

Enables caching of various colcon tasks, such as building or testing packages, by associating successful jobs with the respective state of package source files. In conjunction with [colcon-package-selection](https://github.com/colcon/colcon-package-selection), this extension can accelerate developer or continuous integration workflows by allowing users to finely cache valid workspace artifacts and skip processing of unaltered or unaffected packages during software development. For example, when pulling various changes into an local workspace to review pull requests, this extension can be used to track which packages need to be rebuilt or retested, maximizing the caching of the existing artifacts.

The extension works by generating lockfiles that incorporate the respective state of package source files, either directly via hashing source directories or indirectly via detected revision control. Upon successful task completion for a package job, as when evoking colcon verbs like build, test, etc, these lockfiles are updated for the evoked verb, thereby delineating the provenance of the job’s results. For package selection, these lockfiles are then used to assess whether a verb’s cached outcome for a package remains relevant or valid.

## Subverbs

### `lock` - Lock Package Cache

The `lock` subverb generates or updates lockfiles for selected packages by capturing the current state of package source files. The subverb provides basic arguments to change the build base path where lockfiles are recorded, as well the option to ignore dependencies when capturing package state. More advance arguments specific lock tasks used to capture the package state are also provided.

- `--build-base`
  - The base path for all build directories (default: build)
- `--ignore-dependencies`
  - Ignore dependencies when capturing caches (default: false)

## Package Selection

This extension provides additional package selection arguments that can filter by modified package source or by validity of workspace cache with respect the most recent invocation of lock subverb. By default, the  internal cache key is selected by the colcon verb that invokes the package selection arguments, but can be manually overridden.

- `--packages-select-cache-key`
  - Only process packages using considered cache key. Fallbacks using invoked verb handler if unspecified.

### Modified Package Selection

Check if the `current` checksum in a package's lockfile is matches it's `reference` checksum.

- `--packages-select-cache-modified`
  - Only process a subset of packages whose cache denote package modifications (packages without lockfiles are not considered as modified)
- `--packages-select-cache-unmodified`
  - Only process a subset of packages whose cache denote no package modifications (packages without lockfiles are not considered as unmodified)

### Valid Package Selection

Check if the `current` checksum in the cached lockfile matches the `current` checksum in a package's lockfile.

- `--packages-select-cache-invalid`
  - Only process a subset of packages with an invalid cache (packages without a reference cache are not considered)
- `--packages-skip-cache-valid`
  - Skip a set of packages with a valid cache (packages without a reference cache are not considered)

## Extension points
### VerbExtensionPoint
### PackageAugmentationExtensionPoint
#### DirhashPackageAugmentation
#### GitPackageAugmentation
### PackageSelectionExtensionPoint
#### KeyPackageSelection
#### ModifiedPackageSelection
#### ValidPackageSelection
### TaskExtensionPoint
#### DirhashLockTask
#### GitLockTask
### EventHandlerExtensionPoint
#### LockfileEventHandler

## Example usage

Setup workspace:
```
mkdir -p ~/ws/src && cd ~/ws
wget https://raw.githubusercontent.com/colcon/colcon.readthedocs.org/main/colcon.repos
vcs import src < colcon.repos
```

Lock workspace by generating `cache` lockfiles:
```
colcon cache lock
```

Build and test workspace:
```
colcon build
colcon test
```

Modify package source:
```
echo "#foo" >> src/colcon-cmake/setup.py
```

Update `cache` lockfiles:
```
colcon cache lock
```

List modified packges by comparing `cache` lockfile checksums
```
PKGS_MODIFIED=$(colcon list --packages-select-cache-modified | xarg)
```

Rebuild only modified packages and above:
```
colcon build --packages-above $PKGS_MODIFIED
```

Modify package source again:
```
echo "#bar" >> src/colcon-cmake/setup.py
echo "#baz" >> src/colcon-package-information/setup.py
```

Update cache lockfiles again:
```
colcon cache lock
```

Rebuild by skipping packages with valid `build` lockfiles:
```
colcon build --packages-skip-cache-valid
```

Retest by skipping packages with valid `test` lockfiles:
```
colcon test --packages-skip-cache-valid
```

List generated lockfiles from each `verb`:
```
ls build/colcon-cmake/cache
```