# colcon-cache

[![GitHub Workflow Status](https://github.com/ruffsl/colcon-cache/actions/workflows/test.yml/badge.svg)](https://github.com/ruffsl/colcon-cache/actions/workflows/test.yml)
[![Codecov](https://codecov.io/gh/ruffsl/colcon-cache/branch/master/graph/badge.svg)](https://codecov.io/gh/ruffsl/colcon-cache)

An extension for [colcon-core](https://github.com/colcon/colcon-core) to cache the processing of packages.

Enables caching of various colcon tasks, such as building or testing packages, by associating successful jobs with the respective state of package source files. In conjunction with [colcon-package-selection](https://github.com/colcon/colcon-package-selection), this extension can accelerate developer or continuous integration workflows by allowing users to finely cache valid workspace artifacts and skip processing of unaltered or unaffected packages during software development.

The extension works by generating lockfiles that incorporate the respective state of package source files, either directly via hashing source directories or indirectly via detected revision control. Upon successful task completion for a package job, as when evoking colcon verbs like build, test, etc, these lockfiles are updated for the evoked verb, thereby delineating the provenance of the job’s results. For package selection, these lockfiles are then used to assess whether a verb’s cached outcome for a package remains relevant or valid.

## Subverbs

### `lock` - Lock Package Cache

The `lock` subverb generates or updates lockfiles for selected packages by capturing the current state of package source files. The subverb provides basic arguments to change the build base path where lockfiles are recorded, as well the option to ignore dependencies when capturing package state. More advance arguments specific lock tasks used to capture the package state are also provided.

## Package selection arguments

### Modified Package Selection

#### `--packages-select-cache-modified`
#### `--packages-select-cache-unmodified`

### Valid Package Selection

#### `--packages-select-cache-invalid`
#### `--packages-skip-cache-valid`

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