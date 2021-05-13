# colcon-cache

An extension for [colcon-core](https://github.com/colcon/colcon-core) to cache packages for processing.

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