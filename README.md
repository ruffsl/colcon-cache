# colcon-cache

An extension for [colcon-core](https://github.com/colcon/colcon-core) to cache packages for processing.

## Example usage

```
# setup workspace
mkdir -p ~/ws/src && cd ~/ws
wget https://raw.githubusercontent.com/colcon/colcon.readthedocs.org/main/colcon.repos
vcs import src < colcon.repos

# lock cache of workspace source
colcon cache lock

# build and test workspace
colcon build
colcon test

# change package source
echo "#foo" >> src/colcon-cmake/setup.py

# update cache lock
colcon cache lock

# list changed packges by comparing lockfile checksums
PKGS_CHANGED=$(colcon list --packages-select-lock-changed | xarg)

# rebuild only changed packages and above
colcon build --packages-above $PKGS_CHANGED

# alter package source again
echo "#bar" >> src/colcon-cmake/setup.py
echo "#baz" >> src/colcon-package-information/setup.py

# update cache lock again
colcon cache lock

# rebuild changed packages by comparing verb lockfiles
colcon build --packages-skip-cache-valid

# retest packages with any untested build changes
colcon test  --packages-skip-cache-valid

# list generated lockfiles from each verb
ls build/colcon-cmake/cache
```