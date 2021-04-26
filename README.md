# colcon-cache

An extension for [colcon-core](https://github.com/colcon/colcon-core) to cache packages for processing.

## Example usage

```
# setup workspace
mkdir -p ~/ws/src && cd ~/ws
wget https://raw.githubusercontent.com/colcon/colcon.readthedocs.org/main/colcon.repos
vcs import src < colcon.repos

# capture cache of workspace source
colcon cache capture

# build and test workspace
colcon build
colcon test

# change package source
echo "#foo" >> src/colcon-cmake/setup.py

# update cache capture
colcon cache capture

# list changed packges by comparing lockfile checksums
PKGS_CHANGED=$(colcon list --packages-select-lock-changed | xarg)

# rebuild only changed packages and above
colcon build --packages-above $PKGS_CHANGED

# alter package source again
echo "#bar" >> src/colcon-cmake/setup.py
echo "#baz" >> src/colcon-package-information/setup.py

# update cache capture again
colcon cache capture

# rebuild changed packages by comparing verb lockfiles
colcon build --packages-skip-cache-hit

# retest packages with any untested build changes
colcon test  --packages-skip-cache-hit

# list generated lockfiles from each verb
ls build/colcon-cmake/cache
```