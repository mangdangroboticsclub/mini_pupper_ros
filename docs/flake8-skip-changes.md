# Flake8 Skip Changes (Deferred from Jazzy Migration)

These changes are needed if `colcon test` flake8 failures need to be suppressed.
The failures are pre-existing style violations from the Humble upstream, not Jazzy regressions.

## CMakeLists.txt changes (5 packages)

Add the following line inside the `if(BUILD_TESTING)` block, before `ament_lint_auto_find_test_dependencies()`:

```cmake
set(ament_cmake_flake8_FOUND TRUE)
```

### Files:
- `mini_pupper_bringup/CMakeLists.txt`
- `mini_pupper_description/CMakeLists.txt`
- `mini_pupper_fleet/CMakeLists.txt`
- `mini_pupper_navigation/CMakeLists.txt`
- `mini_pupper_simulation/CMakeLists.txt`

## test_flake8.py changes (4 packages)

Add the following decorator before `def test_flake8():`:

```python
@pytest.mark.skip(reason='Style-only issues deferred from Jazzy migration')
```

### Files:
- `mini_pupper_dance/test/test_flake8.py`
- `mini_pupper_driver/test/test_flake8.py`
- `mini_pupper_music/test/test_flake8.py`
- `stanford_controller/test/test_flake8.py`
