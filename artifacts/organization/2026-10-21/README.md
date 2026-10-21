# Project reorganization evidence

- `main-moves.json`: old/new paths; 28 moved firmware and historical files retain their bytes.
- `preservation.json`: all 24 pre-existing modified/untracked user files are unchanged.
- `firmware-preservation.json`: both Pico runtime sources match their previous Git revision, allowing line-ending normalization.
- `build-rp2350.log`, `build-rp2040.log`: both firmware targets build using the new entry point and paths.
- `build-sitl.log`: production C/C++ host libraries build from the new paths.
- `math-render.log`: the existing offline guide equations render after updating links.
- `layout-check.json`: command targets/domain paths and active Python/JavaScript syntax checks.
- `launcher-attempt-1.log`: original argument-binding failure before the launcher fix.

No firmware was flashed and no new physical or Gazebo flight was performed.
Only architecture/path/launcher behavior changed. Existing evidence is kept in place;
the new commits exclude the user's pre-existing modifications and untracked recordings.
