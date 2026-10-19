# Kestrel

Kestrel is our own simulator stack for an autonomous racing drone that flies by one normal camera.
A kestrel is a small falcon that holds still in the air and then strikes fast. That is the job here: hold a gate in view, then race through it.

## The stack

| Part | Role |
|---|---|
| PX4 | Flight software, the same code path a real drone uses (MAVSDK) |
| Gazebo | Main simulator (free GitHub machine, Google Colab T4) |
| Project AirSim | Realism stage: photo-like scenes, cars, people |
| Crazyflow | Fast learning: many worlds at once on a GPU |

We also bring in the better features of other simulators: gates, tracks and lap timing; motion blur, sensor delay, noise and
random drones; other controllers (Mellinger, MPC). The order of the work:

1. Gates and lap timing (this first piece: `racecourse/`)
2. The camera-to-control loop in PX4 + Gazebo
3. One camera-effects layer that works the same in every simulator
4. Crazyflow for training

## What is here now

- `racecourse/course.py`: a course read from a route file, and a lap counter that counts gates in order over several laps and
  times each lap. It uses no simulator, so every simulator decides "gate passed" by the same rule.
- `tracks/hard_race_v1.json`: the hard race route (seven gates, two poles, one box).
- `tests/test_course.py`: offline checks; GitHub runs them on every push (`.github/workflows/tests.yml`).
