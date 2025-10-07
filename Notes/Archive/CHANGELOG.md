# SLAM Project Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]
### To Do
- Integrate TF2 transforms properly
- Add Foxy/Jazzy bridge for TurtleBot3
- Implement Phase 2: Particle Filter Localization

## [0.1.0] - 2025-09-26
### Added
- Initial project structure from ROS 2 template
- Phase 1: Basic occupancy grid generator implementation
- Simplified version without TF2 dependencies
- Fake scan publisher for testing
- Comprehensive documentation (DEVELOPMENT_LOG, TROUBLESHOOTING_GUIDE, ALGORITHM_NOTES)
- RViz configuration for visualization
- Launch files for simulation demo

### Fixed
- ROS 2 Jazzy TF2 include path issues (workaround: simplified version)
- CMake stale cache warnings after template duplication

### Technical Details
- Implemented probabilistic occupancy grid with log-odds representation
- Bresenham line algorithm for ray tracing
- Grid: 2000x2000 cells at 0.05m resolution
- Update rate: 10Hz capable

### Known Issues
- RViz shows TF transform warnings (expected without TF2 integration)
- Full TF2 version doesn't compile due to header path issues in Jazzy

---

## Version History Guidelines

### Version Numbering
- **Major (X.0.0)**: Significant architecture changes or new SLAM algorithms
- **Minor (0.X.0)**: New features, phases completed
- **Patch (0.0.X)**: Bug fixes, documentation updates

### When to Update
- After completing each development phase
- When fixing significant bugs
- When adding new algorithms or features
- Before sharing with collaborators
