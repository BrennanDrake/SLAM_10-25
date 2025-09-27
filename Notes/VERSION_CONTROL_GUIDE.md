# Version Control Guide for SLAM Project

## Git Workflow Strategy

### Branch Structure
```
main (or master)
├── develop
│   ├── feature/particle-filter
│   ├── feature/loop-closure
│   └── feature/tf2-integration
├── release/v0.2.0
└── hotfix/critical-bug-fix
```

### Recommended Workflow: Git Flow Lite

#### Main Branches
- **main**: Production-ready code, tagged releases only
- **develop**: Integration branch for features

#### Supporting Branches
- **feature/***: New features (Phase 2, 3, etc.)
- **release/***: Prepare for release
- **hotfix/***: Emergency fixes to main

### Basic Git Commands for Your Project

```bash
# Initial setup (one time)
git init
git remote add origin https://github.com/yourusername/SLAM_10-25.git

# First push
git add .
git commit -m "Initial commit: Phase 1 occupancy grid generator"
git push -u origin main

# Create and switch to develop branch
git checkout -b develop
git push -u origin develop

# Start a new feature
git checkout -b feature/particle-filter develop
# ... make changes ...
git add .
git commit -m "feat: Add particle filter localization"
git push -u origin feature/particle-filter

# Merge feature back to develop
git checkout develop
git merge --no-ff feature/particle-filter
git push origin develop
```

## Version Numbering System

### Semantic Versioning (SemVer)
Format: **MAJOR.MINOR.PATCH** (e.g., 1.2.3)

- **MAJOR (1.x.x)**: Incompatible API changes, major architecture shifts
- **MINOR (x.2.x)**: New functionality, backwards compatible
- **PATCH (x.x.3)**: Bug fixes, backwards compatible

### For Your SLAM Project

```
0.1.0 - Phase 1: Basic occupancy grid (current)
0.2.0 - Phase 2: Particle filter localization
0.3.0 - Phase 3: Graph-based SLAM
0.4.0 - Phase 4: Loop closure
0.5.0 - Phase 5: Multi-robot SLAM
1.0.0 - Production-ready complete SLAM system
```

## How to Tag Versions in Git

### Creating Version Tags

```bash
# Tag current commit
git tag -a v0.1.0 -m "Phase 1: Basic occupancy grid generator"
git push origin v0.1.0

# Tag specific commit
git tag -a v0.1.1 abc1234 -m "Fix: Resolve TF2 include paths"
git push origin v0.1.1

# List all tags
git tag -l

# Show tag details
git show v0.1.0
```

### GitHub Releases

1. **Via Command Line**:
```bash
# After tagging
git push origin v0.1.0
# Then go to GitHub and create release from tag
```

2. **Via GitHub Web**:
   - Go to your repo → Releases → "Create a new release"
   - Choose tag: v0.1.0
   - Release title: "Phase 1: Occupancy Grid Generator"
   - Add description from CHANGELOG.md
   - Attach binaries if needed
   - Publish release

## Commit Message Convention

### Format
```
<type>(<scope>): <subject>

<body>

<footer>
```

### Types
- **feat**: New feature
- **fix**: Bug fix
- **docs**: Documentation only
- **style**: Code style (formatting, semicolons, etc)
- **refactor**: Code change that neither fixes bug nor adds feature
- **perf**: Performance improvement
- **test**: Adding tests
- **chore**: Maintenance tasks

### Examples for Your Project
```bash
# Feature commit
git commit -m "feat(mapping): Add probabilistic occupancy grid with log-odds"

# Fix commit
git commit -m "fix(build): Resolve TF2 include path for ROS 2 Jazzy"

# Documentation
git commit -m "docs: Add troubleshooting guide for common build errors"

# Performance
git commit -m "perf(grid): Optimize Bresenham line algorithm"
```

## Managing Package Versions

### Option 1: Manual in package.xml
```xml
<!-- ros2_ws/src/occupancy_grid_generator/package.xml -->
<package format="3">
  <name>occupancy_grid_generator</name>
  <version>0.1.0</version>  <!-- Update this -->
  ...
</package>
```

### Option 2: Version File
```python
# version.py or VERSION file
VERSION = "0.1.0"
```

### Option 3: Git-based (Automatic)
```bash
# Get version from latest tag
VERSION=$(git describe --tags --always)
```

## Development Workflow Example

### Starting Your Next Session

```bash
# 1. Pull latest changes
git pull origin develop

# 2. Create feature branch for Phase 2
git checkout -b feature/particle-filter

# 3. Work on feature
# ... edit files ...

# 4. Commit regularly
git add -A
git commit -m "feat(localization): Implement particle initialization"

# 5. Push to GitHub
git push origin feature/particle-filter

# 6. Create Pull Request on GitHub (if working with others)
# Or merge locally if working alone
git checkout develop
git merge --no-ff feature/particle-filter
git push origin develop

# 7. When ready for release
git checkout main
git merge --no-ff develop
git tag -a v0.2.0 -m "Phase 2: Particle filter localization"
git push origin main --tags
```

## GitHub-Specific Features

### GitHub Actions for Automation
Create `.github/workflows/ros2_build.yml`:
```yaml
name: ROS 2 Build
on: [push, pull_request]
jobs:
  build:
    runs-on: ubuntu-latest
    container: ros:jazzy
    steps:
      - uses: actions/checkout@v2
      - name: Build workspace
        run: |
          cd ros2_ws
          colcon build
```

### Issue Tracking
Link commits to issues:
```bash
git commit -m "fix: Resolve memory leak in grid update, closes #12"
```

### Project Boards
Use GitHub Projects to track phases:
- Column 1: To Do (Phase 2-5)
- Column 2: In Progress (Current phase)
- Column 3: Testing
- Column 4: Done (Phase 1)

## Best Practices for Your SLAM Project

### 1. Commit Frequently
- Commit logical units of work
- Don't mix features in one commit
- Keep commits atomic and reversible

### 2. Write Good Commit Messages
- First line: 50 chars max, imperative mood
- Body: Explain what and why, not how
- Reference issues and PRs

### 3. Use .gitignore
```gitignore
# ROS 2 build artifacts
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/

# Python
__pycache__/
*.pyc

# IDE
.vscode/
.idea/

# OS
.DS_Store
Thumbs.db
```

### 4. Tag Strategically
- Tag when completing phases
- Tag before major refactoring
- Tag stable versions for testing

### 5. Document in Commits
- Update CHANGELOG.md in same commit as feature
- Keep documentation in sync with code

## Quick Reference Commands

```bash
# Status and logs
git status
git log --oneline --graph --all
git diff

# Branching
git branch -a                    # List all branches
git checkout -b feature/name     # Create and switch
git branch -d feature/name       # Delete local branch

# Tagging
git tag -a v0.1.0 -m "message"  # Create annotated tag
git push origin --tags           # Push all tags
git tag -d v0.1.0               # Delete local tag

# Stashing (save work temporarily)
git stash                        # Save current changes
git stash pop                    # Restore changes

# Undoing
git reset --soft HEAD~1         # Undo last commit, keep changes
git reset --hard HEAD~1         # Undo last commit, discard changes
git revert <commit-hash>        # Create new commit that undoes
```

## Tonight's Push Checklist

```bash
# 1. Check what will be committed
git status
git diff

# 2. Add all files (or selectively)
git add -A
# OR selective
git add ros2_ws/src/
git add Notes/
git add .gitignore

# 3. Commit with meaningful message
git commit -m "feat: Initial SLAM project with occupancy grid generator

- Implemented Phase 1: Basic occupancy grid mapping
- Created simplified version without TF2 dependencies  
- Added fake scan publisher for testing
- Comprehensive documentation in Notes/
- Resolved ROS 2 Jazzy build issues"

# 4. Tag the version
git tag -a v0.1.0 -m "Phase 1: Basic occupancy grid generator"

# 5. Push to GitHub
git push -u origin main
git push origin --tags

# 6. Create GitHub Release (optional)
# Go to GitHub web interface → Releases → Create from tag
```

---
*Version Control Guide Created: 2025-09-26*
*Remember: Version control is about telling the story of your project's evolution*
