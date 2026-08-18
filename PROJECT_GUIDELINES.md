# CoE-BIRD Project Repository Guidelines

This document describes the recommended structure, conventions, and archival requirements for project repositories hosted under the CoE-BIRD GitHub organization.

These guidelines are intended to ensure:

* reproducibility,
* maintainability,
* discoverability,
* and long-term archival value of robotics projects.

---

# 1. Repository Naming Convention

Repositories should follow the naming format:

```text
<year>-<project-type>-<project-name>
```

Examples:

```text
2026-major-quadruped-navigation
2026-cornerstone-mobile-manipulation
2026-minor-ros2-exploration
```
Use lowercase letters and hyphens only.

---

# 2. Required Repository Topics

All repositories must include GitHub Topics.

Topics improve discoverability and allow projects to be grouped by:

* research area,
* robot platform,
* and technologies used.

Topics can be added via:

```text
Repository → About → ⚙ Edit → Topics
```

---

## 2.1 Mandatory Topics

Every repository must include:

### A. One Project Type Topic

Choose exactly one:

```text
major-project
cornerstone-project
minor-project
```

---

### B. At Least One Robotics Area Topic

Choose one or more:

```text
perception
cognition
navigation
control
manipulation
robot-learning
multi-agent
human-robot-interaction
embedded-systems
medical-robotics
wearable-robotics
```

---

## 2.2 Optional Platform Topics

Examples:

```text
drone
quadruped
humanoid
mobile-robot
robot-arm
prosthetics
exoskeleton
```

---

## 2.3 Optional Technology Topics

Examples:

```text
ros2
slam
reinforcement-learning
computer-vision
sensor-fusion
mpc
event-camera
gazebo
isaac-sim
mujoco
```

---

# 3. Recommended Repository Structure

Repositories should follow the template structure below.

```text
README.md
LICENSE

code/
report/
media/
docs/
data/
hardware/
simulation/
```

---

## Folder Descriptions

| Folder        | Purpose                                   |
| ------------- | ----------------------------------------- |
| `code/`       | Main source code                          |
| `report/`     | Thesis/report/slides                      |
| `media/`      | Images, GIFs, videos, posters             |
| `docs/`       | Additional documentation                  |
| `data/`       | Small sample datasets/configuration files |
| `hardware/`   | CAD files, wiring diagrams, BOM           |
| `simulation/` | Simulation assets and environments        |

---

# 4. README Requirements

The repository README acts as the public project webpage.

It should contain:

* project title,
* abstract,
* system overview,
* methodology,
* results,
* demo video,
* setup instructions,
* report links,
* contact information.

Projects should aim to make the README understandable to readers outside the immediate subfield.

---

# 5. Reproducibility Guidelines

Projects should include sufficient information for future students and researchers to:

* understand the system,
* build the software,
* and reproduce key results.

Recommended:

* `requirements.txt`
* `environment.yml`
* Docker setup
* ROS2 workspace instructions
* hardware specifications
* configuration files

---

# 6. Large Files and Datasets

Avoid committing large files directly into Git history.

Examples:

* large datasets,
* rosbags,
* trained models,
* videos.

Recommended alternatives:

* Git LFS
* institutional storage
* cloud storage links

---

# 7. Security and Repository Hygiene

Do NOT commit:

* passwords,
* API keys,
* SSH keys,
* confidential data,
* proprietary datasets,
* licensed software binaries.

Repositories may be reviewed before public archival.

---

# 8. Final Submission Checklist

Before final submission, ensure that:

* [ ] Repository name follows convention
* [ ] GitHub Topics have been added
* [ ] README is complete
* [ ] Demo video link is included
* [ ] Report/thesis PDF is uploaded
* [ ] Installation instructions are provided
* [ ] Dependencies are documented
* [ ] No secrets or credentials are committed
* [ ] Repository structure is organized
* [ ] Code is cleaned and documented

---

# 9. Archival Policy

Major projects may be:

* archived permanently,
* made public,
* and showcased as representative CoE-BIRD research outputs.

Minor and cornerstone projects may remain:

* private,
* unarchived,
* or selectively curated.

Visibility decisions are subject to:

* supervisor approval,
* IP constraints,
* sponsor requirements,
* and publication considerations.

---

# 10. Acknowledgement

Please acknowledge CoE-BIRD and IIT Delhi appropriately in:

* reports,
* publications,
* posters,
* and public releases where applicable.
