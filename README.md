# autoware_core

- An [Autoware](https://github.com/autowarefoundation/autoware) repository that contains a basic set of high-quality, stable ROS packages for autonomous driving.

- The scope and design of Autoware Core are guided by the ongoing [Autoware Architecture WG](https://github.com/autowarefoundation/autoware/discussions?discussions_q=label%3Aarchitecture_wg) discussions.

- A more detailed explanation about Autoware Core can be found on the [Autoware concepts documentation page](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-concepts/#the-core-module).

- For researchers and developers who want to extend the functionality of Autoware Core with experimental, cutting-edge ROS packages, see [Autoware Universe](https://github.com/autowarefoundation/autoware_universe).

## Code Coverage Metrics

The table below shows the coverage rate of the entire Autoware Core and of each sub-component.

### Entire Project Coverage

[![codecov](https://codecov.io/gh/autowarefoundation/autoware_core/branch/main/graph/badge.svg)](https://app.codecov.io/gh/autowarefoundation/autoware_core/tree/main)

### Component-wise Coverage

You can check more details by clicking a badge and navigating the Codecov website.

| Component    | Coverage                                                                                                                                                                                                                                                                                                           |
| ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| API          | [![codecov](https://codecov.io/gh/autowarefoundation/autoware_core/branch/main/graph/badge.svg?component=api-packages&precision=2)](https://app.codecov.io/gh/autowarefoundation/autoware_core?components%5B0%5D=API%20Packages)                  |
| Common       | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware_core/components&label=Common%20Packages&query=$.[1].coverage&suffix=%25)](https://app.codecov.io/gh/autowarefoundation/autoware_core?components%5B0%5D=Common%20Packages)             |
| Control      | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware_core/components&label=Control%20Packages&query=$.[2].coverage&suffix=%25)](https://app.codecov.io/gh/autowarefoundation/autoware_core?components%5B0%5D=Control%20Packages)           |
| Localization | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware_core/components&label=Localization%20Packages&query=$.[3].coverage&suffix=%25)](https://app.codecov.io/gh/autowarefoundation/autoware_core?components%5B0%5D=Localization%20Packages) |
| Map          | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware_core/components&label=Map%20Packages&query=$.[4].coverage&suffix=%25)](https://app.codecov.io/gh/autowarefoundation/autoware_core?components%5B0%5D=Map%20Packages)                   |
| Perception   | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware_core/components&label=Perception%20Packages&query=$.[5].coverage&suffix=%25)](https://app.codecov.io/gh/autowarefoundation/autoware_core?components%5B0%5D=Perception%20Packages)     |
| Planning     | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware_core/components&label=Planning%20Packages&query=$.[6].coverage&suffix=%25)](https://app.codecov.io/gh/autowarefoundation/autoware_core?components%5B0%5D=Planning%20Packages)         |
| Sensing      | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware_core/components&label=Sensing%20Packages&query=$.[7].coverage&suffix=%25)](https://app.codecov.io/gh/autowarefoundation/autoware_core?components%5B0%5D=Sensing%20Packages)           |
| Testing      | [![codecov](https://img.shields.io/badge/dynamic/json?url=https://codecov.io/api/v2/github/autowarefoundation/repos/autoware_core/components&label=Testing%20Packages&query=$.[8].coverage&suffix=%25)](https://app.codecov.io/gh/autowarefoundation/autoware_core?components%5B0%5D=Testing%20Packages)           |
