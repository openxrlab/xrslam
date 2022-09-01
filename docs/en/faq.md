# FAQ

We list some common issues faced by many users and their corresponding solutions here.

Feel free to enrich the list if you find any frequent issues and have ways to help others to solve them.

## Installation

* **"Stuck at download OpenCV"**

  set up proxy or change network will solve the problem.
* **"CMake Error: The RandR headers were not found"**

  ```bash
  apt-get install libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
  ```
* **"/usr/include/c++/bits/stl_pair.h:106:45: error: 'value' is not a member of std::_and..."**

  You may use GCC 10, but this project currently only supports GCC 9, please switch to the corresponding version.
