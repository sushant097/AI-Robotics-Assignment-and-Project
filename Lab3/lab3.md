
# Lab 3 - Pathfinding Algorithms in Robotics

## Overview 

This folder contains the implementation and results for Lab 3, which focuses on pathfinding algorithms in robotics. The lab explores the implementation and comparison of different search algorithms, including A* Search, Greedy Best-first Search, and Uniform Cost Search, across various map environments. The goal is to analyze the efficiency, solution quality, and performance of each algorithm.

## Contents

- **Part A & Part B**: Implementation of A* Search Algorithm.
- **Part C**: Comparison of Greedy Best-first Search and Uniform Cost Search with A* Search on five different maps.

## Implementation Details


### Part A & Part B: A* Search Algorithm

The implementation of the A* Search algorithm can be found in the `astar.py` script. This algorithm is tested on multiple maps to evaluate its performance.

**Run Command**:
```bash
python astar.py world/world.world ASTAR
```

### Part C: Comparison of Search Algorithms

In this part of the lab, the priority calculation in the `astar.py` script is modified to enable the use of Greedy Best-first Search (GBFS) and Uniform Cost Search (UCS). The performance of these algorithms is compared against the A* Search algorithm across five different maps.

**Run Commands**:
- A* Search:
  ```bash
  python astar.py world/world.world ASTAR
  ```
- Greedy Best-first Search:
  ```bash
  python astar.py world/world.world GBFS
  ```
- Uniform Cost Search:
  ```bash
  python astar.py world/world.world UCS
  ```

### Results

The results of the comparisons, including the number of node expansions, solution quality, search time, and the path generated for each map, are detailed below:

#### 1. A* Search Results

1.1 **house.world**:
- Node Expansions: 2220
- Quality/Cost of Solution: 20.2893
- Search Time: 0.4439 seconds

1.2 **maze1.world**:
- Node Expansions: 5259
- Quality/Cost of Solution: 48.0972
- Search Time: 2.2390 seconds

1.3 **maze2.world**:
- Node Expansions: 5487
- Quality/Cost of Solution: 46.8647
- Search Time: 2.2376 seconds

1.4 **maze3.world**:
- Node Expansions: 5772
- Quality/Cost of Solution: 44.4072
- Search Time: 2.6937 seconds

1.5 **maze4.world**:
- Node Expansions: 4629
- Quality/Cost of Solution: 24.4387
- Search Time: 1.5998 seconds

#### 2. Greedy Best-first Search (GBFS) Results

2.1 **house.world**:
- Node Expansions: 119
- Quality/Cost of Solution: 20.4131
- Search Time: 0.0208 seconds

2.2 **maze1.world**:
- Node Expansions: 3371
- Quality/Cost of Solution: 64.9386
- Search Time: 1.0691 seconds

2.3 **maze2.world**:
- Node Expansions: 2244
- Quality/Cost of Solution: 56.2273
- Search Time: 0.5640 seconds

2.4 **maze3.world**:
- Node Expansions: 4128
- Quality/Cost of Solution: 77.5922
- Search Time: 1.3657 seconds

2.5 **maze4.world**:
- Node Expansions: 2521
- Quality/Cost of Solution: 31.5154
- Search Time: 0.5859 seconds

#### 3. Uniform Cost Search (UCS) Results

3.1 **house.world**:
- Node Expansions: 6607
- Quality/Cost of Solution: 20.2893
- Search Time: 3.2691 seconds

3.2 **maze1.world**:
- Node Expansions: 5982
- Quality/Cost of Solution: 48.0972
- Search Time: 2.8465 seconds

3.3 **maze2.world**:
- Node Expansions: 6652
- Quality/Cost of Solution: 46.8647
- Search Time: 3.2270 seconds

3.4 **maze3.world**:
- Node Expansions: 6595
- Quality/Cost of Solution: 44.4072
- Search Time: 3.0721 seconds

3.5 **maze4.world**:
- Node Expansions: 6967
- Quality/Cost of Solution: 24.4387
- Search Time: 3.2945 seconds

## Analysis and Discussion

### A* Search
- **Advantages**: A* search is optimal and complete, guaranteeing the shortest path if the heuristic is admissible.
- **Disadvantages**: It requires more memory and computational time, especially in complex maps like `maze3.world`.

### Greedy Best-first Search (GBFS)
- **Advantages**: GBFS is faster and requires fewer node expansions due to its heuristic-driven approach.
- **Disadvantages**: It is not optimal and can lead to longer paths, as observed in the increased cost of solutions.

### Uniform Cost Search (UCS)
- **Advantages**: UCS is also optimal, similar to A*, but without the heuristic guidance, leading to more node expansions.
- **Disadvantages**: It generally takes longer than A* due to the lack of a heuristic to prioritize node expansion.

### Conclusion
- **Best Performance**: A* search performs the best in terms of solution quality but at a higher computational cost.
- **Fastest Algorithm**: GBFS is the fastest, but it sacrifices solution quality.
- **Overall Recommendation**: A* is recommended for scenarios where optimality is crucial, while GBFS can be used where speed is more important than the optimal path.



**The full solution pdf found in [here](Lab3.pdf).**
