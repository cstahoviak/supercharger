# Tesla Coding Challenge

Sections:
- [Overview](#overview)
- [Problem Statement](#problem-statement)
- [Solution](#solution)
- [Results](#results)
- [Future Work](#future-work)

## Overview
This repository implements a solution to the Tesla Supercharger coding challenge (described in more detail in the [Problem Statement](#problem-statement) section). The following is an overview of the solution method and supported features:

### Solution Method

- Algorithm (5) (see Tables 1, 2 and 3 in the [Rsesults](#results) section) approaches the route planning problem as a two-step process:
  - First, a _pseudo_ time-optimal path is found via Dijkstra's algorithm. In this implementation, Dijkstra's cost function makes the assumption that the car will charge only long enough at each charger to make it to the next charger, i.e. the vehicle's arrival range at each charger will be zero.
  - Next, the solution is refined via constrained optimization (using the [NLOpt](https://nlopt.readthedocs.io/en/latest/) library). The optimization scheme minimizes the total charge time by increasing the charging time at nodes with relatively high charging rates, and decreasing the charging time for nodes with low charging rates.
  - This approach achieves an average improvement of over [32.5 minutes](#results) over the _reference result_.
- Algorithm (6) borrows the same optimization scheme as described in the second part of Algorithm (5), but rather than applying a single post-optimization step to the route, the optimization step is built into the cost function used by Dijkstra's algorithm to guarantee that the cost computed for each node in the graph is truly optimal.
	- Incorporating the optimization step into the cost function evaluation comes with a runtime penalty, but is able to acheive an average improvement of over [37.5 minutes](#results) over the _reference result_ (and a 5 minute improvement over Algorithm (5)).
- The constrained optimization problem that both Algorithms (5) and (6) solve can be expressed as  follows

	![Optimization Problem](/figs/optimization.png "Charging Durations")

	where the objective function is the sum of charging durations at nodes `[2, n-1]` (not including the origin and the destination), and the constraint function ensures that the arrival range at each node `[3, n]` is greater than zero, i.e. the vehicle does not run out of charge at any point along the route. The matrix `A` is a function of the charging rates, and the vectors `r` and `d` are the charging rates and distances between adjacent nodes, respectively.

### Features
- The `pysupercharger` module is provided to support python development.
  - The python bindings are written using the [`pybind11`](https://pybind11.readthedocs.io/en/stable/) library.
  - Both the `Planner` and `Optimizer` classes are extensible on the python side. For example, the pure-python `NonlinearOptimizer` class (from the `supercharger.optimize` module) inherits from the bound `Optimizer` class.
  - This workflow enabled rapid prototyping of the constrained optimization improvement by allowing me to experiment with Scipy `minimize` before implementing the optimization solution in C++ via NLOpt.


## Problem Statement
Your objective is to construct a search algorithm to find the minimum time path through the tesla network of supercharging stations. Each supercharger will refuel the vehicle at a different rate given in km/hr of charge time. Your route does not have to fully charge at every visited charger, so long as it never runs out of charge between two chargers. We suggest implementing a quick brute force method before attempting to find an optimal routine.

You will be provided with a code skeleton which includes a header with the
charger network data in the format:

	name, latitude in degrees, longitude in degrees, charge rate in km/hr
 
You may compare your solutions against our reference implementation using the
provided "checker" programs in either OSX or linux, make sure to use it to check
your submission output against several different start and end chargers.

__Input__: Your program should take as input two strings: `“start charger name”`
, `“end charger name"`

__Output__: Your program’s only output should be a print to `std::out` of a
string in the format:

	initial charger name, first charger name, charge time in hrs, 
	second charger name, charge time in hrs,
	…,
	…,
	goal charger name

This is the format required by the checker program as well, for example the
command
```
./solution Council_Bluffs_IA Cadillac_MI
```

might return:

	Council_Bluffs_IA, Worthington_MN, 1.18646, Albert_Lea_MN, 1.90293, Onalaska_WI, 0.69868, Mauston_WI, 1.34287, Sheboygan_WI, 1.69072, Cadillac_MI
	
You can check the solution by providing your output to the included checker, for example
```
./checker_linux “Council_Bluffs_IA, Worthington_MN, 1.18646, Albert_Lea_MN, 1.90293, Onalaska_WI, 0.69868, Mauston_WI, 1.34287, Sheboygan_WI, 1.69072, Cadillac_MI”
```

will return 

	Finding Path Between Council_Bluffs_IA and Cadillac_MI
	Reference result: Success, cost was 17.2531
	Candidate result: Success, cost was 17.2548


You should make the following __assumptions__:

- The car begins at the start charger with a full charge of 320km
- The car travels at a constant speed of 105km/hr along great circle routes
between chargers.
- The Earth is a sphere of radius 6356.752km


Your submission will be run against several start and end chargers and evaluated in terms of the following metrics:

- Path satisfiability (car remains above 0km of range throughout entire trip)
- Path optimality (total time driving + charging)
- Coding structure
- Code clarity
- Computational cost


You should ensure that your submission compiles under gcc 4.8.4 with optimization level 1, for example:

```
g++ -std=c++11 -O1 main.cpp network.cpp -o candidate_solution
```

if your solution includes additional cpp files, include a README file with the appropriate compiler string.

The solution needs to be self-contained with just the use of STL algorithms
(i.e. do not use off-the-shelf packages).


## Solution

### Dependencies
The `supercharger` application depends on the following libraries:

- [GoogleTest](https://google.github.io/googletest/primer.html)
- [pybind11](https://pybind11.readthedocs.io/en/latest/)
- [NLOpt](https://nlopt.readthedocs.io/en/latest/)

These libraries are built and installed locally during the build process (see `cpp/thirdparty`) and do not need to be installed by the user prior to building the `supercharger` project.

### Building
Build the `supercharger` application by following the standard CMake build process.

```
git clone https://github.com/cstahoviak/supercharger.git
cd supercharger
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

### Running the `supercharger` Application
Run the `supercharger` application by passing any two valid Supercharger locations to the executable, e.g.

```
./supercharger Council_Bluffs_IA Cadillac_MI
```

### Tests
The unit tests can be run via

```
./cpp/test/test_supercharger 
```

## Results

### Single Route Results: Council Bluffs, IA to Cadillac, MI

The following table compares algorithm performance (as measured by route cost in hours) for the example route described in the [Problem Statement](#problem-statement). The _reference result_ provided by the `checker_linux` application is included for comparison. The last two columns indicate the percent improvement over the reference result, and the time saved compared to the reference result.

| Num | Algorithm                                    | Cost     | Pct Imprv. | Time Saved |
|:---:|:---------------------------------------------|:--------:|:----------:|:----------:|
|     |                                              | [hrs]    | [%]        | [min:sec]  |
| 1   | _Naive_ Planner                              | 18.1017  | -2.9185    | +50:55     |
| 2   | Dijkstra's Algorithm*                        | 17.2548  | -0.0096    | +00:06     |
| 3   | _Reference_ Result                           | 17.2531  | -          | -          |
| 4   | Dijkstra's + _Naive_ Optimizer               | 17.0697  | 1.0630     | -11:00     | 
| 5   | Dijkstra's + Post Optimization**             | 16.8438  | 2.3723     | -24:33     |
| 6   | Dijkstra's with _Optimized_ Cost Function*** | 16.8438  | 2.3723     | -24:33     |

Table 1: Algorithm performance for the sample route.

*This version of Dijkstra's algorithm uses a _naive_ cost function that does not fully account for the information encoded in the charging rate at each node. The cost for each node is computed as the travel time between nodes (distance / vehicle speed) plus a charging duration at each node that guarantees an arrival range of exactly 0 km at the next node. No "back-optimization" is performed to account for variable charge rates or to allow the vehicle to arrive at a given node along the route with greater than zero range.

**_Post-optimization_ uses the result of algorithm (2) as its starting point, and then refines the charging durations at each node along the route via a constrained optimization scheme that uses NLopt's [SLSQP](https://nlopt.readthedocs.io/en/latest/NLopt_Algorithms/#slsqp) algorithm.

***Algorithm (6) borrows the same constrained optimization scheme used by algorithm (5), but rather than applying a single post-optimization step to the route, the optimization scheme is incorportated into the cost function used by Dijkstra's algorithm. This ensures that the cost that Dijkstra's algorithm computes for each node is truly optimal.

The following figure illustrates how the constrained optimization scheme of algorithms (5) and (6) maximizes the charging time at nodes with relatively high charging rates, and decreases the charging time for nodes with low charging rates.

![charging durations](/figs/charging_durations_stacked.png "Charging Durations")

Figure 1: The lighter area at the bottom of each bar represents the total charging time _up to_ that node, and the darker area at the top of the bar represents the charge time _at_ that node. Upon arrival at the destination node (Cadillac, MI), the constrained optimization scheme has succesfully reduced the total charging time from 6.8217 hrs (6:49) to 6.4107 hrs (6:24) for a total savings of over 24 minutes.

### A Statistical Analysis of Planning Algorithms

For the route considered in the preceding section, no advantage was gained by incorporating the optimization step into the cost function, as opposed to performing a single post-optimization step after the route had been determined by Dijkstra's algorithm using a _naive_ cost function. However, that is not the case once a larger sample of routes is considered.

The following tables describe algorithm performance (in terms of both route cost in hours and algorithm runtime) for a sample of 1000 routes of varying lengths. Only several implementations of Dijkstra's algorithms (2, 5, 6) are evaluated here.

| Num | Algorithm                                 | mean    | std     | max        | min     |
|:---:|:------------------------------------------|:-------:|:-------:|:----------:|:-------:|
|     |                                           | [mm:ss] | [mm:ss] | [hh:mm:ss] | [mm:ss] |
| 2   | Dijkstra's Algorithm                      | -02:29  | -04:52  | -37:04     | +00:31  |
| 5   | Dijkstra's + Post Optimization            | -32:33  | -21:34  | -01:41:56  | -00:16  |
| 6   | Dijkstra's with _Optimized_ Cost Function | -37:49  | -23:35  | -02:05:01  | -00:16  |

Table 2: Algorithm performance (in terms of route cost in hours) relative to the _reference_ result. Algorithm (6) has an average improvement 37:49 over the _reference_ result, and an average improvement of 5:16 over algorithm (5).

| Num | Algorithm                                  | mean    | std     | max      | min   |
|:---:|:-------------------------------------------|:-------:|:-------:|:--------:|:-----:|
|     |                                            | [ms]    | [ms]    | [ms]     | [ms]  |
| 2   | Dijkstra's Algorithm                       | 4.643   | 1.707   | 11.118   | 0.790 |
| 5   | Dijkstra's + Post Optimization             | 4.858   | 1.873   | 11.441   | 0.835 |
| 6   | Dijkstra's with _Optimized_ Cost Function* | 175.321 | 222.121 | 1425.543 | 2.343 |

Table 3: Planning Algorithm profiling (in milliseconds).

*The A-Star algorithm might be able to improve on this runtime performance by being more efficient in directing the exploration of nodes along the route.


### Constrained Nonlinear Optimization with SciPy `minimize`
Adding python bindings to the project via the `pybind11` package has enabled experimentation with various types of optimization algorithms, and has given me additional insight about the use cases and limitations of specific methods. The table below details the types of problems that each optimization algorithm is (and isn't) suitable for.

Based on the information in the table, there are only three optimization algorithms suitable for __constrained optimization__: `COBYQA`, `SLSQP` and the Trust-Region Constrained (`trust-constr`) method.

| Optimization Algorithm                    | Name         | Bounds | Constraints | Gradient | Hessian     |
|:------------------------------------------|:-------------|:------:|:-----------:|:--------:|:-----------:|
| Nelder-Mead                               | Nelder-Mead  | ❌     | ❌          | unused   |             |
| Powell                                    | Powell       | ✅     | ❌          | unused   |             |
| Conjugate Gradient                        | CG           | ❌     | ❌          |          |             |
| Broyden–Fletcher–Goldfarb–Shanno          | BFGS         | ❌     | ❌          |          |             |
| Newton Conjugate Gradient                 | Newton-CG    | ❌     | ❌          | required |             |
| Limited-memory BFGS with Bounds           | L-BFGS-B     | ✅     | ❌          |          |             |
| Truncated Newton                          | TNC          | ✅     | ❌          |          |             |
| Constr. Optimization BY Linear Approx.    | COBYLA       | ✅     | ✅*         | unused   |             |
| Constr. Optimization BY Quadratic Approx. | COBYQA       | ✅     | ✅          | unused   |             |
| Sequential Least Squares Programming      | SLSQP        | ✅     | ✅          | ✅       |             |
| Trust-Region Constrained                  | trust-constr | ✅     | ✅          | ✅       | recommended |
| Dog-leg Trust-Region                      | dogleg       | ❌     | ❌          | required | required    |
| Newton Conjugate Gradient Trust-Region    | trust-ncg    | ❌     | ❌          | required | required    |
| Kyrlov "Nearly-Exact" Trust-Region        | trust-krylov | ❌     | ❌          | required | required    |
| "Nearly-Exact" Trust-Region               | trust-exact  | ❌     | ❌          | required | required    |

*The `COBYLA` method only supports inequality constraints (equality constraints are not supported). 

A ✅ in the "Gradient" column indicates that a user-supplied gradient function is supported, but not required.

## Future Work

1. Imporove the Dijkstra's cost function via NLOpt.
2. Add Valgrind to the C++ unit tests.
3. Add the [A* algorithm](https://www.geeksforgeeks.org/a-search-algorithm/) for route planning.
4. Use [Optuna](https://optuna.org/) python package to tune the two parameters of the "Naive" planning algorithm cost function.
5. Add benchmarking to `Planner::PlanRoute` to compare the three different route planners: my _naive_ planner, Dijkstra's and A*. Possibly achieve this via function ["decoration"](https://stackoverflow.com/questions/40392672/whats-the-equivalent-of-python-function-decorators-in-c). 
