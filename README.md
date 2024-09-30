# Tesla Coding Challenge

## Future Work

1. (__DONE__) Add [Dijkstra's algorithm](https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-greedy-algo-7/) for route planning.
2. Add [A* algorithm](https://www.geeksforgeeks.org/a-search-algorithm/) for route planning.
3. (__DONE__) Create python bindings ([`pybind11`](https://pybind11.readthedocs.io/en/stable/)) for the `RoutePlanner` class.
4. Use [Optuna](https://optuna.org/) python package to tune the two parameters of the "Naive" planning algorithm cost function.
5. Add benchmarking to `PlanningAlgorithm::PlanRoute` to compare the three different route planners: my _naive_ planner, Dijkstra's and A*. Possibly achieve this via function ["decoration"](https://stackoverflow.com/questions/40392672/whats-the-equivalent-of-python-function-decorators-in-c).
6. Design an optimization problem to further refine the solution provided by Dijkstra's and/or A*.

## Problem Statement
Your objective is to construct a search algorithm to find the minimum time path through the tesla network of supercharging stations. Each supercharger will refuel the vehicle at a different rate given in km/hr of charge time. Your route does not have to fully charge at every visited charger, so long as it never runs out of charge between two chargers. You should expect to need __no more than 4-6 hours__ to solve this problem. We suggest implementing a quick brute force method before attempting to find an optimal routine.

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
./checker_osx “Council_Bluffs_IA, Worthington_MN, 1.18646, Albert_Lea_MN, 1.90293, Onalaska_WI, 0.69868, Mauston_WI, 1.34287, Sheboygan_WI, 1.69072, Cadillac_MI”
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

### Building
Build the `supercharger` application by following the standard CMake build process:

```
cd supercharger
mkdir build && cd build
cmake ..
make
```

### Running the `supercharger` Application
Run the `supercharger` application by passing any two valid Supercharger locations to the executable, e.g.

```
./supercharger Council_Bluffs_IA Cadillac_MI
```

### Results
So far, the following results have been obtained. The _reference result_ provided by the `checker_linux` application is included for comparison. The last two columns indicate the percent improvement over the reference result, and the time saved compared to the reference result.

TODO: Add profiling for each algorithm.

| Algorithm                           | Cost     | Runtime | Pct Imprv. | Time Saved  |
|:------------------------------------|:--------:|:-------:|:----------:|:-----------:|
|                                     | [hrs]    | [msecs] | [%]        | [mins:secs] |
| _Naive_ Route Planner               | 18.1017  | -       | -2.9185    | +50:55      |
| Dijkstra's Algorithm                | 17.2548  | -       | -0.0096    | +00:06      |
| _Reference_ Result                  | 17.2531  | -       | -          | -           |
| Dijkstra's + _Naive_ Optimizer      | 17.0697  | -       | 1.0630     | -11:00      | 
| Dijkstra's + Nonlinear Optimization | 16.8438  | -       | 2.3723     | -24:33      |

The following figure illustrates how the constrained nonlinear optimization 
scheme maximizes the charging time at nodes with relatively high charging rates, and decreases the charging time for nodes with low charging rates.

![charging durations](/figs/charging_durations.png "Charging Durations")


### Constrained Nonlinear Optimization with SciPy `minimize`
Adding python bindings to the project via the `pybind11` package has enabled experimentation with various types of optimization algorithms, and has given me additional insight about the use cases and limiations of specific methods. The table below details the types of problems that each optimization algorithm is (and isn't) suitable for.

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
