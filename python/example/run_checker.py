"""
Calls the provided checker application with a result from the Supercharger app.
"""
from supercharger.utils.paths import supercharger_build, supercharger_root
from supercharger.utils.subprocess import run_executable


if __name__ == "__main__":
    # Define the route's endpoints
    origin = "Council_Bluffs_IA"
    destination = "Cadillac_MI"

    # Define the application paths
    supercharger_path = supercharger_build() / 'supercharger'
    checker_path = supercharger_root() / 'checker_linux'

    # Run the supercharger application
    route = run_executable(str(supercharger_path), origin, destination)
    print(route)

    # Run the checker application
    result = run_executable(str(checker_path), route)
    print(result)
