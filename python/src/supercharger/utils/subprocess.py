"""

"""
import subprocess

def run_executable(executable_path, *args):
    """Runs an executable with arguments and captures its output."""

    result = subprocess.run(
        [executable_path, *args], capture_output=True, text=True)

    if result.returncode == 0:
        return result.stdout
    else:
        raise Exception(f"Error running executable: {result.stderr}")
