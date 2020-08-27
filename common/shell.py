import subprocess

from typing import List

def run_cmd(cmd: List[str], shell: bool = False):
    return subprocess.check_output(cmd, encoding='utf8', shell=shell).strip()


def run_cmd_default(cmd: List[str], default=None):
  try:
    return run_cmd(cmd)
  except subprocess.CalledProcessError:
    return default


