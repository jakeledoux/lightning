import re
from simpleeval import simple_eval
import sys

# Regex patterns
kwarg_pattern = re.compile(r'-{0,2}(\w+)=(\S+)')

def get_command(idx):
    if len(sys.argv) > idx + 1:
        return sys.argv[idx + 1].lower().strip()
    else:
        return False


def get_kwarg(key, default):
    for arg in sys.argv[1:]:
        match = kwarg_pattern.match(arg)
        if match:
            arg_key, arg_value = match.groups()
            if arg_key == key:
                try:
                    return simple_eval(arg_value)
                except SyntaxError:
                    return arg_value
    return default
