# Copyright 2020 Dirk Thomas
# Copyright 2021 Ruffin White
# Licensed under the Apache License, Version 2.0

import argparse

from colcon_core.verb import get_verb_extensions


def argument_verb_name(value):
    """
    Check if an argument is a valid verb name.

    Used as a ``type`` callback in ``add_argument()`` calls.

    :param str value: The command line argument
    :returns: The verb name
    :raises argparse.ArgumentTypeError: if the value starts with a dash
    """
    if value.lstrip() not in get_verb_extensions():
        raise argparse.ArgumentTypeError('unrecognized verb: ' + value)
    return value.lstrip()
