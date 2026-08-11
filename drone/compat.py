"""Python 3.10+ compatibility shims for DroneKit 2.9.2.

DroneKit was last released in 2020 and still targets Python 2/early 3. Two things
break it on modern interpreters:

1. The ABCs (``MutableMapping`` etc.) were moved out of ``collections`` into
   ``collections.abc`` in 3.3 and the aliases were *removed* in 3.10. DroneKit
   does ``class Parameters(collections.MutableMapping, ...)`` at import time.
2. It imports ``past.builtins.basestring``, which lives in the ``future``
   package (listed in requirements.txt).

Importing this module re-creates the old aliases so ``import dronekit`` works.
It must run *before* dronekit is imported anywhere, which is why
``drone/__init__.py`` imports it first.
"""

import collections
import collections.abc

_MOVED_ABCS = (
    "MutableMapping",
    "Mapping",
    "MutableSequence",
    "Sequence",
    "MutableSet",
    "Set",
    "Iterable",
    "Iterator",
    "Callable",
    "Hashable",
    "Container",
    "Sized",
)


def apply() -> None:
    """Restore the ``collections.*`` ABC aliases removed in Python 3.10."""
    for name in _MOVED_ABCS:
        if not hasattr(collections, name):
            setattr(collections, name, getattr(collections.abc, name))


apply()
