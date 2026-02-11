import time
from collections.abc import Iterator


def sleep(seconds: float) -> Iterator[None]:
    """Interruptable sleep."""
    start = time.time()
    while time.time() - start < seconds:
        time.sleep(0.1)
        yield


def to_list[T](x: T | list[T] | None) -> list[T]:
    if isinstance(x, list):
        return x
    elif x is None:
        return []
    else:
        return [x]
