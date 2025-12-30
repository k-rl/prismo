import time

from .gui import run


def run_async(func):
    def wrapper(blocking=False, *args, **kwargs):
        def run_func():
            yield from func(*args, **kwargs)

        runner = run(run_func)
        if blocking:
            runner.join()
        return runner

    return wrapper


def sleep(seconds: float):
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
