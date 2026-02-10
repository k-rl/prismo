import time
from collections.abc import Callable, Iterator
from typing import Any, Concatenate, ParamSpec

from .session import run

P = ParamSpec("P")


def run_async[**P](func: Callable[P, Iterator[Any]]) -> Callable[Concatenate[bool, P], Any]:
    def wrapper(blocking: bool = False, *args: P.args, **kwargs: P.kwargs) -> Any:
        def run_func(_session):
            yield from func(*args, **kwargs)

        session = run(run_func)
        if blocking:
            session.join()
        return session

    return wrapper


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
