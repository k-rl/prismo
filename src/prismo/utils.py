def to_list[T](x: T | list[T] | None) -> list[T]:
    if isinstance(x, list):
        return x
    elif x is None:
        return []
    else:
        return [x]
