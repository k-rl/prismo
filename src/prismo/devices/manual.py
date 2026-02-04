from numbers import Real


class Objective:
    def __init__(self, name: str, zoom: Real = 1):
        self.name = name
        self._zoom = zoom

    def wait(self):
        pass

    @property
    def state(self) -> Real:
        return self._zoom

    @state.setter
    def state(self, new_state: Real):
        self._zoom = new_state

    @property
    def zoom(self) -> Real:
        return self._zoom
