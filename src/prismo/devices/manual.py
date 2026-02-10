class Objective:
    def __init__(self, name: str, zoom: float = 1):
        self.name = name
        self._zoom = zoom

    def wait(self):
        pass

    @property
    def state(self) -> float:
        return self._zoom

    @state.setter
    def state(self, new_state: float):
        self._zoom = new_state

    @property
    def zoom(self) -> float:
        return self._zoom
