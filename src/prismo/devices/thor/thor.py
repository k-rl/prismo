from . import thor_lib


class Light:
    def __init__(self, name: str, port: str):
        self.name = name
        self._device_id = thor_lib.init(port)

    def wait(self):
        # TODO: Implement this.
        pass

    @property
    def state(self) -> int:
        return int(thor_lib.get_amps(self._device_id) * 200)

    @state.setter
    def state(self, new_state: float):
        new_state = new_state / 200
        thor_lib.set_amps(self._device_id, new_state)
        thor_lib.toggle(self._device_id, new_state > 0)
