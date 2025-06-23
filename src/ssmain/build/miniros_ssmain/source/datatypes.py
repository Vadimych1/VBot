from miniros import datatypes

class Task(datatypes.Dict):
    @staticmethod
    def encode(data):
        return super().encode(data)