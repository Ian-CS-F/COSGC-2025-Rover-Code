class DirectionalSlopes:
    def __init__(
        self,
        y_pos: tuple[tuple[int, int], tuple[int, int]],
        x_pos: tuple[tuple[int, int], tuple[int, int]],
        y_neg: tuple[tuple[int, int], tuple[int, int]],
        x_neg: tuple[tuple[int, int], tuple[int, int]],
    ):
        self._yPos: float = self._slope(*y_pos)
        self._xPos: float = self._slope(*x_pos)
        self._yNeg: float = self._slope(*y_neg)
        self._xNeg: float = self._slope(*x_neg)

    @property
    def yPos(self) -> float:
        return self._yPos

    @property
    def xPos(self) -> float:
        return self._xPos

    @property
    def yNeg(self) -> float:
        return self._yNeg

    @property
    def xNeg(self) -> float:
        return self._xNeg

    @staticmethod
    def _slope(p1: tuple[int, int], p2: tuple[int, int]) -> float:
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        if dx == 0:
            return float("inf") if dy > 0 else float("-inf")
        return dy / dx
