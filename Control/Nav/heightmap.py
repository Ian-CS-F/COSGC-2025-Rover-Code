from slope import DirectionalSlopes


class Heightmap:
    def __init__(self, rows: int, cols: int):
        self.heights: list[list[int]] = [[0] * cols for _ in range(rows)]
        self.slopes: list[list[DirectionalSlopes | None]] = [
            [None] * cols for _ in range(rows)
        ]
        self.rows = rows
        self.cols = cols
