from slope import DirectionalSlopes


class Heightmap:
    def __init__(self, rows: int, cols: int):
        self.heights: list[list[int]] = [[0] * cols for _ in range(rows)]
        self.slopes: list[list[DirectionalSlopes | None]] = [
            [None] * cols for _ in range(rows)
        ]
        self.rows = rows
        self.cols = cols

    def compute_slopes(self) -> None:
        """Recompute directional slopes for every cell from current height data."""
        for r in range(self.rows):
            for c in range(self.cols):
                h = self.heights[r][c]
                h_ypos = self.heights[r + 1][c] if r + 1 < self.rows else h
                h_yneg = self.heights[r - 1][c] if r > 0 else h
                h_xpos = self.heights[r][c + 1] if c + 1 < self.cols else h
                h_xneg = self.heights[r][c - 1] if c > 0 else h
                self.slopes[r][c] = DirectionalSlopes(
                    y_pos=((0, h), (1, h_ypos)),
                    x_pos=((0, h), (1, h_xpos)),
                    y_neg=((0, h), (1, h_yneg)),
                    x_neg=((0, h), (1, h_xneg)),
                )
