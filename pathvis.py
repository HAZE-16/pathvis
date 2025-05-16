import tkinter as tk
from tkinter import messagebox
import heapq
import time

CELL_SIZE = 25
GRID_ROWS = 20
GRID_COLS = 30

# Colors
START_COLOR = "green"
END_COLOR = "red"
OBSTACLE_COLOR = "black"
VISITED_COLOR = "light blue"
PATH_COLOR = "yellow"

class PathfindingVisualizer:
    def __init__(self, root):
        self.root = root
        self.root.title("PathVis: Pathfinding Visualizer")

        self.canvas = tk.Canvas(root, width=GRID_COLS*CELL_SIZE, height=GRID_ROWS*CELL_SIZE)
        self.canvas.pack()

        self.grid = [[0 for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
        self.rects = [[None for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]

        self.start = None
        self.end = None

        self.draw_grid()
        self.canvas.bind("<Button-1>", self.on_click)

        self.setup_controls()

    def draw_grid(self):
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                x1, y1 = c*CELL_SIZE, r*CELL_SIZE
                x2, y2 = x1+CELL_SIZE, y1+CELL_SIZE
                rect = self.canvas.create_rectangle(x1, y1, x2, y2, outline="gray", fill="white")
                self.rects[r][c] = rect

    def on_click(self, event):
        r, c = event.y // CELL_SIZE, event.x // CELL_SIZE
        if self.start is None:
            self.start = (r, c)
            self.canvas.itemconfig(self.rects[r][c], fill=START_COLOR)
        elif self.end is None and (r, c) != self.start:
            self.end = (r, c)
            self.canvas.itemconfig(self.rects[r][c], fill=END_COLOR)
        elif (r, c) != self.start and (r, c) != self.end:
            self.grid[r][c] = 1
            self.canvas.itemconfig(self.rects[r][c], fill=OBSTACLE_COLOR)

    def setup_controls(self):
        frame = tk.Frame(self.root)
        frame.pack()

        tk.Button(frame, text="BFS", command=self.run_bfs).pack(side=tk.LEFT, padx=5)
        tk.Button(frame, text="DFS", command=self.run_dfs).pack(side=tk.LEFT, padx=5)
        tk.Button(frame, text="Dijkstra", command=self.run_dijkstra).pack(side=tk.LEFT, padx=5)
        tk.Button(frame, text="A*", command=self.run_astar).pack(side=tk.LEFT, padx=5)
        tk.Button(frame, text="Clear", command=self.clear).pack(side=tk.LEFT, padx=5)

    def get_neighbors(self, node):
        r, c = node
        for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]:
            nr, nc = r+dr, c+dc
            if 0 <= nr < GRID_ROWS and 0 <= nc < GRID_COLS and self.grid[nr][nc] == 0:
                yield (nr, nc)

    def animate(self, came_from):
        node = self.end
        while node != self.start:
            if node != self.end:
                self.canvas.itemconfig(self.rects[node[0]][node[1]], fill=PATH_COLOR)
            node = came_from[node]
            self.canvas.update()
            time.sleep(0.02)

    def run_bfs(self):
        if not self.start or not self.end:
            messagebox.showwarning("Warning", "Set start and end points first.")
            return

        start_time = time.time()
        queue = [self.start]
        visited = set()
        came_from = {}
        visited.add(self.start)

        while queue:
            current = queue.pop(0)
            if current == self.end:
                self.animate(came_from)
                print("BFS Time:", time.time() - start_time)
                return

            for neighbor in self.get_neighbors(current):
                if neighbor not in visited:
                    visited.add(neighbor)
                    came_from[neighbor] = current
                    queue.append(neighbor)
                    if neighbor != self.end:
                        self.canvas.itemconfig(self.rects[neighbor[0]][neighbor[1]], fill=VISITED_COLOR)
                    self.canvas.update()
                    time.sleep(0.01)

    def run_dfs(self):
        if not self.start or not self.end:
            messagebox.showwarning("Warning", "Set start and end points first.")
            return

        start_time = time.time()
        stack = [self.start]
        visited = set()
        came_from = {}
        visited.add(self.start)

        while stack:
            current = stack.pop()
            if current == self.end:
                self.animate(came_from)
                print("DFS Time:", time.time() - start_time)
                return

            for neighbor in self.get_neighbors(current):
                if neighbor not in visited:
                    visited.add(neighbor)
                    came_from[neighbor] = current
                    stack.append(neighbor)
                    if neighbor != self.end:
                        self.canvas.itemconfig(self.rects[neighbor[0]][neighbor[1]], fill=VISITED_COLOR)
                    self.canvas.update()
                    time.sleep(0.01)

    def run_dijkstra(self):
        if not self.start or not self.end:
            messagebox.showwarning("Warning", "Set start and end points first.")
            return

        start_time = time.time()
        heap = [(0, self.start)]
        visited = set()
        came_from = {}
        dist = {self.start: 0}

        while heap:
            cost, current = heapq.heappop(heap)
            if current == self.end:
                self.animate(came_from)
                print("Dijkstra Time:", time.time() - start_time)
                return
            if current in visited:
                continue
            visited.add(current)

            for neighbor in self.get_neighbors(current):
                new_cost = dist[current] + 1
                if neighbor not in dist or new_cost < dist[neighbor]:
                    dist[neighbor] = new_cost
                    heapq.heappush(heap, (new_cost, neighbor))
                    came_from[neighbor] = current
                    if neighbor != self.end:
                        self.canvas.itemconfig(self.rects[neighbor[0]][neighbor[1]], fill=VISITED_COLOR)
                    self.canvas.update()
                    time.sleep(0.01)

    def heuristic(self, a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    def run_astar(self):
        if not self.start or not self.end:
            messagebox.showwarning("Warning", "Set start and end points first.")
            return

        start_time = time.time()
        heap = [(0, self.start)]
        came_from = {}
        g_score = {self.start: 0}
        f_score = {self.start: self.heuristic(self.start, self.end)}
        visited = set()

        while heap:
            _, current = heapq.heappop(heap)
            if current == self.end:
                self.animate(came_from)
                print("A* Time:", time.time() - start_time)
                return
            visited.add(current)

            for neighbor in self.get_neighbors(current):
                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, self.end)
                    heapq.heappush(heap, (f_score[neighbor], neighbor))
                    if neighbor != self.end and neighbor not in visited:
                        self.canvas.itemconfig(self.rects[neighbor[0]][neighbor[1]], fill=VISITED_COLOR)
                    self.canvas.update()
                    time.sleep(0.01)

    def clear(self):
        self.start = None
        self.end = None
        self.grid = [[0 for _ in range(GRID_COLS)] for _ in range(GRID_ROWS)]
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                self.canvas.itemconfig(self.rects[r][c], fill="white")


if __name__ == "__main__":
    root = tk.Tk()
    app = PathfindingVisualizer(root)
    root.mainloop()
