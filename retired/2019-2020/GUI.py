import tkinter as tk


class GUI:
    canvas = None
    robot = None
    root = None

    xMax = 0
    yMax = 0

    def __init__(self, height, width):
        self.xMax = width
        self.yMax = height

        self.initializeGrid()

    def animateRobotMovingGivenCoordList(self, coords):
        for x, y in coords:
            self.moveRobot(x, y)

    def moveRobot(self, x, y):
        self.canvas.coords(self.robot, x - 5, y - 5, x + 5, y + 5)
        self.canvas.update()
        self.canvas.after(300)

    def initializeGrid(self):
        self.root = tk.Tk()
        self.canvas = tk.Canvas(
            self.root, height=self.yMax, width=self.xMax, bg="white"
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", self.createGrid)
        self.robot = self.canvas.create_rectangle(0, 0, 5, 5, fill="red")

    def initializeMap(self):
        df = pd.read_csv("longandlats.csv")

        # BBox = ((df.longitude.min(),   df.longitude.max(),
        #          df.latitude.min(), df.latitude.max()))
        # #         > (42.4596, 42.4642, -76.5119, -76.5013)

        BBox = [-76.5119, -76.5013, 42.4596, 42.4642]

        ruh_m = plt.imread("map.png")

        fig, ax = plt.subplots(figsize=(8, 7))
        ax.scatter(df.longitude, df.latitude, zorder=1, alpha=1, c="r", s=10)
        ax.set_title("Cayuga Lake Shore")
        ax.set_xlim(BBox[0], BBox[1])
        ax.set_ylim(BBox[2], BBox[3])

        ax.imshow(ruh_m, zorder=0, extent=BBox, aspect="equal")

        # polys1 = geopandas.GeoSeries([Polygon([(-76.5033,42.4636),(-76.5023,42.4641),(-76.5013,42.4646),(-76.5113,42.4601)])])
        # df1 = geopandas.GeoDataFrame({'geometry': polys1, 'df1':[-76.5028,42.4638]})
        # ax1 = df1.plot(color='blue')

        plt.show()

    def createGrid(self, event=None):
        w = self.canvas.winfo_width()  # Get current width of canvas
        h = self.canvas.winfo_height()  # Get current height of canvas
        self.canvas.delete("grid_line")  # Will only remove the grid_line

        # Creates all vertical lines at intevals of 100
        for i in range(0, w, 10):
            self.canvas.create_line([(i, 0), (i, h)], tag="grid_line")

        # Creates all horizontal lines at intevals of 100
        for i in range(0, h, 10):
            self.canvas.create_line([(0, i), (w, i)], tag="grid_line")
