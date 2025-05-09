import matplotlib.pyplot as plt

def graph_data(x_array, y_array):
    plt.plot(x_array, y_array)
    plt.xlabel("Time Since Start (s)")
    plt.ylabel("Depth (m)")
    plt.grid(True)
    plt.savefig("float_depth_graph.png")
    plt.show()

if __name__ == "__main__":
    graph_data([1,2,3,4,5], [0,3,5,3,0])