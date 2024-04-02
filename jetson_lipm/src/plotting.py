import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation




class plotter():
    def __init__(self,funct):

        fig, ax = plt.subplots()

        self.max_x = 50
        self.max_rand = 10

        self.x = np.arange(0, self.max_x)
        # ax.set_ylim(0, max_rand)
        self.y = list(range(0,self.max_x))
        self.line, = ax.plot(self.x, self.y)
        self.funct = funct
        ani = animation.FuncAnimation(
        fig, self.animate, init_func=self.init, interval=100, blit=True, save_count=100)

        plt.show()
    def init(self):  # give a clean slate to start
        self.line.set_ydata(self.y)
        return self.line,

    def animate(self,i):  # update the y values (every 1000ms)
        self.y = self.y[1:]
        self.y.extend(self.funct())
        self.line.set_ydata(self.y)
        return self.line,


# def rand_etter():
#     return np.random.randint(0, 40,1)


# pt = plotter(rand_etter)

