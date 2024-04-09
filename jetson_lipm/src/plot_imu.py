
import io
import time
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.animation


BUFFER_LEN = 64*2
DATA_FILENAME = "data.txt"
PLOT_LIMIT = 20
ANIM_FILENAME = "video.gif"


fig_acc_0, ax_acc_0 = plt.subplots(2, 3, figsize=(10,8))
ax_acc_0[0][0].set_title("ACC X")
ax_acc_0[0][0].set_xlabel("time / s")
ax_acc_0[0][0].set_ylabel("X_acc")


# fig_acc_1, ax_acc_1 = plt.subplots(1, 2, figsize=(10,8))
# ax_acc_1.set_title("ACC Y")
# ax_acc_1.set_xlabel("time / s")
# ax_acc_1.set_ylabel("Y_acc")
# ax.set_ylim([0, 1])


timestamps = []
values = [[],[],[],[],[],[]]
# values = [[],[],[]]

def read_all(filename):
    with open(filename) as openfileobject:
        for line in openfileobject:
            split = line.split(":")
            print(split)      

            x = split[0]
            y = split[1]
            x = float(x)
            y = eval(y)
            timestamps.append(x)
            for v in range(len(y)):
                values[v].append(y[v])


def get_data(filename, buffer_len, delay=0.2):
    with open(filename, "r") as f:
        f.seek(0, io.SEEK_END)
        if delay:
            time.sleep(delay)
        data = f.readline()
    return data


def animate(i, xs, ys, limit=PLOT_LIMIT, verbose=True):
    # grab the data
    try:
        data = get_data(DATA_FILENAME, BUFFER_LEN)
        print(data)
        if verbose:
            print("[VERBOSE]",data)
        split = data.split(":")
        print(split)
        x = split[0]
        y = split[1]
        x = float(x)

        print("YY",y)
        y = eval(y)
        # y = float(y[0])
        print(y)
        if x > xs[-1] and not y == None:
            # Add x and y to lists
            xs.append(x)
            ys.append(y)
            # Limit x and y lists to 10 items
            xs = xs[-limit:]
            ys = ys[-limit:]
            # print(ys)
        else:
            print(f"W: {time.time()} :: STALE!")
    except ValueError:
        print(f"W: {time.time()} :: EXCEPTION!")
    else:
        # Draw x and y lists
        ax_acc_0[0][0].clear()
        ax_acc_0[0][1].clear()
        # ax.set_ylim([0, ])
        # print(ys)
        y_acc_x = [ k[0] for k in ys]
        y_acc_y = [ k[1] for k in ys]
        ax_acc_0[0][0].plot(xs, y_acc_x)
        ax_acc_0[0][1].plot(xs, y_acc_y)
        # ax_acc_1.plot(xs, y_acc_y)


realtime = False

# save video (only to attach here) 
#anim = mpl.animation.FuncAnimation(fig, animate, fargs=([time.time()], [None]), interval=1, frames=3 * PLOT_LIMIT, repeat=False)
#anim.save(ANIM_FILENAME, writer='imagemagick', fps=10)
#print(f"I: Saved to `{ANIM_FILENAME}`")
# show interactively
if realtime:
    anim = mpl.animation.FuncAnimation(fig_acc_0, animate, fargs=([time.time()], [[0,0,0]]), interval=1)
else:
    read_all(DATA_FILENAME)
    ax_acc_0[0][0].plot(timestamps, values[0])
    ax_acc_0[0][1].plot(timestamps, values[1])
    ax_acc_0[0][2].plot(timestamps, values[2])
    ax_acc_0[1][0].plot(timestamps, values[3])
    ax_acc_0[1][1].plot(timestamps, values[4])
    ax_acc_0[1][2].plot(timestamps, values[5])


plt.show()
plt.close()
