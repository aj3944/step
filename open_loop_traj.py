x  = 10;
y_right =  4;
y_left =  4;
traj_walk_foot = []

traj_walk_foot.append([0,0,0,0,0,0])
for i in range(10):
    traj_walk_foot.append([0,0,0,0,i*y_right/10,i*y_right/10])
traj_walk_foot.append([5,0,-5,0,y_right,y_right])

traj_walk_foot.append([0,0,0,0,0,0])
for i in range(10):
    traj_walk_foot.append([0,0,0,0,-i*y_left/10,-i*y_left/10])
traj_walk_foot.append([7,0,-7,0,-y_left,-y_left])


x  = 20;
traj_high_foot = [
    [-x, 2*x, 0.0, 0.0, 0.0 , 0.0],
    [-0.0, 0.0, 0.0, 0.0, 0.0 , 0.0],
    [-0.0, 0.0, x, -2*x, 0.0 , 0.0],
    [-0.0, 0.0, 0.0, 0.0, 0.0 , 0.0],
]


def c_print(traj):
    print("{")
    for k in range(len(traj)):
        print("{",end="")
        for i in range(len(traj[k])):
            print("\t",traj[k][i],end="")
            if i < len(traj[k]) -1:
                print(",",end="")
        if k == len(traj) -1:
            print("}",end="\n")
        else:
            print("},",end="\n")
    print("};")
    print(len(traj))

c_print(traj_high_foot);
