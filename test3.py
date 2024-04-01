def smart_step_traj(step_len_l = 0.1,step_len_r = 0.1):
    xl  = step_len_l;
    xr = step_len_r;
    del_shift_L = -3;
    del_shift_R = 3;
    traj_high_foot = [
        [-0 , 0 , 0.0, 0.0,del_shift_R, del_shift_R],
        [-0 , 2*xl , 0.0, 0.0,del_shift_R, del_shift_R],
        [xl , -2*xl , 0.0, 0.0,del_shift_R*2, del_shift_R*2],
        [-0.0, 0.0, 0, -0, del_shift_L , del_shift_L],
        [-0.0, 0.0, 0, 0, 0.0 , 0.0],
        [-0 , 0 , 0.0, 0.0,del_shift_L, del_shift_L],
        [-0 , 0 , 0.0, -2*xr,del_shift_L, del_shift_L],
        [-0 , 0 , -xr, 2*xr,del_shift_L*2, del_shift_L*2],
        [-0.0, 0.0, 0, -0, del_shift_R , del_shift_R],
        [-0.0, 0.0, 0, 0, 0.0 , 0.0],
    ]  
    return traj_high_foot