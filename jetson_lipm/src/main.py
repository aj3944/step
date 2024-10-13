

from step_bot import Bot
import time
from plotting import plotter


# def rand_etter():
#     return np.random.randint(0, 40,1)





if __name__ == "__main__":



    # app = QtWidgets.QApplication([])
    # main = MainWindow([rand_etter,rand_etter,rand_etter])




    # timer = QtCore.QTimer()
    # timer.setInterval(300)
    # timer.timeout.connect(main.updater)
    # timer.start()


    # main.show()
    # app.exec()

    #pt = plotter(rand_etter)


    mark_4 = Bot()
    mark_4.home()
    time.sleep(3)
    print(mark_4.acc)
    print(mark_4.gyro)
    mark_4.readIMU()
    # mark_4.lift_leg(5,1)
    # mark_4.lift_leg(5,0)

    step_l = 0;

    half_swing = 4;

    leg_raise =  0;


    while 1:
        # mark_4.swing_hips(-2)
        # mark_4.exec_ik(0.03)
        mark_4.swing_hips(half_swing)
        mark_4.exec_ik(0.06)
        # mark_4.swing_hips(-4)
        # mark_4.exec_ik(0.01)

        # mark_4.readIMU()
        # print(mark_4.acc)
        # print(mark_4.gyro)
        # print(mark_4.atilts)
        # print(mark_4.gtilts)
        mark_4.swing_thighs(-step_l)
        mark_4.lift_leg(leg_raise,0)
        # mark_4.abduct_leg(-5,0)
        mark_4.exec_ik(0.05)



        mark_4.lift_leg(-leg_raise,0)
        mark_4.exec_ik(0.01)


        mark_4.swing_hips(-2*half_swing)    
        # mark_4.abduct_leg(5,0)
        mark_4.exec_ik(.1)    


     



        mark_4.swing_thighs(2*step_l)
        mark_4.lift_leg(leg_raise,1)
        mark_4.exec_ik(0.05)
        mark_4.lift_leg(-leg_raise,1)
        # mark_4.kick_leg(45,1)
        mark_4.exec_ik(0.04)

        mark_4.home()

        # time.sleep(0.1)
        # time.sleep(.4)
