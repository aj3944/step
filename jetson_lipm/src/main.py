

from step_bot import Bot

if __name__ == "__main__":

    mark_4 = Bot()
    mark_4.home()
    print(mark_4.acc)
    print(mark_4.gyro)
    mark_4.readIMU()

    mark_4.swing_hips(-4)
    mark_4.exec_ik(0.01)
    mark_4.swing_hips(7)
    mark_4.exec_ik(0.1)

    mark_4.readIMU()
    print(mark_4.acc)
    print(mark_4.gyro)
    print(mark_4.atilts)
    print(mark_4.gtilts)
    mark_4.swing_thighs(-5)
    mark_4.lift_leg(5,1)
    mark_4.exec_ik(0.1)
    