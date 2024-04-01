

from step_bot import Bot

if __name__ == "__main__":

    mark_4 = Bot()
    mark_4.home()
    print(mark_4.acc)
    print(mark_4.gyro)
    mark_4.readIMU()

    mark_4.swing_hips(5)
    mark_4.swing_hips(-5)

    mark_4.readIMU()
    print(mark_4.acc)
    print(mark_4.gyro)
    print(mark_4.atilts)
    print(mark_4.gtilts)