from accelerometer import Accelerometer

acc = Accelerometer()


while 1:
	print(list(acc.readGyroData()))