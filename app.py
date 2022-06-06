#	Programación de Microcontroladores
#	Proyecto Final
# 	GUI Python

#	Santiago Rivera	 	-		20269
#	Santiago Penagos	-		20296


import sys
from PyQt5 import uic
from PyQt5.QtWidgets import QMainWindow, QApplication

import serial


# -----------------------------------Variables---------------------------- 

# Valores individuales de Servos 
# Proceso del Encode - Decode (Explicado por primera vez en linea 46)

code_servo1 = 0
code_servo2 = 0
code_servo3 = 0
code_servo4 = 0

#--------------------------------------------------------------------------

class App(QMainWindow):
	def __init__(self):
		super().__init__()
		uic.loadUi("Interfaz_proyectofinal.ui", self)			# Ingreso nombre del archivo .ui
		
		self.ser = serial.Serial(port="COM6", baudrate=9600, timeout=1.0)
		self.ser.close()


		self.servo1.sliderReleased.connect( self.get_value_servo1 )			# Servo 1
		self.servo2.sliderReleased.connect( self.get_value_servo2 )			# Servo 2
		self.servo3.sliderReleased.connect( self.get_value_servo3 )			# Servo 3
		self.servo4.sliderReleased.connect( self.get_value_servo4 )			# Servo 4	


#------------------------------------------------------------------------------------------------------------------------------------------------
	def get_value_servo1(self):							# CONFIGURACIÓN SERVO #1
		value1 = self.servo1.value()
		code_servo1 = int(value1*31/99)					# Se le da un valor a la variable inicial (value1)
														# para que el uC logré identificar que este es el Servo # 1
														# con la segunda variable (code_servo1) multiplicandole un dato 


		self.ser.open()									# Abrimos puerto serial
		self.ser.write( chr(code_servo1).encode() )			# Enviamos valor del slider como caracter
		self.ser.close()								# Cerramos puerto serial (por si se cierra mal no dará errores)

		print("\nValor Final del Servo 1: \n" + str( code_servo1 ))

		self.label_servo1.setText( str( value1 ) )		# Etiqueta que muestra valor que se esta enviando desde la interfaz (No al PIC)



#------------------------------------------------------------------------------------------------------------------------------------------------
	def get_value_servo2(self):							# CONFIGURACIÓN SERVO #2
		value2 = self.servo2.value()
		code_servo2 = int((value2*31/99)+32)			# Se le da un valor a la variable inicial (value2)
														# para que el uC logré identificar que este es el Servo # 2
														# con la segunda variable (code_servo2) multiplicandole un dato y sumandole otro


		self.ser.open()									# Abrimos puerto serial
		self.ser.write( chr(code_servo2).encode() )		# Enviamos valor del slider como caracter
		self.ser.close()								# Cerramos puerto serial (por si se cierra mal no dará errores)

		print("\nValor Final del Servo 2: \n" + str( code_servo2 ))

		self.label_servo2.setText( str( value2 ) )		# Etiqueta que muestra valor que se esta enviando desde la interfaz (No al PIC)



#------------------------------------------------------------------------------------------------------------------------------------------------
	def get_value_servo3(self):							# CONFIGURACIÓN SERVO #3
		value3 = self.servo3.value()
		code_servo3 = int((value3*31/99)+64)			# Se le da un valor a la variable inicial (value3)
														# para que el uC logré identificar que este es el Servo # 3
														# con la segunda variable (code_servo3) multiplicandole un dato y sumandole otro


		self.ser.open()									# Abrimos puerto serial
		self.ser.write( chr(code_servo3).encode() )		# Enviamos valor del slider como caracter
		self.ser.close()								# Cerramos puerto serial (por si se cierra mal no dará errores)

		print("\nValor Final del Servo 3: \n" + str( code_servo3 ))

		self.label_servo3.setText( str( value3 ) )		# Etiqueta que muestra valor que se esta enviando desde la interfaz (No al PIC)



#------------------------------------------------------------------------------------------------------------------------------------------------
	def get_value_servo4(self):							# CONFIGURACIÓN SERVO #4
		value4 = self.servo4.value()
		code_servo4 = int((value4*31/99)+96)			# Se le da un valor a la variable inicial (value4)
														# para que el uC logré identificar que este es el Servo # 4
														# con la segunda variable (code_servo4) multiplicandole un dato y sumandole otro


		self.ser.open()									# Abrimos puerto serial
		self.ser.write( chr(code_servo4).encode() )		# Enviamos valor del slider como caracter
		self.ser.close()								# Cerramos puerto serial (por si se cierra mal no dará errores)

		print("\nValor Final del Servo 4: \n" + str( code_servo4 ))

		self.label_servo4.setText( str( value4 ) )		# Etiqueta que muestra valor que se esta enviando desde la interfaz (No al PIC)



if __name__ == '__main__':
	app = QApplication(sys.argv)
	GUI = App()
	GUI.show()
	sys.exit(app.exec_())