#! /usr/bin/python
# -*- coding: UTF-8 -*-

# Importamos el módulo pygtk y le indicamos que use la versión 2 (en caso de
# que existan varias versiones de pygtk instaladas en el sistema)
#import pygtk
#pygtk.require("2.0")

# Importamos el módulo de gtk para poder acceder a los controles de Gtk+

from gi.repository import Gtk, Gdk

# ---------------------------------------

import sys
import yaml
import roslib
roslib.load_manifest("canusb");
from std_msgs.msg import Int16
from std_msgs.msg import String

import struct
import rospy
from _CAN import CAN

import serial
import time
import threading

# INICIO CODIGO TEST

class MainWin:
    # Primero definimos como sera la ventana:
    def __init__(self):
    
        # Creamos una ventana toplevel (o sea que esta al frente de todas las
        # ventanas) llamada "main_win" y fijamos su titulo como "Ejemplo 1"

        main_win = Gtk.Window()
        main_win.set_title("SARA")
	main_win.set_default_size(500,470)
	self.overlay = Gtk.Overlay()
        main_win.add(self.overlay)
        self.background = Gtk.Image.new_from_file('./src/canusb/script/SillaRuedas.jpg')
        self.overlay.add(self.background)
        self.grid = Gtk.Grid()
	#main_win.modify_bg(gtk.STATE_NORMAL, gtk.gdk.color_parse("red"))
        
        # A main_win le conectamos una señal (destroy), esto hará que cada
        # vez que se presione el botón salir (la cruz del manejador de
        # ventanas) se llamará al método on_quit que cerrara la ventana
        main_win.connect("destroy", self.on_quit)
        
        # Para agregar widgets (controles como botones, etiquetas, etc.) a la
        # ventana, primero es necesario crear contenedores como cajas que
        # contengan las widgets. En este ejemplo creamos una caja vertical con
        # un espacio entre widgets de 5px y con la propiedad homogéneo en False

	#marco=gtk.Frame()
	#marco.modify_bg(gtk.STATE_NORMAL, gtk.gdk.color_parse("red"))
        vbox = Gtk.VBox(False, 0)
	
	hbox_D = Gtk.HBox(True, 0)
	hbox_D.set_size_request(500,50)
	
	hbox_C= Gtk.HBox(False, 0)
	hbox_C.set_size_request(500,350)
	hbox_C.set_border_width(10)

	vbox_I = Gtk.VBox(True, 0)

	vbox_C = Gtk.VBox(False, 0)
	vbox_C.set_size_request(250,350)
	vbox_C.set_border_width(10)

	vbox_D = Gtk.VBox(True, 0)

	hbox_T = Gtk.HBox(True, 0)
	hbox_T.set_size_request(500,50)

	sepaV1=Gtk.VSeparator()
	sepaV2=Gtk.VSeparator()
	sepaV3=Gtk.VSeparator()
	sepaV4=Gtk.VSeparator()

	sepaH1=Gtk.HSeparator()
	sepaH2=Gtk.HSeparator()
	sepaH3=Gtk.HSeparator()
	sepaH4=Gtk.HSeparator()

        
        # Creamos una etiqueta con el texto "Hola mundo", se usa la palabra
        # reservada "self" de python para poder hacer referencia a esta
        # etiqueta desde otros métodos
	self.label_SDI = Gtk.Label("SDI = 255")
	self.label_SDI.set_halign(Gtk.Align.END)
	self.label_SDD = Gtk.Label("SDD = 255")
	self.label_SDD.set_halign(Gtk.Align.START)

	self.label_SLID = Gtk.Label("SLID = 255")
	self.label_SLIT = Gtk.Label("SLIT = 255")

	self.label_SLDD = Gtk.Label("SLDD = 255")
	self.label_SLDT = Gtk.Label("SLDT = 255")

	self.label_STI = Gtk.Label("STI = 255")
	self.label_STI.set_halign(Gtk.Align.END)
	self.label_STD = Gtk.Label("STD = 255")
	self.label_STD.set_halign(Gtk.Align.START)        

        # Creamos un cuadro donde escribir (una entrada de texto en blanco)
        # y luego le conectamos la señal "activate" que llama al método
        # "on_button1_clicked", esto producirá que cuando se haga click en el
        # botón Ok (que se creara mas adelante) la entrada de texto reaccione

        #self.entry = gtk.Entry()
        #self.entry.connect("activate", self.on_button1_clicked)
        
        # Ahora creamos el botón, que sera el botón OK del inventario de
        # botones de GNOME.
        # Y luego le indicamos al botón que cuando le hagan click emita la
        # señal "clicked" que llamará a "on_button1_clicked"

        button = Gtk.Button(stock=Gtk.STOCK_STOP)
        button.connect("clicked", self.on_button_clicked)

	# Creamos vertical scale Derecho
	ad1 = Gtk.Adjustment(0, -1, 1, 0.05, 0.05, 0)
        self.v_scale = Gtk.VScale(adjustment=ad1)
	self.v_scale.set_digits(2)
	self.v_scale.set_draw_value(True)
	self.v_scale.set_inverted(True)
	self.v_scale.set_show_fill_level(True)
	
	# Cuando cambiamos valor sccaleD

        self.v_scale.connect("value_changed", self.scale)

	# Creamos vertical scale Izquiedo
	ad2 = Gtk.Adjustment(0, -3, 3, 0.1, 0.1, 0)
        self.h_scale = Gtk.HScale(adjustment=ad2)
	self.h_scale.set_digits(1)
	self.h_scale.set_draw_value(True)
	self.h_scale.set_inverted(False)
	self.h_scale.set_show_fill_level(True)

	# Cuando cambiamos valor sccaleI

	self.h_scale.connect("value_changed", self.scale)
        
        # Ahora que ya creamos las widgets (la etiqueta, la entrada de texto y
        # el botón) hay que añadirlos a la caja vertical creada anteriormente
        
        # Primero le añadimos la etiqueta llamada label a la caja vertical
        #vbox.add(self.label)

	#marco.add(self.label)
	self.grid.add(vbox)

	vbox.pack_start(hbox_D, True, True, 0)
	
	hbox_D.pack_start(self.label_SDI, True, True, 0)
	hbox_D.pack_start(sepaV1, True, True, 0)
	hbox_D.pack_start(self.label_SDD, True, True, 0)

	vbox.pack_start(sepaH1, True, True, 0)

        # Luego añadimos al inicio de la segunda fila la entrada de texto
        # activando las propiedades de expandir, rellenar y espaciado.
        
	#vbox.pack_start(self.entry, True, True, 0)
        
        # Finalmente en la tercer fila agregamos el botón.

        #vbox.pack_start(button, True, False, 0)
	
	vbox.pack_start(hbox_C,True,True,0)
	
	hbox_C.pack_start(vbox_I, True, True, 10)
	
	vbox_I.pack_start(self.label_SLID, True, True, 0)
	vbox_I.pack_start(sepaH2, True, True, 0)
	vbox_I.pack_start(self.label_SLIT, True, True, 0)

	hbox_C.pack_start(sepaV2, True, True, 0)

	hbox_C.pack_start(vbox_C, True, True, 0)
	vbox_C.pack_start(self.v_scale, True, True, 0)
	vbox_C.pack_start(self.h_scale, True, True, 0)
	vbox_C.pack_start(button, False, False, 0)

	hbox_C.pack_start(sepaV3, True, True, 0)

	hbox_C.pack_start(vbox_D, True, True, 10)

	vbox_D.pack_start(self.label_SLDD, True, True, 0)
	vbox_D.pack_start(sepaH3, True, True, 0)
	vbox_D.pack_start(self.label_SLDT, True, True, 0)

	vbox.pack_start(sepaH4, True, True, 0)

	vbox.pack_start(hbox_T, True, True, 0)
	
	hbox_T.pack_start(self.label_STI, True, True, 0)
	hbox_T.pack_start(sepaV4, True, True, 0)
	hbox_T.pack_start(self.label_STD, True, True, 0)

        self.overlay.add_overlay(self.grid)
        # Ahora agregamos la caja vertical a la ventana y luego se muestra
        # la caja (y todo lo que contiene) en la ventana principal.
        #main_win.add(vbox)
        main_win.show_all()


    # Ahora dentro de nuestra clase principal "MainWin" tenemos que definir
    # que hacen cada uno de los métodos que se llamaron anteriormente
    
    # Definimos el método "on_button_clicked" (cuando pulsamos boton)
    # ESTE BOTON SE ENCARGA DE PARAR LOS DOS MOTORES EN CASO DE PULSARLO.
    def on_button_clicked(self, widget):
	self.v_scale.set_value(0)
	self.h_scale.set_value(0)
	self.enviar(0,0)

    #Definimos el método "scale" en la que entra cuando cambiamos el valor de los scale
    def scale(self, widget):
        V = self.v_scale.get_value()
	R = self.h_scale.get_value()
	wD= ((V/0.155) + (R*1.693))
	wI= ((V/0.155) - (R*1.693))
	
	#"{0:.0f}".format(wI*10)
	datoD= int(wD*10)
	datoI= int(wI*10)
	if datoD < 0:
	    datoD= datoD + 256
	if datoI < 0:
	    datoI= datoI + 256
	self.enviar(datoD,datoI)
    
    # Ahora se define el método "on_quit" que destruye la aplicación
    def on_quit(self, widget):
        Gtk.main_quit()

    #FUNCION ENCARGADA DE HACER LA SINCRONIZACION PARA QUE APAREZCA MENSAJE "SINCRONIZADO" EN SILLA

    def sincronizar(self, sinc):
	if sinc == 3:
	    self.enviar(0,0)

    # FUNCION QUE IDENTIFICA LAS TRAMAS CAN QUE NECESITAMOS COMO LA DE SENSORES Y QUE MODIFICA LAS ETIQUETAS DE LA INTERFACE

    def lectura(self, rx):

	if rx.stdId == 273:
	    (self.modoPC,)= struct.unpack('B', rx.data[:1])
	    self.sincronizar(self.modoPC)

	elif rx.stdId == 513:
	    (self.STD,) = struct.unpack('H', rx.data[:2])
	    (self.SDI,) = struct.unpack('H', rx.data[2:4])
	    (self.SLIT,) = struct.unpack('H', rx.data[4:6])
	    (self.SLDD,) = struct.unpack('H', rx.data[6:8])

	    self.label_STD.set_text("STD = %d cm" % self.STD)
	    self.label_SDI.set_text("SDI = %d cm" % self.SDI)
	    self.label_SLIT.set_text("SLIT = %d cm" % self.SLIT)
	    self.label_SLDD.set_text("SLDD = %d cm" % self.SLDD)

	elif rx.stdId == 514:
	    (self.SLDT,) = struct.unpack('H', rx.data[6:8])
	    (self.STI,) = struct.unpack('H', rx.data[4:6])
	    (self.SLID,) = struct.unpack('H', rx.data[2:4])
	    (self.SDD,) = struct.unpack('H', rx.data[:2])

	    self.label_SLDT.set_text("SLDT = %d cm" % self.SLDT)
	    self.label_STI.set_text("STI = %d cm" % self.STI)
	    self.label_SLID.set_text("SLID = %d cm" % self.SLID)
	    self.label_SDD.set_text("SDD = %d cm" % self.SDD)


    # FUNCION ENVIAR ENCARGADA DE ENVIAR LOS DATOS A LOS MOTORES.

    def enviar(self,datoD,datoI):
	#pub = rospy.Publisher('cantx', CAN, queue_size=100)
    	if not rospy.is_shutdown():
    	     msg = CAN()
    	     msg.stdId = 288
    	     msg.extId = -1
    	     msg.data = struct.pack('B', datoI) + struct.pack('B', datoD) + struct.pack('B', 0) + struct.pack('B', 0) + 'A' + struct.pack('B', 0) + struct.pack('B', 0) + struct.pack('B', 0)
	     try:
    	         pub.publish( msg )
	     except rospy.ROSInterruptException: pass
	
    	     print msg


# Para terminar iniciamos el programa

if __name__ == "__main__":

    # Iniciamos la clase.
    win=MainWin()
    # Además iniciamos el método gtk.main, que genera un ciclo que se utiliza
    # para recibir todas las señales emitidas por los botones y demás widgets.

# INICIO CODIGO TEST
    try:
	pub = rospy.Publisher('cantx', CAN, queue_size=100)
	rospy.Subscriber("canrx", CAN, win.lectura)
    	rospy.init_node('canusb_test', anonymous=True)
	Gtk.main()
        #talker()
	
    except rospy.ROSInterruptException: pass

# FIN CODIGO TEST

