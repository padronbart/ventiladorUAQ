from kivy.app import App
from kivy.uix.floatlayout import FloatLayout
from kivy.core.window import Window
from kivy.clock import Clock
from threading import Thread
from queue import Queue
import codecs
import serial
#import RPi.GPIO as GPIO
from kivy.uix.label import Label
from kivy.garden.graph import MeshLinePlot,LinePlot

#pin = 40
#GPIO.setmode(GPIO.BOARD)
#GPIO.setup(pin, GPIO.OUT)
#GPIO.output(pin, GPIO.HIGH)  # HIGH should turn the buzzer off

class TestGUI(FloatLayout):

    setpoint_list = []
    actual_list = []
    coms_on = False
    serial_read = False
    def __init__(self):
        super(TestGUI, self).__init__()
        #Window.clearcolor = (1, 1, 1, 1)
        Window.clearcolor = (.14, .18, .2, 1)
        self.ser = serial.Serial()
        self.ser.port = '/dev/ttyUSB0'
        self.ser.baudrate = 115200

        self.setpoint_plot = LinePlot(line_width=2.5,color=[0, 1, 0.8, 1])
        self.act_pos_plot = LinePlot(line_width=2.5,color=[0, 1, 0.8, 1])


        self.ids.pos_graph.add_plot(self.setpoint_plot)
        self.ids.pos_graph.add_plot(self.act_pos_plot)


        self.setpoint_plot.color = [0, 1, 0.8, 1]
        self.act_pos_plot.color = [0.2196, 0.6431, 0.8, 1]


        self.graph_clock = Clock.schedule_interval(self.get_plot_points, (1/100))
        self.graph_clock.cancel()
        self.slider_clock = Clock.schedule_interval(self.pid_listener, (1 / 10))

        self.q = Queue()
        self.dt = 0
        self.ts = Queue()
        self.maxt = 1000
        self.tdata = [0]
        self.ydata = [0]

    def pid_listener(self, other):
            self.p_val = self.ids.p_slider.value
            self.d_val = self.ids.d_slider.value


    def coms_btn_press(self):
        if self.coms_on:
            self.coms_on = False
            self.ids.coms_btn.background_color = [1,.73,0,1]
            self.graph_clock.cancel()
            self.ids.coms_btn.text = "ENCENDER"
            self.serial_read = False
            print("OFF")


        else:
            self.coms_on = True
            self.ids.coms_btn.background_color = [0,.8,.42,1]
            self.graph_clock()
            self.ids.coms_btn.text = "APAGAR"
            self.serial_read = True
            print("ON")
            get_serial_thread = Thread(target=self.get_serial)
            get_serial_thread.daemon = True
            get_serial_thread.start()

    def get_plot_points(self, other):
        # Reset position plots every 1000 data points
        if (len(self.actual_list)) > 500:
            self.actual_list = []
            self.setpoint_list = []
            self.tdata = [0]
            self.ydata = [0]
            self.dt = 0
            temp_time = 0
            temp_vals = 0
            #self.actual_list.append((float(temp_time), temp_vals))
            #self.act_pos_plot.points = list(self.actual_list)



        while not self.q.empty():
            t = self.tdata[-1] + self.dt
            temp_vals = self.q.get()
            temp_time = self.ts.get()
            #presion_1 = (0.77504 * temp_vals)  - 28.57
            presion_1 = ((0.77504 * temp_vals) - 28.57)*0.145
            #if (presion_1>30):
            #    GPIO.output(pin, GPIO.LOW) # peep
            #else:
            #    GPIO.output(pin, GPIO.HIGH)
            #print(presion_1)
            self.tdata.append(temp_time)
            self.ydata.append(presion_1)
            #print(temp_vals)
            self.actual_list.append((float(temp_time), float(presion_1)))
            #self.setpoint_list.appendappend((temp_vals))
            #self.actual_list.append((float(temp_vals),float(temp_vals)),(float(temp_vals),float(temp_vals)))
            #self.actual_list.append((self.dt,float(temp_vals)))

        self.act_pos_plot.points = list(self.actual_list)
        #self.setpoint_plot.points = self.setpoint_list[:]

    def toggle_tabs(self):
        if self.ids.tab_panel.current_tab == self.ids.error_tab:
            self.ids.tab_panel.switch_to(self.ids.pos_tab)
        else: self.ids.tab_panel.switch_to(self.ids.error_tab)


    def get_serial(self):
        _FLAG_RX = 170
        flagcom = 0
        escV = 5 / 1024
        try: # Try and open serial port
            if not self.ser.is_open:
                self.ser.open()

        except: # Serial Port isn't connected
            print("No Serial Found!")

        else: # If serial port is opened, start scanning for data
            while self.serial_read:
                received1 = bytes.hex(self.ser.read(1))
                received = int(codecs.encode(received1), 16)
                if len(received1) > 0:
                    if flagcom != 0:
                        flagcom += 1
                    if flagcom == 0 and received == _FLAG_RX:
                        flagcom = 1
                        sf1 = 0
                        sf2 = 0
                        sf3 = 0
                        sp1 = 0
                        sp2 = 0
                        ecg = 0
                    if flagcom == 2:
                        sf1 = received
                    if flagcom == 3:
                        sf2 = received
                    if flagcom == 4:
                        sf3 = received
                    if flagcom == 5:
                        self.sp1 = received
                        self.sp1 = sp1 << 8
                    if flagcom == 6:
                        self.sp1 += received
                    if flagcom == 7:
                        self.sp2 = received
                        self.sp2 = self.sp2 << 8
                    if flagcom == 8:
                        self.sp2 += received
                    if flagcom == 9:
                        self.sp3 = received
                        self.sp3 = self.sp3 << 8
                    if flagcom == 10:
                        self.sp3 += received
                        self.q.put(self.sp3)
                        self.dt +=0.5
                        self.ts.put(self.dt)
                        #print(self.dt)
                        #presion_1 = (0.0063* sp1 )*escV + 0.18
                        presion_2 = escV * sp2
                        presion_3 = escV * self.sp3
                        T_on = int(self.d_val)
                        T_off = 10-T_on
                        pwmf1= chr(T_on << 4 | T_off)
                        pwmf2 = int(self.p_val)
                        pwm_2 = pwmf1
                        pwm_1 = chr(pwmf2)
                        start = chr(170)
                        self.ser.write(start.encode('UTF-8'))
                        self.ser.write(pwm_1.encode('UTF-8'))
                        self.ser.write(pwm_1.encode('UTF-8'))
                        self.ser.write(pwm_2.encode('UTF-8'))
                        flagcom = 0



class TestApp(App):
    def build(self):
        return TestGUI()


if __name__ == '__main__':
    TestApp().run()
