from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis, Icon
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, Matrix


class MiDriveBase:
    """
    Clase con nuestra propia DriveBase para
    mover el robot con los parámetros que queramos
    """
    def __init__(self, drivebase: DriveBase, hub: PrimeHub, rueda_izq: Motor, rueda_der: Motor):
        self.drivebase = drivebase
        self.settings_predeterminados = self.drivebase.settings()
        self.hub = hub
        self.rueda_izq = rueda_izq
        self.rueda_der = rueda_der

    def recto(self, distancia: int, *, velocidad: int = None,
              stop: Stop = Stop.HOLD, wait_ms: int = 50,
              espera: bool = True):
        # Si se especifica una velocidad, se usa esta velocidad, si no, se queda como está
        if velocidad is not None:
            self.drivebase.settings(velocidad)

        distancia_actual = self.drivebase.distance()

        self.drivebase.straight(distancia - distancia_actual, then=stop, wait=espera)

        if wait_ms > 0 or stop != Stop.NONE or espera:
            wait(wait_ms)
        self.drivebase.settings(*self.settings_predeterminados)

    def recto_angulo(self, grados: int, *, velocidad: int = 700,
              stop: Stop = Stop.HOLD, wait_ms: int = 100,
              espera: bool = True):
        grados_iniciales = rueda_izq.angle()
        rueda_izq.run_angle(velocidad, grados - grados_iniciales, then=stop, wait=False)
        rueda_der.run_angle(velocidad, grados - grados_iniciales, then=stop, wait=False)
        if espera:
            while not (rueda_izq.done() and rueda_der.done()):
                wait(1)
        if wait_ms > 0 or stop != Stop.NONE or espera:
            wait(wait_ms)

    def drive(self, velocidad: int = None, *, giro: int = 0, sentido: int = 1):
        if velocidad == None:
            velocidad = self.settings_predeterminados[0]
        self.drivebase.drive(velocidad * sentido, giro)

    def giro(self, angulo_objetivo: int, *, velocidad: int = None,
                 stop: Stop = Stop.HOLD, wait_ms: int = 100,
                 espera: bool = True):
        angulo_inicial = self.hub.imu.heading()

        # Si se especifica una velocidad, se usa esta velocidad, si no, se queda como está
        if velocidad:
            settings = list(self.settings_predeterminados)
            settings[2] = velocidad
            # El asterisco antes de una lista la "desempaqueta", por ejemplo si
            # settings = [217, 816, 189, 851],
            # *settings es 217, 816, 189, 851
            # drivebase.settings() usa la lista desempaquetada como parámetros
            self.drivebase.settings(*settings)

        self.drivebase.turn(angulo_objetivo - angulo_inicial, then=stop, wait=espera)
        if wait_ms > 0 or stop != Stop.NONE or espera:
            wait(wait_ms)

        self.drivebase.settings(*self.settings_predeterminados)

    def brake(self):
        self.drivebase.brake()
        self.rueda_izq.brake()
        self.rueda_der.brake()

    def coast(self):
        self.drivebase.stop()
        self.rueda_izq.stop()
        self.rueda_der.stop()

    def reset_giro(self):
        self.hub.imu.reset_heading(0)

    def reset_motores(self):
        self.rueda_izq.reset_angle(0)
        self.rueda_der.reset_angle(0)
        self.drivebase.reset()
        # (estoy en versión 3.3.0 de Pybricks)
        # es importante tener el reset de la distancia debajo del reset de los
        # motores porque el cambio de ángulo de los motores modifica la distancia
        # https://github.com/pybricks/support/issues/1449

    def distance(self):
        return self.drivebase.distance()

    def settings(self):
        return self.drivebase.settings()


# Limpiamos el terminal
print("\x1b[H\x1b[2J", end="")

hub = PrimeHub(top_side=Axis.Z, front_side=Axis.Y)
hub.display.orientation(Side.TOP)

rueda_izq = Motor(Port.B, Direction.COUNTERCLOCKWISE, reset_angle=True)
rueda_der = Motor(Port.A, Direction.CLOCKWISE, reset_angle=True)

drivebase = DriveBase(rueda_izq, rueda_der, 62.4, 110)
drivebase.use_gyro(True)
drivebase.settings(217*1.5, 816*1, 189*1, 851*0.75)
# straight_speed, straight_acceleration, turn_speed, turn_acceleration

utillaje_izq = Motor(Port.E, Direction.COUNTERCLOCKWISE, reset_angle=True)
utillaje_der = Motor(Port.F, Direction.COUNTERCLOCKWISE, reset_angle=True)

sensor_color = ColorSensor(Port.D)

robot = MiDriveBase(drivebase, hub, rueda_izq, rueda_der)
hub.system.set_stop_button(Button.BLUETOOTH)

# La tensión nos dice (más o menos) el nivel de la batería
print(f"tension: {hub.battery.voltage()} mV\n") # 100%: 8324 mV

# Para tener la referencia de parámetros de velocidad y eso a mano
print("settings:")
print("    straight_speed:", robot.settings()[0])
print("    straight_acceleration:", robot.settings()[1])
print("    turn_speed:", robot.settings()[2])
print("    turn_acceleration:", robot.settings()[3], "\n")

# No sigue hasta que no está bien calibrado
hub.light.animate([Color.RED, Color.BLACK], 200)
while not hub.imu.ready():
    hub.speaker.beep(100)
    wait(100)
hub.speaker.volume(50)
hub.speaker.beep(440)
hub.speaker.beep(590)
hub.light.on(Color.GREEN)

def espera_boton():
    global hub
    while Button.CENTER not in hub.buttons.pressed():
        wait(1)

def salida_1(hub: PrimeHub, rueda_izq: Motor, rueda_der: Motor,
             drivebase: DriveBase, robot: MiDriveBase,
             utillaje_izq: Motor, utillaje_der: Motor):
    #robot.recto_angulo(-280)
    #wait(200)
    #robot.recto_angulo(10)

    utillaje_izq.run_until_stalled(-200, then=Stop.HOLD, duty_limit=50)
    utillaje_izq.run_angle(200, 20)
    utillaje_izq.reset_angle()
    # para llevar el brazo hasta el tope y que siempre empiece desde el mismo sitio

    robot.recto(-150)
    wait(200)
    robot.recto(16)
    # carrito liberado

    robot.giro(45)
    robot.recto(400, stop=Stop.NONE)
    robot.recto_angulo(12400, velocidad=130, stop=Stop.COAST_SMART, espera=False)
    utillaje_der.run_angle(1000, -1800, wait=False)

    stalled = False
    while not utillaje_der.done() and not stalled:
        for i in range(1000):
            wait(1)
            if i == 999:
                stalled = True
                break
            if not utillaje_der.stalled():
                break

    utillaje_der.stop()
    robot.coast()
    robot.reset_giro()
    robot.reset_motores()
    # pollo hecho

    robot.recto(-40, velocidad=80, stop=Stop.NONE)
    robot.recto(-165)
    robot.giro(-98)
    #robot.recto_angulo(-1122, stop=Stop.NONE)
    #robot.recto(-586, stop=Stop.NONE)
    robot.recto(-582, stop=Stop.NONE)
    robot.coast()
    wait(300)
    utillaje_der.run_angle(300, 300, wait=False)
    wait(500)
    robot.recto(-535)
    # altavoces y luces hechas

    robot.giro(-20)
    robot.recto_angulo(-500)
    robot.giro(-42)
    utillaje_der.run_angle(200, -250, wait=False)
    robot.recto(130, stop=Stop.NONE)
    robot.recto(180, velocidad=150, stop=Stop.NONE)
    robot.drive(60)

    linea_detectada = False
    while not linea_detectada:
        for i in range(30):
            wait(1)
            if i == 29:
                linea_detectada = True
                break
            if sensor_color.color() == Color.WHITE:
                break
    linea_detectada = False
    while not linea_detectada:
        for i in range(30):
            wait(1)
            if i == 29:
                linea_detectada = True
                break
            if sensor_color.color() != Color.WHITE:
                break
    linea_detectada = False
    while not linea_detectada:
        for i in range(30):
            wait(1)
            if i == 29:
                linea_detectada = True
                break
            if sensor_color.color() == Color.WHITE:
                break
    del linea_detectada
    
    """
    while True:
        if sensor_color.color() != Color.WHITE:
            wait(10)
            if sensor_color.color() != Color.WHITE:
                wait(10)
                if sensor_color.color() != Color.WHITE:
                    break
    while True:
        if sensor_color.color() == Color.WHITE:
            wait(10)
            if sensor_color.color() == Color.WHITE:
                wait(10)
                if sensor_color.color() == Color.WHITE:
                    break
    while True:
        if sensor_color.color() != Color.WHITE:
            wait(10)
            if sensor_color.color() != Color.WHITE:
                wait(10)
                if sensor_color.color() != Color.WHITE:
                    break
    """
    robot.brake()
    robot.reset_motores()
    #wait(100)
    #robot.recto(31)
    # referenciados con la línea
    wait(100)
    robot.giro(-132)
    utillaje_izq.run_angle(200, 140)
    # brazo bajado

    robot.recto(-70, stop=Stop.NONE)
    robot.recto(-100, velocidad=250, stop=Stop.NONE)
    robot.recto_angulo(grados=-20000, velocidad=200, espera=False)
    wait(500)
    utillaje_izq.run_angle(200, -140)
    robot.reset_motores()
    robot.reset_giro()
    # paredes lilas hechas
    
    #robot.recto(170)
    robot.recto(155)
    #espera_boton()
    robot.giro(217)
    #espera_boton()
    robot.recto(-335)
    #espera_boton()
    #rueda_der.run_angle(400, -210)
    rueda_der.run(-200)
    while hub.imu.heading() <= 250:
        wait(1)
    robot.brake()
    #espera_boton()
    wait(100)
    robot.recto(-1000, velocidad=600, stop=Stop.COAST)
    
    print("rueda_izq:", rueda_izq.angle())
    print("rueda_der:", rueda_der.angle())
    print("distance:", robot.distance())
    print("heading:", hub.imu.heading())
    print()
    
def salida_2(hub: PrimeHub, rueda_izq: Motor, rueda_der: Motor,
             drivebase: DriveBase, robot: MiDriveBase,
             utillaje_izq: Motor, utillaje_der: Motor):
    robot.reset_giro()
    robot.reset_motores()

    utillaje_izq.run_until_stalled(200, duty_limit=50)
    utillaje_izq.run_angle(200, -20, wait=False)
    utillaje_izq.reset_angle(0)

    utillaje_der.run_angle(900, 1100, then=Stop.NONE)
    while (not utillaje_der.stalled()) and (not utillaje_der.done()):
        wait(1)
    if utillaje_der.stalled():
        hub.speaker.beep(440)
    utillaje_der.brake()

    robot.recto(-50, velocidad=200, stop=Stop.NONE)
    robot.recto_angulo(-10000, velocidad=300, espera=False)
    wait(400)
    robot.brake()
    robot.reset_giro()
    robot.reset_motores()

    rueda_izq.run(100)
    while hub.imu.heading() <=  37:
        wait(1)
    robot.brake()
    wait(100)
    robot.recto(280, stop=Stop.NONE)
    robot.recto(430, velocidad=200, stop=Stop.NONE)
    robot.recto(510, velocidad=70, stop=Stop.NONE)
    robot.coast()
    wait(300)
    robot.brake()
    robot.reset_giro()
    robot.reset_motores()
    # mezclador hecho

    robot.recto(-220)
    robot.giro(-40)
    robot.recto(179)
    rueda_der.run(200)
    while hub.imu.heading() >= -84:
        wait(1)
    robot.brake()
    wait(100)
    robot.recto(285)
    utillaje_izq.run_angle(70, -100)
    # experto recogido y teatro hecho

    # DEPENDIENDO DEL OTRO EQUIPO:
    teatro_numero = int.from_bytes(hub.system.storage(0, read=1), "big")
    teatro_dict = {
        0: Color.BLUE,
        1: Color.MAGENTA,
        2: Color.ORANGE
    }
    if teatro_dict[teatro_numero] in [Color.BLUE, Color.ORANGE]:
        robot.recto(220)
        wait(200)
        robot.recto(285)

    rueda_izq.run(-200)
    while hub.imu.heading() >= -129:
        wait(1)
    robot.recto(50)
    utillaje_der.run_angle(900, -950)
    robot.recto(-200, stop=Stop.NONE)
    #      _
    #     / \
    #    / ! \
    #   /_____\
    #
    robot.recto(-390, velocidad=200, stop=Stop.NONE)
    #utillaje_der.run_angle(900, 780, wait=False)
    hub.speaker.beep(duration=-1)
    robot.drive(-60)
    linea_detectada = False
    while not linea_detectada:
        for i in range(30):
            wait(1)
            if i == 29:
                linea_detectada = True
                break
            if sensor_color.color() == Color.WHITE:
                break
    linea_detectada = False
    while not linea_detectada:
        for i in range(30):
            wait(1)
            if i == 29:
                linea_detectada = True
                break
            if sensor_color.color() != Color.WHITE:
                break
    while not linea_detectada:
        for i in range(30):
            wait(1)
            if i == 29:
                linea_detectada = True
                break
            if sensor_color.color() == Color.WHITE:
                break
    robot.brake()
    hub.speaker.beep()
    wait(100)
    robot.reset_motores()
    # referenciados con la línea
    robot.recto(-45)

    rueda_izq.run(350)
    while hub.imu.heading() <= -45:
        wait(1)
    robot.brake()
    wait(100)
    robot.recto(50, stop=Stop.NONE)
    robot.recto_angulo(-10000, velocidad=400, espera=False)
    wait(600)
    robot.brake()
    robot.reset_giro()
    robot.reset_motores()
    
    utillaje_der.run_until_stalled(1000, then=Stop.HOLD)
    utillaje_der.run_angle(1000, -200)

    robot.recto(25)
    rueda_izq.run(500)
    while hub.imu.heading() <= 95:
        wait(1)
    robot.brake()
    wait(100)
    robot.recto(650, stop=Stop.NONE)
    drivebase.curve(300, 46, then=Stop.NONE)
    robot.recto_angulo(10000, velocidad=800, espera=False)
    wait(1000)
    robot.coast()
    
    print("rueda_izq:", rueda_izq.angle())
    print("rueda_der:", rueda_der.angle())
    print("distance:", robot.distance())
    print("heading:", hub.imu.heading())
    print()

def salida_3(hub: PrimeHub, rueda_izq: Motor, rueda_der: Motor,
             drivebase: DriveBase, robot: MiDriveBase,
             utillaje_izq: Motor, utillaje_der: Motor):
    robot.reset_giro()
    robot.reset_motores()

    utillaje_der.run_until_stalled(200, then=Stop.HOLD, duty_limit=80)

    robot.recto(-270, stop=Stop.NONE)
    robot.recto(-450, velocidad=40, stop=Stop.NONE)
    robot.recto(-600, stop=Stop.NONE)
    robot.recto(-650, velocidad=200, stop=Stop.NONE)
    robot.recto(-720, velocidad=140, stop=Stop.NONE)
    robot.coast()
    robot.recto(-210)
    robot.giro(-85)
    robot.recto(-250, stop=Stop.NONE)
    robot.recto_angulo(-10000, velocidad=600, espera=False)
    wait(600)
    robot.brake()
    robot.reset_giro()
    robot.reset_motores()
    robot.recto(500)
    rueda_der.run(200)
    while hub.imu.heading() >= -30:
        wait(1)
    robot.brake()
    wait(100)
    robot.recto(670)
    rueda_izq.run(200)
    while hub.imu.heading() <= 44:
        wait(1)
    robot.brake()
    wait(100)
    robot.recto(900, velocidad=120)
    utillaje_izq.run_angle(1000, -350)
    robot.recto(730)
    robot.giro(90)
    robot.recto(305)
    robot.giro(178)
    robot.recto(250, stop=Stop.NONE)
    robot.recto_angulo(-10000, velocidad=400, espera=False)
    wait(750)
    utillaje_der.run_angle(200, -220)
    robot.brake()
    robot.reset_giro()
    robot.reset_motores()
    robot.recto(164, velocidad=150)
    robot.giro(-90, velocidad=40)
    robot.recto(482)
    robot.giro(-181)
    robot.recto(545, stop=Stop.NONE)
    robot.coast()
    
    print("rueda_izq:", rueda_izq.angle())
    print("rueda_der:", rueda_der.angle())
    print("distance:", robot.distance())
    print("heading:", hub.imu.heading())
    print()

"""for i in range(5):
    for j in range(5):
        hub.display.pixel(i, j, 100)"""

utillaje_izq.reset_angle(0)
utillaje_der.reset_angle(0)
robot.reset_giro()
robot.reset_motores()


def display_salida(salida):
    if salida == 1:
        matriz = [
            [  0,   0, 100,   0,   0],
            [  0,   0, 100,   0,   0],
            [  0,   0, 100,   0,   0],
            [  0,   0, 100,   0,   0],
            [  0,   0, 100,   0,   0]
        ]
        hub.light.on(Color.GREEN)
    elif salida == 2:
        matriz = [
            [  0, 100, 100,   0,   0],
            [  0,   0,   0, 100,   0],
            [  0,   0, 100,   0,   0],
            [  0, 100,   0,   0,   0],
            [  0, 100, 100, 100,   0]
        ]
        hub.light.on(Color.RED)
    elif salida == 3:
        matriz = [
            [  0, 100, 100, 100,   0],
            [  0,   0,   0, 100,   0],
            [  0,   0, 100, 100,   0],
            [  0,   0,   0, 100,   0],
            [  0, 100, 100, 100,   0]
        ]
        hub.light.on(Color.BLUE)
    else:
        matriz = [
            [100, 100, 100, 100, 100],
            [100, 100, 100, 100, 100],
            [100, 100, 100, 100, 100],
            [100, 100, 100, 100, 100],
            [100, 100, 100, 100, 100]
        ]
        hub.speaker.beep(200)
        hub.light.on(Color.BLACK)
        print("??2")
    for i in range(5):
        for j in range(5):
            hub.display.pixel(i, j, matriz[i][j])

hub.display.off()

# mejor pasarle esta lista desempaquetada a las salidas en vez de todo eso cada vez
robot_objetos = [hub, rueda_izq, rueda_der, drivebase, robot, utillaje_izq, utillaje_der]

#salida_1(*robot_objetos)
#salida_2(*robot_objetos)
#salida_3(*robot_objetos)

"""while True:
    hub.speaker.beep(400)
    wait(2000)"""

def elige_teatro():
    hub.display.off()
    hub.speaker.beep(500)
    wait(50)
    hub.speaker.beep(500)

    # Leer el número de teatro actual desde el almacenamiento del sistema
    teatro_numero = int.from_bytes(hub.system.storage(0, read=1), "big")

    # Diccionario de configuración de teatros
    teatro_dict = {
        0: {"color": Color.BLUE, "pixel": (4, 0), "char": "A"},
        1: {"color": Color.MAGENTA, "pixel": (4, 2), "char": "R"},
        2: {"color": Color.ORANGE, "pixel": (4, 4), "char": "N"},
    }

    # Función para actualizar la luz, el pixel y el sonido basado en teatro_numero
    def actualizar_display_y_luz():
        hub.display.off()
        #hub.display.pixel(*teatro_dict[teatro_numero]["pixel"], 100)
        hub.display.char(teatro_dict[teatro_numero]["char"])
        hub.light.on(teatro_dict[teatro_numero]["color"])
        hub.speaker.beep(400)

    # Actualizar display y luz iniciales
    actualizar_display_y_luz()

    while True:
        while hub.buttons.pressed():
            wait(1)
        while not hub.buttons.pressed():
            wait(1)
        pressed_buttons = hub.buttons.pressed()

        # Lógica para el botón CENTRO
        if Button.CENTER in pressed_buttons:
            hub.speaker.beep(500)
            wait(300)
            return  # Salir de la función

        # Lógica para el botón IZQUIERDO (ciclo hacia atrás)
        elif Button.LEFT in pressed_buttons:
            teatro_numero = (teatro_numero - 1) % 3  # Ciclo hacia atrás
            hub.system.storage(0, write=bytes([teatro_numero]))
            actualizar_display_y_luz()

        # Lógica para el botón DERECHO (ciclo hacia adelante)
        elif Button.RIGHT in pressed_buttons:
            teatro_numero = (teatro_numero + 1) % 3  # Ciclo hacia adelante
            hub.system.storage(0, write=bytes([teatro_numero]))
            actualizar_display_y_luz()


salida = 1
stopwatch = StopWatch()
stopwatch.pause()
stopwatch.reset()
stopwatch_threshold = 2000
while True:
    robot.reset_giro()
    robot.reset_motores()

    if not hub.imu.ready():
        hub.light.on(Color.RED)
        hub.display.off()
        while not hub.imu.ready():
            hub.speaker.beep(100)
            wait(100)
        hub.light.on(Color.GREEN)
        hub.speaker.beep(500)
        wait(400)

    display_salida(salida)
    while not hub.buttons.pressed():
        wait(1)
    
    stopwatch.reset()
    stopwatch.resume()
    pressed_buttons = list(hub.buttons.pressed())
    pulsacion_larga = False  # Indicador de si se detectó una pulsación larga.

    while hub.buttons.pressed():
        if stopwatch.time() >= stopwatch_threshold:
            if (Button.LEFT in pressed_buttons or Button.RIGHT in pressed_buttons) and not pulsacion_larga:
                elige_teatro()
                hub.display.off()
                hub.light.off()
                pulsacion_larga = True  # Evita entrar de nuevo en esta condición.
                break  # Finaliza el bucle ya que se ha entrado al modo teatro.
        wait(1)
    
    stopwatch.pause()
    tiempo_pulsado = stopwatch.time()

    """
    # Omitir la lógica de botones si se activó el modo teatro por pulsación larga.
    if pulsacion_larga:
        wait(100)  # Pequeña pausa antes de la siguiente iteración.
        continue
    """

    if tiempo_pulsado < stopwatch_threshold:
        if Button.CENTER in pressed_buttons:
            if salida == 1:
                robot.reset_giro()
                robot.reset_motores()
                salida_1(*robot_objetos)
                print("salida 1\n")
                salida = 2
                wait(100)
            elif salida == 2:
                robot.reset_giro()
                robot.reset_motores()
                salida_2(*robot_objetos)
                print("salida 2\n")
                salida = 3
                wait(100)
            elif salida == 3:
                robot.reset_giro()
                robot.reset_motores()
                salida_3(*robot_objetos)
                print("salida 3\n")
                salida = 1
                wait(100)
        
        elif Button.LEFT in pressed_buttons:
            salida = 3 if salida == 1 else salida - 1
            display_salida(salida)
            hub.speaker.beep(440)
        
        elif Button.RIGHT in pressed_buttons:
            salida = 1 if salida == 3 else salida + 1
            display_salida(salida)
            hub.speaker.beep(440)

    wait(1)  # Pequeña espera antes de la próxima iteración.


while True:
    if Button.CENTER in hub.buttons.pressed():
        hub.speaker.beep(440)
    wait(100)