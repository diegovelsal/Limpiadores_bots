from mesa.model import Model
from mesa.agent import Agent
from mesa.space import MultiGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector

import numpy as np

# Clase Celda que guarda la información de si está sucia o no
class Celda(Agent):
    def __init__(self, unique_id, model, suciedad: bool = False):
        super().__init__(unique_id, model)
        self.sucia = suciedad

# Clase Mueble
class Mueble(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

# Clase Estación de carga para el robot
class Estacion(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.ocupada = False

# Clase Robot 
class RobotLimpieza(Agent):
    # Inicialización del robot
    def __init__(self, unique_id, model, posiciones_estaciones):
        super().__init__(unique_id, model)
        self.sig_pos = None
        self.movimientos = 0
        self.buscando_estacion = None
        self.posiciones_estaciones = posiciones_estaciones
        self.carga = 30

    # Método para obtener las celdas disponibles vecinas al robot en las que se puede mover
    def celdas_disponibles_cercanas(self, radius=1):
        celdas_disponibles = self.model.grid.get_neighbors(
            self.pos, moore=True, include_center=False, radius=radius)

        # De los vecinos, elimina los muebles, robots y estaciones de carga para quedarse con las celdas vacías
        for vecino in celdas_disponibles:
            if isinstance(vecino, (Mueble, RobotLimpieza, Estacion)):
                celdas_disponibles.remove(vecino)

        celdas_disponibles = [vecino for vecino in celdas_disponibles if not isinstance(vecino, (Mueble, RobotLimpieza, Estacion))]

        return celdas_disponibles

    # Método para calcular la distancia a una posición objetivo
    @staticmethod
    def calcular_distancias(lista_posiciones, celda_comparacion):
        distancias = []
        for celda in lista_posiciones:
            distancia = abs(celda_comparacion.pos[0] - celda.pos[0]) + abs(celda_comparacion.pos[1] - celda.pos[1])
            distancias.append((distancia, celda))

        # Buscar la celda sucia mas cercana
        distancias.sort(key=lambda tup: tup[0])
        return distancias[0][1]

    # Método para limpiar una celda sucia al azar de las que se encuentran vecinas al robot
    def limpiar_una_celda(self, lista_de_celdas_sucias):
        # Obtiene la celda sucia más cercana
        celda_sucia_cercana = self.calcular_distancias(lista_de_celdas_sucias, self)

        # Se mueve a la estación más cercana, debe moverse de una celda a la vez  
        celda_disponible = self.celdas_disponibles_cercanas(1)

        # Si la celda sucia más cercana está en las celdas disponibles, se mueve a ella
        if celda_sucia_cercana in celda_disponible:
            self.sig_pos = celda_sucia_cercana.pos
            celda_sucia_cercana.sucia = False
        else:
            # Si no, se mueve a la celda más cercana a la celda sucia más cercana
            self.sig_pos = self.calcular_distancias(celda_disponible, celda_sucia_cercana).pos
            
    # Método para seleccionar una nueva posición al azar en caso de que no haya celdas sucias
    def seleccionar_nueva_pos(self, lista_de_vecinos):
        self.sig_pos = self.random.choice(lista_de_vecinos).pos

    # Regresa una lista de celdas sucias vecinas al robot
    @staticmethod
    def buscar_celdas_sucia(lista_de_vecinos):
        return [vecino for vecino in lista_de_vecinos
                        if isinstance(vecino, Celda) and vecino.sucia]

    def buscar_estacion(self):
        # Calcula la distancia a cada estación
        distancias = []
        for pos in self.posiciones_estaciones:
            estacion = self.model.grid.get_cell_list_contents(pos)
            if not estacion[0].ocupada:
                distancias.append((abs(self.pos[0] - pos[0]) + abs(self.pos[1] - pos[1]), pos))

        if len(distancias) == 0:
            self.sig_pos = self.pos
            return
        # Ordena las distancias de menor a mayor
        distancias.sort(key=lambda tup: tup[0])

        # Selecciona la estación más cercana
        self.buscando_estacion = distancias[0][1]
        estacion = self.model.grid.get_cell_list_contents(self.buscando_estacion)
        estacion[0].ocupada = True

    def moverse_a_estacion(self):
        if self.pos == self.buscando_estacion:
            self.carga += 25
            if (self.carga > 100):
                self.carga = 100
                estacion = self.model.grid.get_cell_list_contents(self.buscando_estacion)
                estacion[0].ocupada = False
                self.buscando_estacion = None
            self.sig_pos = self.pos
        else:
            # Se mueve a la estación más cercana, debe moverse de una celda a la vez
            celda_disponible = self.model.grid.get_neighbors(
                self.pos, moore=True, include_center=False)

            # De los vecinos, elimina los muebles, robots y estaciones de carga para quedarse con las celdas vacías
            for vecino in celda_disponible:
                if isinstance(vecino, (Mueble, RobotLimpieza)):
                    celda_disponible.remove(vecino)

            celda_disponible = [vecino for vecino in celda_disponible if not isinstance(vecino, (Mueble, RobotLimpieza))]

            # Obtiene las distancias de las celdas disponibles a la estación
            distancias = []
            for cell in celda_disponible:
                distancias.append((abs(self.buscando_estacion[0] - cell.pos[0]) + abs(self.buscando_estacion[1] - cell.pos[1]), cell.pos))

            # Ordena las distancias de menor a mayor
            distancias.sort(key=lambda tup: tup[0])

            # Selecciona la celda más cercana
            self.sig_pos = distancias[0][1]
            
    # Step del sistema
    def step(self):
        # Obtiene los vecinos mediante lo de mesa
        vecinos = self.celdas_disponibles_cercanas(3)

        # Obtiene las celdas sucias de los vecinos que quedaron
        celdas_sucias = self.buscar_celdas_sucia(vecinos)

        # Disminuye la carga del robot
        if self.carga > 20 and not self.buscando_estacion:
            # Robot hace una acción dependiendo de si hay o no celdas sucias en los vecinos
            if len(celdas_sucias) == 0:
                self.seleccionar_nueva_pos(self.celdas_disponibles_cercanas(1))
            else:
                self.limpiar_una_celda(celdas_sucias)
        else:
            if self.buscando_estacion is None:
                self.buscar_estacion()
                if self.buscando_estacion is not None:
                    self.moverse_a_estacion()
            else:
                self.moverse_a_estacion()

    # Método con el que se confirma el movimiento del robot
    def advance(self):
        # Si hay un cambio de posición, se aumenta el contador de movimientos
        if self.pos != self.sig_pos:
            self.movimientos += 1
            self.carga -= 1

        self.model.grid.move_agent(self, self.sig_pos)

# Modelo del sistema
class Habitacion(Model):
    # Inicialización del modelo con valores predeterminados
    def __init__(self, M: int, N: int,
                 num_agentes: int = 5,
                 porc_celdas_sucias: float = 0.6,
                 porc_muebles: float = 0.1,
                 modo_pos_inicial: str = 'Fija',
                 ):

        self.num_agentes = num_agentes # Número de robots
        self.porc_celdas_sucias = porc_celdas_sucias # Porcentaje de celdas sucias
        self.porc_muebles = porc_muebles # Porcentaje de muebles

        # Creación de la grilla y el scheduler
        self.grid = MultiGrid(M, N, False)
        self.schedule = SimultaneousActivation(self)

        # Inicializa las posiciones disponibles con todas las coordenadas de la grilla
        posiciones_disponibles = [pos for _, pos in self.grid.coord_iter()]

        # Posicionamiento de estaciones de carga de la misma forma que los muebles
        num_estaciones = 4 # Número de estaciones de carga
        posiciones_estaciones = [] # Lista de posiciones de las estaciones de carga
        n = len(posiciones_disponibles) # Número de posiciones por estación
        posiciones_estaciones.append(tuple([self.random.randint(0, int(M/2)), self.random.randint(0, int(N/2))]))
        posiciones_estaciones.append(tuple([self.random.randint(int(M/2), M-1), self.random.randint(0, int(N/2))]))
        posiciones_estaciones.append(tuple([self.random.randint(0, int(M/2)), self.random.randint(int(N/2), N-1)]))
        posiciones_estaciones.append(tuple([self.random.randint(int(M/2), M-1), self.random.randint(int(N/2), N-1)]))

        # Agrega las estaciones a la grilla y los elimina de las posiciones disponibles
        for id, pos in enumerate(posiciones_estaciones):
            estacion = Estacion(int(f"{num_agentes}0{id}") + 1, self)
            self.grid.place_agent(estacion, pos)
            posiciones_disponibles.remove(pos)

        # Posicionamiento de muebles
        num_muebles = int(M * N * porc_muebles) # Calcula el número de muebles
        posiciones_muebles = self.random.sample(posiciones_disponibles, k=num_muebles) # Asigna posiciones aleatorias a los muebles

        # Agrega los muebles a la grilla y los elimina de las posiciones disponibles
        for id, pos in enumerate(posiciones_muebles):
            mueble = Mueble(int(f"{num_agentes}0{id}") + 1, self)
            self.grid.place_agent(mueble, pos)
            posiciones_disponibles.remove(pos)

        # Posicionamiento de celdas sucias de la misma forma que los muebles
        self.num_celdas_sucias = int(M * N * porc_celdas_sucias)
        posiciones_celdas_sucias = self.random.sample(
            posiciones_disponibles, k=self.num_celdas_sucias)

        for id, pos in enumerate(posiciones_disponibles):
            suciedad = pos in posiciones_celdas_sucias
            celda = Celda(int(f"{num_agentes}{id}") + 1, self, suciedad)
            self.grid.place_agent(celda, pos)

        # Posicionamiento de agentes robot
        if modo_pos_inicial == 'Aleatoria':
            pos_inicial_robots = self.random.sample(posiciones_disponibles, k=num_agentes)
        else:  # 'Fija'
            pos_inicial_robots = [(1, 1)] * num_agentes

        for id in range(num_agentes):
            robot = RobotLimpieza(id, self, posiciones_estaciones)
            self.grid.place_agent(robot, pos_inicial_robots[id])
            self.schedule.add(robot)

        self.datacollector = DataCollector(
            model_reporters={"Grid": get_grid, "Cargas": get_cargas,
                             "CeldasSucias": get_sucias},
        )

    def step(self):
        self.datacollector.collect(self)

        self.schedule.step()

    def todoLimpio(self):
        for (content, x, y) in self.grid.coord_iter():
            for obj in content:
                if isinstance(obj, Celda) and obj.sucia:
                    return False
        return True


def get_grid(model: Model) -> np.ndarray:
    """
    Método para la obtención de la grid y representarla en un notebook
    :param model: Modelo (entorno)
    :return: grid
    """
    grid = np.zeros((model.grid.width, model.grid.height))
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        x, y = pos
        for obj in cell_content:
            if isinstance(obj, RobotLimpieza):
                grid[x][y] = 2
            elif isinstance(obj, Celda):
                grid[x][y] = int(obj.sucia)
    return grid


def get_cargas(model: Model):
    return [(agent.unique_id, agent.carga) for agent in model.schedule.agents]


def get_sucias(model: Model) -> int:
    """
    Método para determinar el número total de celdas sucias
    :param model: Modelo Mesa
    :return: número de celdas sucias
    """
    sum_sucias = 0
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        for obj in cell_content:
            if isinstance(obj, Celda) and obj.sucia:
                sum_sucias += 1
    return sum_sucias / model.num_celdas_sucias


def get_movimientos(agent: Agent) -> dict:
    if isinstance(agent, RobotLimpieza):
        return {agent.unique_id: agent.movimientos}
    # else:
    #    return 0
