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
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.sig_pos = None
        self.movimientos = 0
        self.carga = 100

    # Método para limpiar una celda sucia al azar de las que se encuentran vecinas al robot
    def limpiar_una_celda(self, lista_de_celdas_sucias):
        celda_a_limpiar = self.random.choice(lista_de_celdas_sucias)
        celda_a_limpiar.sucia = False
        self.sig_pos = celda_a_limpiar.pos

    # Método para seleccionar una nueva posición al azar en caso de que no haya celdas sucias
    def seleccionar_nueva_pos(self, lista_de_vecinos):
        self.sig_pos = self.random.choice(lista_de_vecinos).pos

    # Regresa una lista de celdas sucias vecinas al robot
    @staticmethod
    def buscar_celdas_sucia(lista_de_vecinos):
        return [vecino for vecino in lista_de_vecinos
                        if isinstance(vecino, Celda) and vecino.sucia]

    # Step del sistema
    def step(self):
        # Obtiene los vecinos mediante lo de mesa
        vecinos = self.model.grid.get_neighbors(
            self.pos, moore=True, include_center=False)

        # De los vecinos, elimina los muebles, robots y estaciones de carga para quedarse con las celdas vacías
        for vecino in vecinos:
            if isinstance(vecino, (Mueble, RobotLimpieza, Estacion)):
                vecinos.remove(vecino)

        # Obtiene las celdas sucias de los vecinos que quedaron
        celdas_sucias = self.buscar_celdas_sucia(vecinos)

        # Robot hace una acción dependiendo de si hay o no celdas sucias en los vecinos
        if len(celdas_sucias) == 0:
            self.seleccionar_nueva_pos(vecinos)
        else:
            self.limpiar_una_celda(celdas_sucias)

    # Método con el que se confirma el movimiento del robot
    def advance(self):
        # Si hay un cambio de posición, se aumenta el contador de movimientos
        if self.pos != self.sig_pos:
            self.movimientos += 1

        # Disminuye la carga del robot
        if self.carga > 0:
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
            print(pos)
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
            robot = RobotLimpieza(id, self)
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
