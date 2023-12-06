#!/usr/bin/env python

from matplotlib.animation import FuncAnimation

import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np
import threading

# Classe para processar e visualizar dados de LaserScan
class LaserScanProcessor:
    def __init__(self):
        # Inicializando o nó ROS
        #rospy.init_node('laser_scan_processor', anonymous=True)
        
         # Lock para sincronização
        self.lock = threading.Lock()

        # Subscribing to the LaserScan topic
        rospy.Subscriber("eboat/laser/scan", LaserScan, self.callback)

        # Dados do LaserScan
        self.laser_data = None

        # Configurando a plotagem
        
        self.fig, self.ax = plt.subplots(
            figsize=(8, 8)
            ) # figsize=(8, 8)
              

    def callback(self, data):
        # Atualizando os dados do LaserScan
        with self.lock:
            self.laser_data = data

    def process_data(self):
        with self.lock:
            if self.laser_data:
                ranges = np.array(self.laser_data.ranges)
                num_groups = 5  # Dividir em 5 grupos
                group_size = 3  # Cada grupo tem 3 feixes

                # Verificar se o número total de feixes é divisível em 5 grupos de 3
                if len(ranges) % group_size != 0:
                    print("O número de feixes não é compatível com a divisão em grupos de 3.")
                    return None

                groups = [ranges[i * group_size : (i + 1) * group_size] for i in range(num_groups)]

                for i in range(len(groups)):
                    # Substituir valores infinitos por NaN para não incluí-los na média
                    groups[i] = np.where(np.isinf(groups[i]), np.nan, groups[i])
                    # Calcular a média, ignorando NaNs (que eram valores infinitos)
                    groups[i] = np.nanmean(groups[i])

                return groups

        return None

        
    def update_plot(self, frame):
        groups = self.process_data()
        if groups:
            self.ax.clear()

            # Gerar cores dinamicamente para cada grupo
            group_colors = ['red', 'green', 'blue', 'yellow', 'orange', 'purple', 'pink', 'cyan', 'magenta', 'brown', 'gray', 'olive', 'lime', 'teal', 'navy']

            # Definir a largura e a altura dos retângulos dos grupos
            rect_width = 0.1
            rect_height = 0.2

            # Gerar posições para cada grupo
            positions = [(0.1 + i * 0.15, 0.5) for i in range(len(groups))]

            # Plotar retângulos para cada grupo
            for group_index, group_value in enumerate(groups):
                pos = positions[group_index]
                color = group_colors[group_index]
                alpha = 0.7 if group_value > 0 else 0.1
                # Desenhar o retângulo para o grupo
                rect = plt.Rectangle(pos, rect_width, rect_height, color=color, alpha=alpha)
                self.ax.add_patch(rect)
                self.ax.text(pos[0] + rect_width / 2, pos[1] + rect_height / 2, f'Grupo {group_index + 1}', horizontalalignment='center', verticalalignment='center')

            self.ax.set_xlim(0, 1)
            self.ax.set_ylim(0, 1)

    def run(self):
        # Executando a animação
        ani = FuncAnimation(self.fig, self.update_plot, interval=100)
        plt.show()

# Criando e executando a instância da classe
if __name__ == "__main__":
    processor = LaserScanProcessor()
    processor.run()
