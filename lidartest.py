import sys
import time
import threading
import math
import pygame
import serial
from collections import deque
from rplidar import RPLidar

# Configuration
PORT_NAME = 'COM5'  # Ajustez selon votre port
VEHICLE_WIDTH = 42  # Largeur du véhicule en cm
OBSTACLE_THRESHOLD = 200  # Distance max pour considérer un obstacle (2m = 200cm)
VIEW_ANGLE = 90  # Angle de 180° centré (-90° à +90°)
WINDOW_SIZE = (800, 600)
MAX_DISTANCE = 500  # Distance maximale à afficher (en cm)
HISTORY_SIZE = 5  # Nombre de frames à conserver pour stabilité

# Couleurs
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)

class LidarProcessor:
    def __init__(self):
        self.lidar = None
        self.running = False
        self.scan_data = {}
        self.obstacle_points = []
        self.closest_distance = float('inf')
        self.closest_angle = 0
        self.history = deque(maxlen=HISTORY_SIZE)
        
        # Calcul de la zone de passage de la voiture
        self.angle_width = math.degrees(math.atan((VEHICLE_WIDTH/2) / 100))  # Demi-angle en degrés
        
        # Mutex pour protéger l'accès aux données
        self.data_lock = threading.Lock()
        
    def connect(self):
        try:
            self.lidar = RPLidar(PORT_NAME)
            info = self.lidar.get_info()
            print(f"LIDAR connecté: {info}")
            health = self.lidar.get_health()
            print(f"Statut LIDAR: {health}")
            return True
        except Exception as e:
            print(f"Erreur de connexion au LIDAR: {e}")
            return False
            
    def start(self):
        if self.connect():
            self.running = True
            self.scan_thread = threading.Thread(target=self._scan_thread)
            self.scan_thread.daemon = True
            self.scan_thread.start()
            return True
        return False
        
    def stop(self):
        self.running = False
        if self.scan_thread:
            self.scan_thread.join(timeout=1.0)
        if self.lidar:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
    
    def _scan_thread(self):
        try:
            self.lidar.start_motor()
            time.sleep(1)  # Attendre que le moteur atteigne sa vitesse
            
            for scan in self.lidar.iter_scans():
                if not self.running:
                    break
                    
                # Traiter les données du scan
                scan_data = {}
                for _, angle, distance in scan:
                    # Convertir en cm
                    distance = distance / 10
                    # Normaliser l'angle entre -180 et 180
                    norm_angle = angle
                    if norm_angle > 180:
                        norm_angle -= 360
                    scan_data[norm_angle] = distance
                
                # Identifier les obstacles
                obstacle_points = []
                min_distance = float('inf')
                min_angle = 0
                
                for angle, distance in scan_data.items():
                    # Vérifier si le point est dans la zone d'intérêt (devant la voiture)
                    if -VIEW_ANGLE <= angle <= VIEW_ANGLE:
                        # Vérifier si la distance est inférieure au seuil
                        if distance <= OBSTACLE_THRESHOLD:
                            # Calculer la position latérale par rapport à l'axe central
                            lateral_position = distance * math.tan(math.radians(angle))
                            
                            # Vérifier si l'obstacle est dans la trajectoire du véhicule
                            if abs(lateral_position) <= (VEHICLE_WIDTH / 2):
                                obstacle_points.append((angle, distance, True))  # Dans la trajectoire
                                
                                # Mettre à jour la distance minimale
                                if distance < min_distance:
                                    min_distance = distance
                                    min_angle = angle
                            else:
                                obstacle_points.append((angle, distance, False))  # Hors trajectoire
                
                # Ajouter à l'historique pour stabilité
                with self.data_lock:
                    self.scan_data = scan_data
                    self.obstacle_points = obstacle_points
                    if min_distance < float('inf'):
                        self.closest_distance = min_distance
                        self.closest_angle = min_angle
                    self.history.append((scan_data, obstacle_points, min_distance, min_angle))
        
        except Exception as e:
            print(f"Erreur dans le thread de scan: {e}")
        finally:
            if self.running:
                self.stop()
    
    def get_stable_data(self):
        """Retourne des données stabilisées en utilisant l'historique"""
        with self.data_lock:
            if not self.history:
                return {}, [], float('inf'), 0
                
            # Moyenne des dernières mesures pour la distance la plus proche
            closest_distances = [h[2] for h in self.history if h[2] < float('inf')]
            if closest_distances:
                avg_closest = sum(closest_distances) / len(closest_distances)
            else:
                avg_closest = float('inf')
                
            # Utiliser les données les plus récentes pour l'affichage
            return self.scan_data, self.obstacle_points, avg_closest, self.closest_angle

class LidarVisualizer:
    def __init__(self, lidar_processor):
        self.lidar_processor = lidar_processor
        self.running = False
        
        # Initialiser Pygame
        pygame.init()
        self.screen = pygame.display.set_mode(WINDOW_SIZE)
        pygame.display.set_caption("RPLidar - Détection d'obstacles")
        self.font = pygame.font.Font(None, 36)
        self.small_font = pygame.font.Font(None, 24)
        self.clock = pygame.time.Clock()
        
        # Centre de l'écran (position de la voiture)
        self.center_x = WINDOW_SIZE[0] // 2
        self.center_y = WINDOW_SIZE[1] - 50
        
        # Facteur d'échelle pour l'affichage
        self.scale = min(WINDOW_SIZE) / (MAX_DISTANCE * 1.5)
    
    def start(self):
        self.running = True
        self.display_thread = threading.Thread(target=self._display_thread)
        self.display_thread.daemon = True
        self.display_thread.start()
        return True
    
    def stop(self):
        self.running = False
        if self.display_thread:
            self.display_thread.join(timeout=1.0)
        pygame.quit()
    
    def _display_thread(self):
        try:
            while self.running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False
                        break
                
                self._update_display()
                self.clock.tick(30)  # 30 FPS maximum
        
        except Exception as e:
            print(f"Erreur dans le thread d'affichage: {e}")
        finally:
            pygame.quit()
    
    def _update_display(self):
        # Obtenir les données stabilisées
        scan_data, obstacle_points, closest_distance, closest_angle = self.lidar_processor.get_stable_data()
        
        # Effacer l'écran
        self.screen.fill(BLACK)
        
        # Dessiner le fond et les éléments de référence
        self._draw_reference_elements()
        
        # Dessiner les points du scan
        for angle, distance in scan_data.items():
            if -VIEW_ANGLE <= angle <= VIEW_ANGLE and distance <= MAX_DISTANCE:
                x, y = self._polar_to_cartesian(angle, distance)
                pygame.draw.circle(self.screen, WHITE, (x, y), 2)
        
        # Dessiner les obstacles
        for angle, distance, in_path in obstacle_points:
            x, y = self._polar_to_cartesian(angle, distance)
            color = RED if in_path else YELLOW
            pygame.draw.circle(self.screen, color, (x, y), 4)
        
        # Afficher la distance la plus proche
        if closest_distance < float('inf'):
            # Dessiner une ligne vers l'obstacle le plus proche
            x, y = self._polar_to_cartesian(closest_angle, closest_distance)
            pygame.draw.line(self.screen, GREEN, (self.center_x, self.center_y), (x, y), 2)
            
            # Afficher la distance
            distance_text = self.font.render(f"Distance: {closest_distance:.2f} cm", True, WHITE)
            self.screen.blit(distance_text, (10, 10))
            
            # Afficher un avertissement si nécessaire
            if closest_distance < 50:  # Alerte si moins de 50 cm
                warning_text = self.font.render("FREINAGE RECOMMANDÉ!", True, RED)
                self.screen.blit(warning_text, (10, 50))
        
        # Afficher les informations de la voiture
        vehicle_info = self.small_font.render(f"Largeur: {VEHICLE_WIDTH} cm", True, WHITE)
        self.screen.blit(vehicle_info, (10, WINDOW_SIZE[1] - 30))
        
        # Mettre à jour l'affichage
        pygame.display.flip()
    
    def _draw_reference_elements(self):
        # Dessiner le cercle de référence de distance
        for r in range(100, MAX_DISTANCE + 1, 100):
            radius = int(r * self.scale)
            pygame.draw.circle(self.screen, (30, 30, 30), (self.center_x, self.center_y), radius, 1)
            # Étiquette de distance
            label = self.small_font.render(f"{r} cm", True, (100, 100, 100))
            self.screen.blit(label, (self.center_x + 5, self.center_y - radius - 10))
        
        # Dessiner les angles de référence
        for angle in range(-VIEW_ANGLE, VIEW_ANGLE + 1, 30):
            rads = math.radians(angle)
            x = self.center_x + int(MAX_DISTANCE * self.scale * math.sin(rads))
            y = self.center_y - int(MAX_DISTANCE * self.scale * math.cos(rads))
            pygame.draw.line(self.screen, (30, 30, 30), (self.center_x, self.center_y), (x, y), 1)
            
            # Étiquette d'angle
            if angle != 0:
                label = self.small_font.render(f"{angle}°", True, (100, 100, 100))
                self.screen.blit(label, (x + 5, y + 5))
        
        # Dessiner la voiture (rectangle simple)
        half_width = int((VEHICLE_WIDTH / 2) * self.scale)
        car_rect = pygame.Rect(self.center_x - half_width, self.center_y - 10, half_width * 2, 20)
        pygame.draw.rect(self.screen, BLUE, car_rect)
        
        # Dessiner le chemin de la voiture comme deux lignes parallèles
        half_width = int((VEHICLE_WIDTH / 2) * self.scale)
        
        # Ligne parallèle gauche
        left_x1 = self.center_x - half_width
        left_y1 = self.center_y
        left_x2 = self.center_x - half_width
        left_y2 = self.center_y - int(MAX_DISTANCE * self.scale)
        
        # Ligne parallèle droite
        right_x1 = self.center_x + half_width
        right_y1 = self.center_y
        right_x2 = self.center_x + half_width
        right_y2 = self.center_y - int(MAX_DISTANCE * self.scale)
        
        # Dessiner les deux lignes parallèles
        pygame.draw.line(self.screen, (0, 100, 255), (left_x1, left_y1), (left_x2, left_y2), 1)
        pygame.draw.line(self.screen, (0, 100, 255), (right_x1, right_y1), (right_x2, right_y2), 1)    
    def _polar_to_cartesian(self, angle, distance):
        """Convertit les coordonnées polaires en coordonnées cartésiennes pour l'affichage"""
        rads = math.radians(angle)
        x = self.center_x + int(distance * self.scale * math.sin(rads))
        y = self.center_y - int(distance * self.scale * math.cos(rads))
        return x, y


#Communication Arduino 
import serial
import time
import threading


def main():
    try:
        # Initialiser le processeur LIDAR
        lidar_processor = LidarProcessor()
        if not lidar_processor.start():
            print("Impossible de démarrer le LIDAR. Vérifiez la connexion.")
            return
        
        # Initialiser le visualiseur
        visualizer = LidarVisualizer(lidar_processor)
        visualizer.start()
        
        # Initialiser la communication avec Arduino
        # Ajustez le port en fonction de votre configuration
        
        
        # Attendre que l'utilisateur ferme l'application
        try:
            while visualizer.running and lidar_processor.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Programme interrompu par l'utilisateur")
        
        # Nettoyer
        visualizer.stop()
        lidar_processor.stop()
        
        
    except Exception as e:
        print(f"Erreur dans le programme principal: {e}")
    finally:
        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    main()