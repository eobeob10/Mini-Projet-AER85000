import tkinter as tk
from tkinter import ttk
import math
from enum import Enum


# Constantes du projet
ALTITUDE_MAX = 40000    # Altitude maximale en pieds (pour la logique)
TAUX_MAX = 800.0        # Taux de montée maximal en m/min
ANGLE_MAX = 16.0        # Angle d'attaque maximal en degrés

# Temps de simulation
TIME_STEP_MS = 100      # Mise à jour toutes les 100 ms
DT = TIME_STEP_MS / 1000.0  # dt en secondes (0.1 s)

# Dimensions du canvas
CANVAS_WIDTH = 600
CANVAS_HEIGHT = 800

# Facteur d'exagération pour l'affichage
EXAG_DISPLAY = 0.018

# Définition des états avioniques
class EtatAvionique(Enum):
    AU_SOL = 0
    CHANGEMENT_ALT = 1
    VOL_CROISIÈRE = 2

# Classe représentant l'état du système avionique
class SystemeAvion:
    def __init__(self):
        self.altitude_actuelle = 0      # en pieds
        self.taux_monte = 0           # en m/min
        self.angle_attaque = 0      # en degrés
        self.puissance_moteur = 0       # en pourcentage
        self.etat = EtatAvionique.AU_SOL

# Fonction pour encoder une valeur en BCD sur un nombre de chiffres donné (simulation)
def encode_BCD(value, digits):
    bcd = 0
    for i in range(digits):
        digit = value % 10
        bcd |= (digit << (4 * i))
        value //= 10
    return bcd

# Encodage ARINC429 (simulation)
def encoder_ARINC429(sys_avion):
    # Label 001: Bits [13:28] pour l'altitude et Bits [11:12] pour l'état avionique
    word1 = ((sys_avion.altitude_actuelle & 0xFFFF) << 13) | ((sys_avion.etat.value & 0x03) << 11)
    # Label 002: Taux de montée en BCD sur 4 chiffres (résolution 0.1 m/min)
    taux_val = int(round(abs(sys_avion.taux_monte) * 10))
    word2 = encode_BCD(taux_val, 4)
    # Label 003: Angle d'attaque en BCD sur 3 chiffres (résolution 0.1°)
    angle_val = int(round(abs(sys_avion.angle_attaque) * 10))
    word3 = encode_BCD(angle_val, 3)
    return word1, word2, word3

# Fonction calculateur mise à jour pour gérer à la fois la montée et la descente
def calculateur(altitude_desiree, taux_entree, angle_entree, puissance, sys_avion, dt=1.0):
    # Si l'avion est en VOL_CROISIÈRE et que la nouvelle altitude désirée diffère, passer en CHANGEMENT_ALT
    if sys_avion.etat == EtatAvionique.VOL_CROISIÈRE:
        if altitude_desiree != sys_avion.altitude_actuelle:
            sys_avion.etat = EtatAvionique.CHANGEMENT_ALT


    # Transition de AU_SOL à CHANGEMENT_ALT
    if sys_avion.etat == EtatAvionique.AU_SOL:
        if (altitude_desiree > 0 and puissance > 0) or (taux_entree != 0 and angle_entree != 0):
            sys_avion.etat = EtatAvionique.CHANGEMENT_ALT
            if taux_entree == 0 and angle_entree == 0:
                taux_entree = 100
                angle_entree = 5

    # Gestion de l'état CHANGEMENT_ALT
    if sys_avion.etat == EtatAvionique.CHANGEMENT_ALT:
        taux_calcule = (puissance / 10.0) * 100.0
        # Si on monte
        if sys_avion.altitude_actuelle < altitude_desiree:
            if (altitude_desiree - sys_avion.altitude_actuelle) < 1000:
                taux_calcule *= 0.5
            # taux_calcule reste positif
        # Si on descend
        elif sys_avion.altitude_actuelle > altitude_desiree:
            if (sys_avion.altitude_actuelle - altitude_desiree) < 1000:
                taux_calcule *= 0.5
            taux_calcule = -taux_calcule  # Inversion du signe pour la descente
        sys_avion.taux_monte = taux_calcule
        sys_avion.angle_attaque = angle_entree
        sys_avion.puissance_moteur = puissance

        if sys_avion.angle_attaque > 15.0:
            print("Avertissement: Angle de décrochage dépassé!")
        # Passage en VOL_CROISIÈRE si l'altitude désirée est atteinte
        if (sys_avion.altitude_actuelle == altitude_desiree and altitude_desiree != 0) or sys_avion.altitude_actuelle >= ALTITUDE_MAX:
            sys_avion.etat = EtatAvionique.VOL_CROISIÈRE
            sys_avion.taux_monte = 0

    if sys_avion.etat == EtatAvionique.VOL_CROISIÈRE:
        sys_avion.taux_monte = 0
        sys_avion.puissance_moteur = puissance

    # Mise à jour de l'altitude (conversion tenant compte de dt)
    delta_alt = int((sys_avion.taux_monte * 3.28084) * (dt / 60.0))
    print(delta_alt)
    sys_avion.altitude_actuelle += delta_alt
    if sys_avion.altitude_actuelle < 0:
        sys_avion.altitude_actuelle = 0
    if sys_avion.altitude_actuelle > ALTITUDE_MAX:
        sys_avion.altitude_actuelle = ALTITUDE_MAX

    return encoder_ARINC429(sys_avion)

# Classe de l'interface graphique avec animations et réinitialisation
class AvionGUI:
    def __init__(self, root):

        self.root = root
        self.root.title("Simulation de Régulation d'Altitude")
        self.sys_avion = SystemeAvion()
        self.running = False

        # Cadres de l'interface
        self.control_frame = tk.Frame(root)
        self.control_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)
        self.canvas_frame = tk.Frame(root)
        self.canvas_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Contrôles d'entrée
        tk.Label(self.control_frame, text="Altitude désirée (ft):").grid(row=0, column=0, sticky="w")
        self.altitude_entry = tk.Entry(self.control_frame, width=10)
        self.altitude_entry.grid(row=0, column=1, padx=5)
        self.altitude_entry.insert(0, "10000")

        tk.Label(self.control_frame, text="Taux de montée (m/min):").grid(row=0, column=2, sticky="w")
        self.taux_entry = tk.Entry(self.control_frame, width=10)
        self.taux_entry.grid(row=0, column=3, padx=5)
        self.taux_entry.insert(0, "0")

        tk.Label(self.control_frame, text="Angle d'attaque (°):").grid(row=0, column=4, sticky="w")
        self.angle_entry = tk.Entry(self.control_frame, width=10)
        self.angle_entry.grid(row=0, column=5, padx=5)
        self.angle_entry.insert(0, "0")

        tk.Label(self.control_frame, text="Puissance du moteur (%):").grid(row=0, column=6, sticky="w")
        self.puissance_entry = tk.Entry(self.control_frame, width=10)
        self.puissance_entry.grid(row=0, column=7, padx=5)
        self.puissance_entry.insert(0, "50")

        # Boutons de contrôle
        self.start_button = tk.Button(self.control_frame, text="Démarrer Simulation", command=self.start_simulation)
        self.start_button.grid(row=1, column=0, columnspan=3, pady=5)
        self.stop_button = tk.Button(self.control_frame, text="Arrêter Simulation", command=self.stop_simulation)
        self.stop_button.grid(row=1, column=3, columnspan=2, pady=5)
        self.reset_button = tk.Button(self.control_frame, text="Réinitialiser Simulation", command=self.reset_simulation)
        self.reset_button.grid(row=1, column=5, columnspan=3, pady=5)

        # Canvas pour l'animation
        self.canvas = tk.Canvas(self.canvas_frame, width=CANVAS_WIDTH, height=CANVAS_HEIGHT, bg="skyblue")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Ligne de sol
        self.canvas.create_line(0, CANVAS_HEIGHT-20, CANVAS_WIDTH, CANVAS_HEIGHT-20, fill="green", width=4)

        # Règle verticale indiquant l'altitude
        self.draw_ruler()



        # Représentation de l'avion (triangle)
        self.plane = self.canvas.create_polygon(self.get_plane_coords(self.sys_avion.altitude_actuelle), fill="red")

        # Étiquette d'information sur l'état système
        self.info_label = tk.Label(root, text="", font=("Arial", 12))
        self.info_label.pack(side=tk.BOTTOM, pady=5)

    def get_plane_coords(self, altitude):
        effective_alt = altitude * EXAG_DISPLAY
        if effective_alt > (CANVAS_HEIGHT - 40):
            effective_alt = CANVAS_HEIGHT - 40
        y = CANVAS_HEIGHT - 20 - effective_alt
        x = CANVAS_WIDTH // 2
        size = 20
        return [x, y - size, x - size, y + size, x + size, y + size]

    def draw_ruler(self):
        RULER_X = 50
        TEXT_Y_OFFSET = 10

        self.canvas.create_line(RULER_X, 20, RULER_X, CANVAS_HEIGHT-20, fill="black", width=2)
        for alt in range(0, ALTITUDE_MAX+1, 5000):
            effective_alt = alt * EXAG_DISPLAY
            if effective_alt > (CANVAS_HEIGHT - 40):
                effective_alt = CANVAS_HEIGHT - 40
            y = CANVAS_HEIGHT - 20 - effective_alt
            self.canvas.create_line(RULER_X-5, y, RULER_X+5, y, fill="black", width=2)
            self.canvas.create_text(RULER_X-10, y + TEXT_Y_OFFSET, text=str(alt), anchor="e", font=("Arial", 10))

    def update_canvas(self):
        coords = self.get_plane_coords(self.sys_avion.altitude_actuelle)
        self.canvas.coords(self.plane, *coords)
        word1, word2, word3 = encoder_ARINC429(self.sys_avion)
        info_text = (
            f"Altitude: {self.sys_avion.altitude_actuelle} ft | "
            f"Vitesse: {self.sys_avion.taux_monte:.1f} m/min | "
            f"Puissance: {self.sys_avion.puissance_moteur}% | "
            f"Etat: {self.sys_avion.etat.name}\n"
            f"ARINC429: Label001: 0x{word1:08X}  Label002: 0x{word2:08X}  Label003: 0x{word3:08X}"
        )
        self.info_label.config(text=info_text)

    def simulation_step(self):
        try:
            altitude_desiree = int(self.altitude_entry.get())
            taux_entree = int(self.taux_entry.get())
            angle_entree = int(self.angle_entry.get())
            puissance = int(self.puissance_entry.get())
        except ValueError:
            print("Entrée invalide.")
            self.stop_simulation()
            return
        calculateur(altitude_desiree, taux_entree, angle_entree, puissance, self.sys_avion, dt=DT)
        self.update_canvas()
        if self.running:
            self.root.after(TIME_STEP_MS, self.simulation_step)

    def start_simulation(self):
        if not self.running:
            self.running = True
            self.simulation_step()

    def stop_simulation(self):
        self.running = False

    def reset_simulation(self):
        self.stop_simulation()
        self.sys_avion = SystemeAvion()
        self.update_canvas()
        print("Simulation réinitialisée.")

if __name__ == "__main__":
    root = tk.Tk()
    app = AvionGUI(root)
    root.mainloop()
