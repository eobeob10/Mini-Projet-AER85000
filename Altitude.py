import tkinter as tk
from tkinter import ttk
import math
from enum import Enum

# -- Constantes du projet
ALTITUDE_MAX = 40000    # Altitude maximale en pieds (pour la logique)
TAUX_MAX = 800.0        # Taux de montée maximal en m/min
ANGLE_MAX = 16.0        # Angle d'attaque maximal en degrés

TIME_STEP_MS = 100      # Mise à jour toutes les 100 ms
DT = TIME_STEP_MS / 1000.0  # dt en secondes

CANVAS_WIDTH = 600
CANVAS_HEIGHT = 800

EXAG_DISPLAY = 0.018

# -- Définitions d'état avionique
class EtatAvionique(Enum):
    AU_SOL = 0
    CHANGEMENT_ALT = 1
    VOL_CROISIÈRE = 2

class SystemeAvion:
    def __init__(self):
        self.altitude_actuelle = 0    # en pieds
        self.taux_monte = 0.0         # en m/min
        self.angle_attaque = 0.0      # en degrés
        self.puissance_moteur = 0     # en pourcentage
        self.etat = EtatAvionique.AU_SOL

# --------------------------
# ARINC429 Encoding Utilities
# --------------------------
def compute_parity(word):
    """Calcule la parité impaire sur le mot 32 bits (avant ajout du bit de parité).
       Si le nombre de 1 est pair, retourne 1 afin d'obtenir une parité impaire."""
    ones = bin(word).count("1")
    parity_bit = 1 if ones % 2 == 0 else 0
    return parity_bit

def bcd_encode(value, digits):
    """Encode un entier non négatif en BCD sur le nombre de chiffres spécifié."""
    s = str(value).zfill(digits)
    bcd = 0
    for char in s:
        bcd = (bcd << 4) | int(char)
    return bcd

def encoder_ARINC429(sys_avion):
    """
    Génère trois mots ARINC429 32 bits corrigés selon les spécifications :
      - \textbf{Altitude (Label 001)} :
          \begin{itemize}
            \item Bits 1-8 : Label (0x01)
            \item Bits 9-10 : Réservés (0)
            \item Bits 11-12 : État avionique (0 pour AU_SOL, 1 pour CHANGEMENT_ALT, 2 pour VOL_CROISIÈRE)
            \item Bits 13-28 : Altitude (16 bits)
            \item Bit 29 : Réservé (0)
            \item Bits 30-31 : SSM fixé à 0x03
            \item Bit 32 : Parité impaire
          \end{itemize}
      - \textbf{Taux de montée (Label 002)} :
          \begin{itemize}
            \item Encodé en BCD sur 4 chiffres (16 bits) avec résolution 0.1 m/min.
          \end{itemize}
      - \textbf{Angle d'attaque (Label 003)} :
          \begin{itemize}
            \item Encodé en BCD sur 3 chiffres (12 bits) puis décalé de 4 bits pour occuper 16 bits, avec résolution 0.1°.
          \end{itemize}
    """
    # --- Mot d'altitude (Label 001)
    # Bits : [31-24]=label, [23-22]=0, [21-20]=état, [19-4]=altitude, [3]=0, [2-1]=SSM
    altitude_word = (0x01 << 24) | ((sys_avion.etat.value & 0x03) << 20) | ((sys_avion.altitude_actuelle & 0xFFFF) << 4) | (0x03 << 1)
    altitude_word |= compute_parity(altitude_word)
    
    # --- Mot du taux de montée (Label 002) – Encodage BCD sur 4 chiffres
    rate_value = int(round(abs(sys_avion.taux_monte) * 10))  # Résolution 0.1 m/min
    rate_bcd = bcd_encode(rate_value, 4)  # 4 chiffres -> 16 bits
    taux_word = (0x02 << 24) | (rate_bcd << 4) | (0x03 << 1)
    taux_word |= compute_parity(taux_word)
    
    # --- Mot de l'angle d'attaque (Label 003) – Encodage BCD sur 3 chiffres
    angle_value = int(round(abs(sys_avion.angle_attaque) * 10))  # Résolution 0.1 degré
    angle_bcd = bcd_encode(angle_value, 3)  # 3 chiffres -> 12 bits
    # Décalage de 4 bits pour occuper un champ de 16 bits
    angle_bcd = angle_bcd << 4
    angle_word = (0x03 << 24) | (angle_bcd << 4) | (0x03 << 1)
    angle_word |= compute_parity(angle_word)
    
    return altitude_word, taux_word, angle_word

# --------------------------
# AFDX Encoding Utilities (Simulation)
# --------------------------
def encoder_AFDX(sys_avion):
    """
    Simule l'encodage AFDX en packant chaque mesure dans un champ de 32 bits.
    Chaque trame combine un VLID 16 bits avec une mesure 16 bits.
    VLIDs utilisés :
      - 0x1001 pour l'altitude
      - 0x1002 pour le taux de montée
      - 0x1003 pour l'angle d'attaque
    """
    vlid_alt = 0x1001
    frame_alt = ((vlid_alt & 0xFFFF) << 16) | (sys_avion.altitude_actuelle & 0xFFFF)

    vlid_rate = 0x1002
    rate_value = int(round(abs(sys_avion.taux_monte) * 10))
    frame_rate = ((vlid_rate & 0xFFFF) << 16) | (rate_value & 0xFFFF)

    vlid_angle = 0x1003
    angle_value = int(round(abs(sys_avion.angle_attaque) * 10))
    frame_angle = ((vlid_angle & 0xFFFF) << 16) | (angle_value & 0xFFFF)

    return frame_alt, frame_rate, frame_angle

# --------------------------
# Calculateur de l'évolution du système avionique
# --------------------------
def calculateur(altitude_desiree, taux_entree, angle_entree, puissance, sys_avion, dt=1.0):
    # Transition : Si l'avion est en vol de croisière et que l'altitude désirée change, passer en CHANGEMENT_ALT.
    if sys_avion.etat == EtatAvionique.VOL_CROISIÈRE:
        if altitude_desiree != sys_avion.altitude_actuelle:
            sys_avion.etat = EtatAvionique.CHANGEMENT_ALT

    # Transition de AU_SOL vers CHANGEMENT_ALT.
    if sys_avion.etat == EtatAvionique.AU_SOL:
        if (altitude_desiree > 0 and puissance > 0) or (taux_entree != 0 and angle_entree != 0):
            sys_avion.etat = EtatAvionique.CHANGEMENT_ALT
            if taux_entree == 0 and angle_entree == 0:
                taux_entree = 100
                angle_entree = 5

    if sys_avion.etat == EtatAvionique.CHANGEMENT_ALT:
        taux_calcule = (puissance / 10.0) * 100.0
        angle_entree = 5 if angle_entree == 0 else angle_entree
        # Si montée
        if sys_avion.altitude_actuelle < altitude_desiree:
            if (altitude_desiree - sys_avion.altitude_actuelle) < 1000:
                taux_calcule *= 0.5
        elif sys_avion.altitude_actuelle > altitude_desiree:
            if (sys_avion.altitude_actuelle - altitude_desiree) < 1000:
                taux_calcule *= 0.5
            taux_calcule = -taux_calcule  # Pour la descente
            angle_entree = -angle_entree  # Inversion de l'angle pour descente
        sys_avion.taux_monte = taux_calcule
        sys_avion.angle_attaque = angle_entree
        sys_avion.puissance_moteur = puissance

        if sys_avion.angle_attaque > 15.0:
            print("Avertissement: Angle de décrochage dépassé!")
        # Passage en VOL_CROISIÈRE lorsque l'altitude désirée est atteinte ou que l'altitude maximale est franchie
        if (sys_avion.altitude_actuelle == altitude_desiree and altitude_desiree != 0) or sys_avion.altitude_actuelle >= ALTITUDE_MAX:
            sys_avion.etat = EtatAvionique.VOL_CROISIÈRE
            sys_avion.taux_monte = 0

    if sys_avion.etat == EtatAvionique.VOL_CROISIÈRE:
        sys_avion.taux_monte = 0
        sys_avion.angle_attaque = 0
        sys_avion.puissance_moteur = puissance

    # Mise à jour de l'altitude : conversion du taux de montée (m/min) en ft/min (1 m/min ≃ 3.28084 ft/min)
    delta_alt = int((sys_avion.taux_monte * 3.28084) * (dt / 60.0))
    sys_avion.altitude_actuelle += delta_alt
    sys_avion.altitude_actuelle = max(0, min(sys_avion.altitude_actuelle, ALTITUDE_MAX))

    return encoder_ARINC429(sys_avion)

# --------------------------
# Interface Graphique
# --------------------------
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

        # Canevas pour l'animation
        self.canvas = tk.Canvas(self.canvas_frame, width=CANVAS_WIDTH, height=CANVAS_HEIGHT, bg="skyblue")
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Ligne de sol
        self.canvas.create_line(0, CANVAS_HEIGHT-20, CANVAS_WIDTH, CANVAS_HEIGHT-20, fill="green", width=4)

        # Règle d'altitude
        self.draw_ruler()

        # Représentation de l'avion (triangle)
        self.plane = self.canvas.create_polygon(self.get_plane_coords(self.sys_avion.altitude_actuelle), fill="red")

        # Label d'information à droite
        self.info_label = tk.Label(self.canvas_frame, text="", font=("Arial", 12), justify="left", anchor="n")
        self.info_label.pack(side=tk.RIGHT, padx=10, pady=10, fill=tk.Y)

    def get_plane_coords(self, altitude):
        effective_alt = altitude * EXAG_DISPLAY
        if effective_alt > (CANVAS_HEIGHT - 40):
            effective_alt = CANVAS_HEIGHT - 40
        y = CANVAS_HEIGHT - 20 - effective_alt
        x = CANVAS_WIDTH // 2
        size = 20
        return [x, y - size, x - size, y + size, x + size, y + size]

    def draw_ruler(self):
        ruler_x = 50
        self.canvas.create_line(ruler_x, 20, ruler_x, CANVAS_HEIGHT-20, fill="black", width=2)
        for alt in range(0, ALTITUDE_MAX+1, 5000):
            effective_alt = alt * EXAG_DISPLAY
            if effective_alt > (CANVAS_HEIGHT - 40):
                effective_alt = CANVAS_HEIGHT - 40
            y = CANVAS_HEIGHT - 20 - effective_alt
            self.canvas.create_line(ruler_x-5, y, ruler_x+5, y, fill="black", width=2)
            self.canvas.create_text(ruler_x-10, y, text=str(alt), anchor="e", font=("Arial", 10))

    def update_canvas(self):
        coords = self.get_plane_coords(self.sys_avion.altitude_actuelle)
        self.canvas.coords(self.plane, *coords)
        # Calcul des mots ARINC429 et AFDX
        arinc1, arinc2, arinc3 = encoder_ARINC429(self.sys_avion)
        afdx1, afdx2, afdx3 = encoder_AFDX(self.sys_avion)
        
        # Formatage des sorties en hexadécimal et en binaire sur 32 bits
        arinc_text = (
            f"ARINC429:\n"
            f"  Altitude (Label001): 0x{arinc1:08X} | Bits: {format(arinc1, '032b')}\n"
            f"  Taux (Label002):     0x{arinc2:08X} | Bits: {format(arinc2, '032b')}\n"
            f"  Angle (Label003):    0x{arinc3:08X} | Bits: {format(arinc3, '032b')}"
        )
        afdx_text = (
            f"AFDX:\n"
            f"  Frame Altitude: 0x{afdx1:08X} | Bits: {format(afdx1, '032b')}\n"
            f"  Frame Taux:     0x{afdx2:08X} | Bits: {format(afdx2, '032b')}\n"
            f"  Frame Angle:    0x{afdx3:08X} | Bits: {format(afdx3, '032b')}"
        )
        info_text = (
            f"Altitude: {self.sys_avion.altitude_actuelle} ft | "
            f"Taux de montée: {self.sys_avion.taux_monte:.1f} m/min | "
            f"Puissance: {self.sys_avion.puissance_moteur}% | "
            f"Etat: {self.sys_avion.etat.name}\n\n"
            f"{arinc_text}\n\n"
            f"{afdx_text}"
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
