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
CANVAS_HEIGHT = 400

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
    """Compute odd parity for the 31-bit word (before adding the parity bit).
       This function counts the number of 1's in the 31 LSBs;
       if that count is even, returns 1 so that overall count becomes odd."""
    ones = bin(word).count("1")
    parity_bit = 1 if ones % 2 == 0 else 0
    return parity_bit

def compute_arinc429_word(label, sdi, data, ssm):
    """
    Assemble a 32-bit ARINC429 word with:
      - label: 8 bits (bits 1-8)
      - sdi: 2 bits (bits 9-10)
      - data: 19 bits (bits 11-29)
      - ssm: 2 bits (bits 30-31)
      - parity: 1 bit (bit 32, odd parity)
    """
    word = (label & 0xFF) << 24         # Label in bits 1-8 (MSBs)
    word |= (sdi & 0x03) << 22            # SDI in bits 9-10
    word |= (data & 0x7FFFF) << 3         # Data in bits 11-29 (19 bits)
    word |= (ssm & 0x03) << 1             # SSM in bits 30-31
    parity_bit = compute_parity(word)
    word |= parity_bit                    # Bit 32: Parity
    return word

def encoder_ARINC429(sys_avion):
    """
    Generate three ARINC429 32-bit words:
      - Altitude word: Uses label 0x01, with altitude in the data field and aircraft state in SDI.
      - Rate word: Uses label 0x02, encoding the rate (multiplied by 10 for 0.1 m/min resolution).
      - Angle word: Uses label 0x03, encoding the angle (multiplied by 10 for 0.1° resolution).
    SSM is set to 0x03 (normal conditions) in all cases.
    """
    altitude_word = compute_arinc429_word(
        label=0x01,
        sdi=sys_avion.etat.value,            # Aircraft state as SDI (2 bits)
        data=sys_avion.altitude_actuelle,      # Altitude in feet (assumed to fit in 19 bits)
        ssm=0x03
    )
    rate_val = int(round(abs(sys_avion.taux_monte) * 10))
    rate_word = compute_arinc429_word(
        label=0x02,
        sdi=0,
        data=rate_val,
        ssm=0x03
    )
    angle_val = int(round(abs(sys_avion.angle_attaque) * 10))
    angle_word = compute_arinc429_word(
        label=0x03,
        sdi=0,
        data=angle_val,
        ssm=0x03
    )
    return altitude_word, rate_word, angle_word

# --------------------------
# AFDX Encoding Utilities (Simulation)
# --------------------------
def encoder_AFDX(sys_avion):
    """
    Simulate AFDX encoding by packing each measurement into a 32-bit field.
    Each frame combines a 16-bit Virtual Link Identifier (VLID) with a 16-bit measurement:
      Frame = (VLID << 16) | measurement
    We use the following VLIDs:
      - 0x1001 for altitude
      - 0x1002 for rate
      - 0x1003 for angle
    The rate and angle measurements are scaled by 10 for 0.1 resolution.
    """
    vlid_alt = 0x1001
    frame_alt = ((vlid_alt & 0xFFFF) << 16) | (sys_avion.altitude_actuelle & 0xFFFF)

    vlid_rate = 0x1002
    rate_val = int(round(abs(sys_avion.taux_monte) * 10)) & 0xFFFF
    frame_rate = ((vlid_rate & 0xFFFF) << 16) | rate_val

    vlid_angle = 0x1003
    angle_val = int(round(abs(sys_avion.angle_attaque) * 10)) & 0xFFFF
    frame_angle = ((vlid_angle & 0xFFFF) << 16) | angle_val

    return frame_alt, frame_rate, frame_angle

# --------------------------
# Calculateur de l'évolution du système avionique
# --------------------------
def calculateur(altitude_desiree, taux_entree, angle_entree, puissance, sys_avion, dt=1.0):
    # Transition: If currently in cruise and altitude desire changes, set state to CHANGEMENT_ALT.
    if sys_avion.etat == EtatAvionique.VOL_CROISIÈRE:
        if altitude_desiree != sys_avion.altitude_actuelle:
            sys_avion.etat = EtatAvionique.CHANGEMENT_ALT

    # Transition from AU_SOL to CHANGEMENT_ALT.
    if sys_avion.etat == EtatAvionique.AU_SOL:
        if (altitude_desiree > 0 and puissance > 0) or (taux_entree != 0 and angle_entree != 0):
            sys_avion.etat = EtatAvionique.CHANGEMENT_ALT
            if taux_entree == 0 and angle_entree == 0:
                taux_entree = 100
                angle_entree = 5

    if sys_avion.etat == EtatAvionique.CHANGEMENT_ALT:
        taux_calcule = (puissance / 10.0) * 100.0
        angle_entree = 5 if angle_entree == 0 else angle_entree
        # Si on monte
        if sys_avion.altitude_actuelle < altitude_desiree:
            if (altitude_desiree - sys_avion.altitude_actuelle) < 1000:
                taux_calcule *= 0.5
        elif sys_avion.altitude_actuelle > altitude_desiree:
            if (sys_avion.altitude_actuelle - altitude_desiree) < 1000:
                taux_calcule *= 0.5
            taux_calcule = -taux_calcule  # Inversion du signe pour la descente
            angle_entree = -angle_entree  # Inversion du signe pour la descente
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
        sys_avion.angle_attaque = 0
        sys_avion.puissance_moteur = puissance

    # Update altitude: conversion from rate (m/min) to ft/min (1 m/min ≃ 3.28084 ft/min)
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

        # Interface frames
        self.control_frame = tk.Frame(root)
        self.control_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)
        self.canvas_frame = tk.Frame(root)
        self.canvas_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Input controls
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

        # Control buttons
        self.start_button = tk.Button(self.control_frame, text="Démarrer Simulation", command=self.start_simulation)
        self.start_button.grid(row=1, column=0, columnspan=3, pady=5)
        self.stop_button = tk.Button(self.control_frame, text="Arrêter Simulation", command=self.stop_simulation)
        self.stop_button.grid(row=1, column=3, columnspan=2, pady=5)
        self.reset_button = tk.Button(self.control_frame, text="Réinitialiser Simulation", command=self.reset_simulation)
        self.reset_button.grid(row=1, column=5, columnspan=3, pady=5)

        # Canvas for animation
        self.canvas = tk.Canvas(self.canvas_frame, width=CANVAS_WIDTH, height=CANVAS_HEIGHT, bg="skyblue")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Ground line
        self.canvas.create_line(0, CANVAS_HEIGHT-20, CANVAS_WIDTH, CANVAS_HEIGHT-20, fill="green", width=4)

        # Altitude ruler
        self.draw_ruler()

        # Airplane representation (triangle)
        self.plane = self.canvas.create_polygon(self.get_plane_coords(self.sys_avion.altitude_actuelle), fill="red")

        # Info label to display state and encoded values
        self.info_label = tk.Label(root, text="", font=("Arial", 12), justify="left")
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
        ruler_x = 30
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
        # Compute encoded ARINC429 and AFDX words
        arinc1, arinc2, arinc3 = encoder_ARINC429(self.sys_avion)
        afdx1, afdx2, afdx3 = encoder_AFDX(self.sys_avion)
        
        # Format outputs: hexadecimal and full 32-bit binary strings
        arinc_text = (
            f"ARINC429:\n"
            f"  Label001: 0x{arinc1:08X} | Bits: {format(arinc1, '032b')}\n"
            f"  Label002: 0x{arinc2:08X} | Bits: {format(arinc2, '032b')}\n"
            f"  Label003: 0x{arinc3:08X} | Bits: {format(arinc3, '032b')}"
        )
        afdx_text = (
            f"AFDX:\n"
            f"  Frame Altitude: 0x{afdx1:08X} | Bits: {format(afdx1, '032b')}\n"
            f"  Frame Taux:     0x{afdx2:08X} | Bits: {format(afdx2, '032b')}\n"
            f"  Frame Angle:    0x{afdx3:08X} | Bits: {format(afdx3, '032b')}"
        )
        info_text = (
            f"Altitude: {self.sys_avion.altitude_actuelle} ft | "
            f"Taux de monté: {self.sys_avion.taux_monte:.1f} m/min | "
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
