"""
Interface de commande du servomoteur Dynamixel AX-12A — Groupe 11
------------------------------------------------------------------
Communique avec la carte Arduino MEGA par le port série (115200 bauds)
et utilise les commandes du programme Arduino :

    PING, ENABLE, DISABLE, JOINT, WHEEL,
    p<angle>  (ex. p150), v<vitesse> (ex. v512, v-300),
    POSITION, SPEED, TEMPERATURE, VOLTAGE, LOAD

Réponses attendues de l'Arduino :
    "Ping : 0/1", "Position : ...", "Vitesse : ...", "Temperature : ...",
    "Tension : ...", "Charge : ...", "Dynamixel detected", "Error! ..."

Installation :  pip install pyside6 pyserial matplotlib numpy
Lancement    :  python interface_dynamixel.py
"""

import sys
import math
import re
import time
import threading

import numpy as np
import serial
import serial.tools.list_ports

from PySide6.QtCore import Qt, QThread, Signal, QTimer
from PySide6.QtGui import QFont, QTextCursor, QTextCharFormat, QColor
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFrame, QLabel, QPushButton, QComboBox,
    QHBoxLayout, QVBoxLayout, QGridLayout, QSlider, QSpinBox, QDoubleSpinBox,
    QCheckBox, QPlainTextEdit, QLineEdit, QButtonGroup, QSizePolicy,
)

from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection


# =====================================================================
#  Paramètres — à adapter si le programme Arduino change
# =====================================================================
BAUDRATE = 115200
DELAI_RESET_ARDUINO = 2000       # ms : la carte redémarre à l'ouverture du port

CMD_PING = "PING"
CMD_ENABLE, CMD_DISABLE = "ENABLE", "DISABLE"
CMD_JOINT, CMD_WHEEL = "JOINT", "WHEEL"
CMD_LECTURE = {                  # grandeur -> commande envoyée
    "position": "POSITION",
    "vitesse": "SPEED",
    "temperature": "TEMPERATURE",
    "tension": "VOLTAGE",
    "charge": "LOAD",
}
# Réponses de l'Arduino -> grandeur
REPONSES = {
    "position": re.compile(r"^\s*Position\s*:\s*(-?\d+(?:[.,]\d+)?)", re.I),
    "vitesse": re.compile(r"^\s*Vitesse\s*:\s*(-?\d+(?:[.,]\d+)?)", re.I),
    "temperature": re.compile(r"^\s*Temp[ée]rature\s*:\s*(-?\d+(?:[.,]\d+)?)", re.I),
    "tension": re.compile(r"^\s*Tension\s*:\s*(-?\d+(?:[.,]\d+)?)", re.I),
    "charge": re.compile(r"^\s*Charge\s*:\s*(-?\d+(?:[.,]\d+)?)", re.I),
}
REP_PING = re.compile(r"^\s*Ping\s*:\s*([01])", re.I)

# Caractéristiques de l'AX-12A (12 V)
VITESSE_MAX_TR_MIN = 59.0        # vitesse à vide
UNITE_VITESSE_SERVO = 0.111      # tr/min par unité en mode servo
VITESSE_INITIALE = 512           # vitesse envoyée à la connexion (mode servo)

# Couleurs
BLEU = "#0B3C5D"
ORANGE = "#E67E22"
FOND = "#F3F5F8"
CARTE = "#FFFFFF"
BORD = "#E1E6ED"
TEXTE = "#1F2933"
GRIS = "#6B7785"
VERT = "#2EAD5B"
ROUGE = "#D64545"


# =====================================================================
#  Communication série (dans un fil d'exécution séparé)
# =====================================================================
class LiaisonSerie(QThread):
    ligne_recue = Signal(str)
    erreur = Signal(str)
    ouverte = Signal()

    def __init__(self, port, baudrate=BAUDRATE):
        super().__init__()
        self.port, self.baudrate = port, baudrate
        self._ser = None
        self._actif = False
        self._verrou = threading.Lock()

    def run(self):
        try:
            self._ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        except Exception as e:
            self.erreur.emit(f"Impossible d'ouvrir {self.port} : {e}")
            return
        self._actif = True
        self.ouverte.emit()
        tampon = b""
        while self._actif:
            try:
                donnees = self._ser.read(self._ser.in_waiting or 1)
            except Exception as e:
                if self._actif:
                    self.erreur.emit(f"Liaison perdue : {e}")
                break
            if not donnees:
                continue
            tampon += donnees
            while b"\n" in tampon:
                ligne, tampon = tampon.split(b"\n", 1)
                texte = ligne.decode("utf-8", errors="replace").strip()
                if texte:
                    self.ligne_recue.emit(texte)
        try:
            self._ser.close()
        except Exception:
            pass

    def envoyer(self, commande):
        if not self._ser or not self._actif:
            return False
        with self._verrou:
            try:
                self._ser.write((commande + "\n").encode())
                return True
            except Exception as e:
                self.erreur.emit(f"Erreur d'envoi : {e}")
                return False

    def arreter(self):
        self._actif = False
        self.wait(1000)


# =====================================================================
#  Modèle 3D du Dynamixel AX-12A
# =====================================================================
def _rot_y(points, centre, phi):
    """Rotation autour d'un axe parallèle à y passant par 'centre'.
    phi > 0 : sens anti-horaire vu depuis l'avant du palonnier (+y)."""
    p = np.asarray(points, float) - centre
    c, s = math.cos(phi), math.sin(phi)
    x = c * p[:, 0] + s * p[:, 2]
    z = -s * p[:, 0] + c * p[:, 2]
    return np.column_stack([x, p[:, 1], z]) + centre


def _cercle(centre, r, y, n=40, phi0=0.0):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False) + phi0
    return np.column_stack([centre[0] + r * np.sin(a), np.full(n, y), centre[2] + r * np.cos(a)])


def _ombre(couleur, normale, lumiere=(0.45, 0.75, 0.6), mini=0.55):
    """Ombrage simple d'une face selon sa normale."""
    n = np.asarray(normale, float)
    n = n / (np.linalg.norm(n) + 1e-9)
    l = np.asarray(lumiere) / np.linalg.norm(lumiere)
    k = mini + (1 - mini) * max(0.0, float(np.dot(n, l)))
    c = QColor(couleur)
    return (c.redF() * k, c.greenF() * k, c.blueF() * k, 1.0)


class Modele3D(FigureCanvasQTAgg):
    # Dimensions réelles de l'AX-12A (mm) : 32 x 50 x 40
    L, H, P = 32.0, 50.0, 40.0
    CENTRE = np.array([0.0, 20.0, 10.0])      # centre du palonnier sur la face avant
    R_PALONNIER, E_PALONNIER = 9.5, 3.0
    R_ARBRE, L_ARBRE = 4.2, 38.0

    def __init__(self, parent=None):
        self.fig = Figure(figsize=(6, 5), facecolor=CARTE)
        super().__init__(self.fig)
        self.ax = self.fig.add_axes([0, 0, 1, 1], projection="3d")
        self.ax.set_facecolor(CARTE)
        self.ax.computed_zorder = False
        self.angle_deg = 150.0
        self._construire()
        self.vue_initiale()

    # ---------- construction ----------
    def _boite(self, x0, x1, y0, y1, z0, z1, couleur, bord="#15171A", lw=0.6, zorder=1):
        v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                      [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]])
        faces = [([0, 1, 2, 3], (0, 0, -1)), ([4, 5, 6, 7], (0, 0, 1)),
                 ([0, 1, 5, 4], (0, -1, 0)), ([2, 3, 7, 6], (0, 1, 0)),
                 ([1, 2, 6, 5], (1, 0, 0)), ([0, 3, 7, 4], (-1, 0, 0))]
        polys = [v[f] for f, _ in faces]
        couleurs = [_ombre(couleur, n) for _, n in faces]
        col = Poly3DCollection(polys, facecolors=couleurs, edgecolors=bord, linewidths=lw)
        col.set_zorder(zorder)
        self.ax.add_collection3d(col, autolim=False)
        return col

    def _construire(self):
        ax = self.ax
        L, H, P = self.L / 2, self.H / 2, self.P / 2
        C = self.CENTRE
        self.statiques_avant, self.statiques_arriere = [], []

        # Corps du servomoteur : capot inférieur, boîtier, capot supérieur
        self._boite(-L, L, -P, P, -H, -H + 9, "#2B2E32", zorder=1)
        corps = self._boite(-L, L, -P, P, -H + 9, H - 6, "#34373C", zorder=1.1)
        self._boite(-L, L, -P, P, H - 6, H, "#2B2E32", zorder=1.2)
        # Trous de fixation sur les côtés (affichés seulement du côté visible)
        self.trous_cote = {}
        for signe in (-1, 1):
            segs = []
            for y in np.linspace(-P + 6, P - 6, 4):
                for z in (-H + 4.5, H - 3.0):
                    a = np.linspace(0, 2 * math.pi, 14)
                    segs.append(np.column_stack([np.full(14, signe * (L + 0.05)), y + 1.3 * np.cos(a),
                                                 z + 1.3 * np.sin(a)]))
            lc = Line3DCollection(segs, colors="#0E0F11", linewidths=1.0)
            lc.set_zorder(2)
            ax.add_collection3d(lc, autolim=False)
            self.trous_cote[signe] = lc
        # Connecteurs (dessous)
        for x in (-8.0, 8.0):
            self._boite(x - 4.5, x + 4.5, -3, 3, -H - 3.5, -H, "#E9E4D8", bord="#9C9585", lw=0.5, zorder=2.5)
        # Palonnier arrière (libre)
        arr = _cercle(C, 7.5, -P - 0.3)
        col = Poly3DCollection([arr], facecolors=_ombre("#8C9299", (0, -1, 0)), edgecolors="#2A2D31", lw=0.6)
        col.set_zorder(0)
        ax.add_collection3d(col, autolim=False)
        self.statiques_arriere.append(col)
        # Cadran gradué 0 -> 300° autour du palonnier
        y_cad = P + 0.2
        for pos in range(0, 301, 10):
            phi = math.radians(pos - 150)
            r0, r1 = (11.2, 13.6) if pos % 50 == 0 else (12.2, 13.6)
            p0 = _rot_y([[0, y_cad, r0]], np.array([0, y_cad, 0]), phi)[0] + [C[0], 0, C[2]]
            p1 = _rot_y([[0, y_cad, r1]], np.array([0, y_cad, 0]), phi)[0] + [C[0], 0, C[2]]
            ax.plot([p0[0], p1[0]], [y_cad, y_cad], [p0[2], p1[2]],
                    color="#E8EBEF" if pos % 50 else "#FFFFFF", lw=1.3 if pos % 50 == 0 else 0.6, zorder=3)
        # zone morte 300° -> 360°
        a = np.radians(np.linspace(150, 210, 16))
        ax.plot(C[0] + 12.9 * np.sin(a), np.full(16, y_cad), C[2] + 12.9 * np.cos(a),
                color="#7A7F86", lw=3, alpha=0.8, zorder=3)

        # ---------- éléments mobiles ----------
        yf = P + self.E_PALONNIER                     # face avant du palonnier
        self.y_face = yf
        # flanc du palonnier (fixe : forme de révolution)
        n = 40
        c0, c1 = _cercle(C, self.R_PALONNIER, P), _cercle(C, self.R_PALONNIER, yf)
        flanc = [np.array([c0[i], c0[(i + 1) % n], c1[(i + 1) % n], c1[i]]) for i in range(n)]
        cf = [_ombre("#AEB4BB", (math.sin(2 * math.pi * i / n), 0, math.cos(2 * math.pi * i / n))) for i in range(n)]
        self.flanc = Poly3DCollection(flanc, facecolors=cf, edgecolors="none")
        self.flanc.set_zorder(4)
        ax.add_collection3d(self.flanc, autolim=False)
        self.face = Poly3DCollection([c1], facecolors=_ombre("#C8CDD3", (0, 1, 0.2)), edgecolors="#6E747B", lw=0.7)
        self.face.set_zorder(5)
        ax.add_collection3d(self.face, autolim=False)
        # trous du palonnier (tournent)
        self.trous = Poly3DCollection([], facecolors="#3A3E44", edgecolors="none")
        self.trous.set_zorder(6)
        ax.add_collection3d(self.trous, autolim=False)
        self.repere_palonnier = Line3DCollection([], colors=ORANGE, linewidths=2.4)
        self.repere_palonnier.set_zorder(7)
        ax.add_collection3d(self.repere_palonnier, autolim=False)

        # Arbre : cylindre (fixe) + traits (tournent)
        y0, y1 = yf, yf + self.L_ARBRE
        self.y_bout = y1
        a0, a1 = _cercle(C, self.R_ARBRE, y0, 32), _cercle(C, self.R_ARBRE, y1, 32)
        cyl = [np.array([a0[i], a0[(i + 1) % 32], a1[(i + 1) % 32], a1[i]]) for i in range(32)]
        cc = [_ombre("#E3E7EC", (math.sin(2 * math.pi * i / 32), 0, math.cos(2 * math.pi * i / 32)), mini=0.6)
              for i in range(32)]
        self.arbre = Poly3DCollection(cyl, facecolors=cc, edgecolors="none")
        self.arbre.set_zorder(8)
        ax.add_collection3d(self.arbre, autolim=False)
        self.bout = Poly3DCollection([a1], facecolors=_ombre("#F2F4F6", (0, 1, 0.2)), edgecolors="#8A9199", lw=0.6)
        self.bout.set_zorder(9)
        ax.add_collection3d(self.bout, autolim=False)
        self.traits = Line3DCollection([], linewidths=1.0)
        self.traits.set_zorder(10)
        ax.add_collection3d(self.traits, autolim=False)
        self.index = Line3DCollection([], colors=ORANGE, linewidths=2.6)
        self.index.set_zorder(11)
        ax.add_collection3d(self.index, autolim=False)

        self.mobiles_avant = [self.flanc, self.face, self.trous, self.repere_palonnier,
                              self.arbre, self.bout, self.traits, self.index]
        self.corps = corps

        self.texte_angle = ax.text2D(0.03, 0.95, "", transform=ax.transAxes, fontsize=11,
                                     color=BLEU, fontweight="bold")
        self.texte_info = ax.text2D(0.03, 0.90, "", transform=ax.transAxes, fontsize=8.5, color=GRIS)

        m = 34
        ax.set_xlim(-m, m)
        ax.set_ylim(-m + 12, m + 12)
        ax.set_zlim(-m + 7, m + 7)
        ax.set_box_aspect((1, 1, 1), zoom=1.32)
        ax.set_axis_off()
        self.mettre_a_jour(self.angle_deg)

    # ---------- mise à jour ----------
    def mettre_a_jour(self, angle_deg, info=""):
        self.angle_deg = angle_deg
        phi = math.radians(angle_deg - 150.0)        # 150° = palonnier en haut
        C = self.CENTRE
        yf = self.y_face + 0.15
        # trous du palonnier
        trous = [_cercle(C, 1.1, yf, 12) + [0, 0, 0]]
        for k in range(4):
            a = phi + math.pi / 4 + k * math.pi / 2
            cx, cz = C[0] + 6.0 * math.sin(a), C[2] + 6.0 * math.cos(a)
            trous.append(_cercle(np.array([cx, 0, cz]), 1.0, yf, 12))
        self.trous.set_verts(trous)
        # repère sur le palonnier
        p0 = [C[0] + 6.6 * math.sin(phi), yf, C[2] + 6.6 * math.cos(phi)]
        p1 = [C[0] + 9.4 * math.sin(phi), yf, C[2] + 9.4 * math.cos(phi)]
        self.repere_palonnier.set_segments([[p0, p1]])
        # traits longitudinaux de l'arbre
        segs, couleurs, larges = [], [], []
        r = self.R_ARBRE + 0.05
        for k in range(8):
            a = phi + k * math.pi / 4
            x, z = C[0] + r * math.sin(a), C[2] + r * math.cos(a)
            segs.append([[x, self.y_face, z], [x, self.y_bout, z]])
            couleurs.append(ORANGE if k == 0 else "#4A5058")
            larges.append(2.2 if k == 0 else 0.9)
        self.traits.set_segments(segs)
        self.traits.set_color(couleurs)
        self.traits.set_linewidth(larges)
        # index en bout d'arbre (croix + aiguille orange)
        yb = self.y_bout + 0.1
        seg_i = []
        for k, ln in ((0, 1.45), (2, 0.9), (4, 0.9), (6, 0.9)):
            a = phi + k * math.pi / 4
            seg_i.append([[C[0], yb, C[2]], [C[0] + self.R_ARBRE * ln * math.sin(a), yb,
                                              C[2] + self.R_ARBRE * ln * math.cos(a)]])
        self.index.set_segments(seg_i)
        self.index.set_color([ORANGE, "#4A5058", "#4A5058", "#4A5058"])
        self.index.set_linewidth([3.0, 1.0, 1.0, 1.0])

        self.texte_angle.set_text(f"θ = {angle_deg % 360:6.1f}°")
        self.texte_info.set_text(info)
        self._ordre_affichage()
        self.draw_idle()

    def _ordre_affichage(self):
        """Ordre d'affichage selon le côté d'où l'on regarde le servomoteur."""
        az = math.radians(self.ax.azim)
        vu_de_face = math.sin(az) * math.cos(math.radians(self.ax.elev)) > -0.05
        self.corps.set_zorder(1.1 if vu_de_face else 20)
        cote = 1 if math.cos(az) >= 0 else -1
        for signe, lc in self.trous_cote.items():
            lc.set_visible(signe == cote)
        for i, a in enumerate(self.mobiles_avant):
            a.set_zorder(4 + i if vu_de_face else 0.5)
        for a in self.statiques_arriere:
            a.set_zorder(0 if vu_de_face else 21)

    def vue_initiale(self):
        self.ax.view_init(elev=18, azim=62)
        self.mettre_a_jour(self.angle_deg)

    def vue_de_face(self):
        self.ax.view_init(elev=0, azim=90)
        self.mettre_a_jour(self.angle_deg)


# =====================================================================
#  Petits composants graphiques
# =====================================================================
class Voyant(QLabel):
    """Point de couleur : gris (inconnu), orange (attente), rouge, vert."""
    COULEURS = {"inconnu": "#B7C0CA", "attente": ORANGE, "non": ROUGE, "oui": VERT}

    def __init__(self, taille=12):
        super().__init__()
        self.setFixedSize(taille, taille)
        self._taille = taille
        self.etat("inconnu")

    def etat(self, e):
        c = self.COULEURS[e]
        self.setStyleSheet(f"background:{c}; border-radius:{self._taille // 2}px;"
                           f"border:2px solid {QColor(c).lighter(135).name()};")


def carte(titre=None):
    cadre = QFrame()
    cadre.setObjectName("carte")
    lay = QVBoxLayout(cadre)
    lay.setContentsMargins(14, 12, 14, 14)
    lay.setSpacing(8)
    if titre:
        t = QLabel(titre)
        t.setObjectName("titreCarte")
        lay.addWidget(t)
    return cadre, lay


class TuileDonnee(QFrame):
    def __init__(self, nom, unite, cle, parent_fenetre):
        super().__init__()
        self.setObjectName("tuile")
        self.cle = cle
        lay = QGridLayout(self)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setHorizontalSpacing(6)
        lay.setVerticalSpacing(0)
        self.nom = QLabel(nom)
        self.nom.setObjectName("nomTuile")
        self.valeur = QLabel("—")
        self.valeur.setObjectName("valeurTuile")
        self.unite = QLabel(unite)
        self.unite.setObjectName("uniteTuile")
        self.detail = QLabel("")
        self.detail.setObjectName("detailTuile")
        self.auto = QCheckBox("auto")
        self.auto.setToolTip("Inclure dans la lecture automatique")
        self.bouton = QPushButton("Lire")
        self.bouton.setObjectName("petit")
        self.bouton.clicked.connect(lambda: parent_fenetre.demander(cle))
        lay.addWidget(self.nom, 0, 0, 1, 2)
        lay.addWidget(self.auto, 0, 2, Qt.AlignRight)
        lay.addWidget(self.valeur, 1, 0, Qt.AlignLeft | Qt.AlignBottom)
        lay.addWidget(self.unite, 1, 1, Qt.AlignLeft | Qt.AlignBottom)
        lay.addWidget(self.bouton, 1, 2, Qt.AlignRight)
        lay.addWidget(self.detail, 2, 0, 1, 3)
        lay.setColumnStretch(1, 1)

    def afficher(self, valeur, detail=""):
        self.valeur.setText(valeur)
        self.detail.setText(detail)


# =====================================================================
#  Fenêtre principale
# =====================================================================
class Fenetre(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dynamixel AX-12A — Interface de commande (Groupe 11)")
        self.resize(1400, 860)
        self.liaison = None
        self.connecte = False

        # État du servomoteur (simulé localement, corrigé par les mesures)
        self.mode_continu = False
        self.couple = True
        self.angle = 150.0
        self.cible = 150.0
        self.vitesse = 512
        self.vitesse_mesuree = None
        self._t = time.perf_counter()

        self._construire_ui()
        self._actualiser_ports()
        self._appliquer_mode(False, envoyer=False)

        self.anim = QTimer(self, interval=40, timeout=self._animer)
        self.anim.start()
        self.lecture_auto = QTimer(self, timeout=self._tick_lecture_auto)
        self._file_auto = []
        self.minuteur_ping = QTimer(self, singleShot=True, timeout=self._ping_sans_reponse)

        self.journal("info", "Interface prête. Choisissez le port de la carte Arduino puis « Connecter ».")
        self.journal("info", "Sans connexion, les commandes sont simulées sur le modèle 3D.")

    # ---------------- construction de l'interface ----------------
    def _construire_ui(self):
        racine = QWidget()
        self.setCentralWidget(racine)
        principal = QVBoxLayout(racine)
        principal.setContentsMargins(16, 14, 16, 16)
        principal.setSpacing(12)

        # ---- barre du haut ----
        barre = QFrame()
        barre.setObjectName("barre")
        lb = QHBoxLayout(barre)
        lb.setContentsMargins(18, 10, 18, 10)
        titre = QLabel("Dynamixel AX-12A")
        titre.setObjectName("titre")
        sous = QLabel("Interface de commande · Groupe 11")
        sous.setObjectName("sousTitre")
        bloc = QVBoxLayout()
        bloc.setSpacing(0)
        bloc.addWidget(titre)
        bloc.addWidget(sous)
        lb.addLayout(bloc)
        lb.addStretch()

        lb.addWidget(QLabel("Port"))
        self.ports = QComboBox()
        self.ports.setMinimumWidth(190)
        lb.addWidget(self.ports)
        b_refresh = QPushButton("↻")
        b_refresh.setObjectName("icone")
        b_refresh.setToolTip("Actualiser la liste des ports")
        b_refresh.clicked.connect(self._actualiser_ports)
        lb.addWidget(b_refresh)
        self.b_connexion = QPushButton("Connecter")
        self.b_connexion.setObjectName("principal")
        self.b_connexion.clicked.connect(self._basculer_connexion)
        lb.addWidget(self.b_connexion)

        lb.addSpacing(18)
        sep = QFrame()
        sep.setFrameShape(QFrame.VLine)
        sep.setObjectName("sep")
        lb.addWidget(sep)
        lb.addSpacing(12)
        self.v_arduino = Voyant()
        lb.addWidget(self.v_arduino)
        lb.addWidget(QLabel("Arduino"))
        lb.addSpacing(14)
        self.v_servo = Voyant()
        lb.addWidget(self.v_servo)
        lb.addWidget(QLabel("Servomoteur"))
        self.b_ping = QPushButton("Ping")
        self.b_ping.setObjectName("secondaire")
        self.b_ping.setToolTip("Vérifier la présence du servomoteur")
        self.b_ping.clicked.connect(self.ping)
        lb.addWidget(self.b_ping)
        principal.addWidget(barre)

        # ---- corps : 3 colonnes ----
        corps = QHBoxLayout()
        corps.setSpacing(12)
        principal.addLayout(corps, 1)

        # Colonne gauche : commandes
        gauche = QVBoxLayout()
        gauche.setSpacing(12)
        corps.addLayout(gauche, 0)

        c, l = carte("Couple")
        rang = QHBoxLayout()
        self.b_couple_on = QPushButton("Activer")
        self.b_couple_off = QPushButton("Désactiver")
        for b in (self.b_couple_on, self.b_couple_off):
            b.setCheckable(True)
            b.setObjectName("segment")
            rang.addWidget(b)
        grp = QButtonGroup(self)
        grp.addButton(self.b_couple_on)
        grp.addButton(self.b_couple_off)
        self.b_couple_on.setChecked(True)
        self.b_couple_on.clicked.connect(lambda: self.commande_couple(True))
        self.b_couple_off.clicked.connect(lambda: self.commande_couple(False))
        l.addLayout(rang)
        aide = QLabel("Sans couple, l'arbre peut être tourné à la main.")
        aide.setObjectName("aide")
        aide.setWordWrap(True)
        l.addWidget(aide)
        gauche.addWidget(c)

        c, l = carte("Mode de rotation")
        rang = QHBoxLayout()
        self.b_servo = QPushButton("Servo")
        self.b_continu = QPushButton("Continu")
        for b in (self.b_servo, self.b_continu):
            b.setCheckable(True)
            b.setObjectName("segment")
            rang.addWidget(b)
        grp2 = QButtonGroup(self)
        grp2.addButton(self.b_servo)
        grp2.addButton(self.b_continu)
        self.b_servo.setChecked(True)
        self.b_servo.clicked.connect(lambda: self._appliquer_mode(False))
        self.b_continu.clicked.connect(lambda: self._appliquer_mode(True))
        l.addLayout(rang)
        self.aide_mode = QLabel("")
        self.aide_mode.setObjectName("aide")
        self.aide_mode.setWordWrap(True)
        l.addWidget(self.aide_mode)
        gauche.addWidget(c)

        self.carte_pos, l = carte("Position cible")
        rang = QHBoxLayout()
        self.s_pos = QSlider(Qt.Horizontal)
        self.s_pos.setRange(0, 3000)
        self.s_pos.setValue(1500)
        self.e_pos = QDoubleSpinBox()
        self.e_pos.setRange(0, 300)
        self.e_pos.setDecimals(1)
        self.e_pos.setSuffix(" °")
        self.e_pos.setValue(150)
        self.e_pos.setFixedWidth(92)
        self.s_pos.valueChanged.connect(lambda v: self.e_pos.setValue(v / 10))
        self.e_pos.valueChanged.connect(lambda v: self.s_pos.setValue(int(round(v * 10))))
        rang.addWidget(self.s_pos, 1)
        rang.addWidget(self.e_pos)
        l.addLayout(rang)
        rang = QHBoxLayout()
        for v in (0, 75, 150, 225, 300):
            b = QPushButton(f"{v}°")
            b.setObjectName("puce")
            b.clicked.connect(lambda _=False, v=v: (self.e_pos.setValue(v), self.envoyer_position()))
            rang.addWidget(b)
        l.addLayout(rang)
        self.b_pos = QPushButton("Envoyer la position")
        self.b_pos.setObjectName("principal")
        self.b_pos.clicked.connect(self.envoyer_position)
        l.addWidget(self.b_pos)
        gauche.addWidget(self.carte_pos)

        c, l = carte("Vitesse")
        rang = QHBoxLayout()
        self.s_vit = QSlider(Qt.Horizontal)
        self.e_vit = QSpinBox()
        self.e_vit.setFixedWidth(92)
        self.s_vit.valueChanged.connect(self.e_vit.setValue)
        self.e_vit.valueChanged.connect(self.s_vit.setValue)
        self.e_vit.valueChanged.connect(self._maj_aide_vitesse)
        rang.addWidget(self.s_vit, 1)
        rang.addWidget(self.e_vit)
        l.addLayout(rang)
        self.aide_vit = QLabel("")
        self.aide_vit.setObjectName("aide")
        l.addWidget(self.aide_vit)
        rang = QHBoxLayout()
        self.b_vit = QPushButton("Envoyer la vitesse")
        self.b_vit.setObjectName("principal")
        self.b_vit.clicked.connect(self.envoyer_vitesse)
        self.b_stop = QPushButton("Stop")
        self.b_stop.setObjectName("danger")
        self.b_stop.clicked.connect(self.arreter_rotation)
        rang.addWidget(self.b_vit, 1)
        rang.addWidget(self.b_stop)
        l.addLayout(rang)
        gauche.addWidget(c)
        gauche.addStretch()
        for i in range(gauche.count()):
            w = gauche.itemAt(i).widget()
            if w:
                w.setFixedWidth(320)

        # Colonne centrale : modèle 3D
        c, l = carte()
        entete = QHBoxLayout()
        t = QLabel("Modèle 3D du servomoteur")
        t.setObjectName("titreCarte")
        entete.addWidget(t)
        entete.addStretch()
        b = QPushButton("Vue de face")
        b.setObjectName("petit")
        b.clicked.connect(lambda: self.modele.vue_de_face())
        entete.addWidget(b)
        b = QPushButton("Vue 3D")
        b.setObjectName("petit")
        b.clicked.connect(lambda: self.modele.vue_initiale())
        entete.addWidget(b)
        l.addLayout(entete)
        self.modele = Modele3D()
        self.modele.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.modele.mpl_connect("motion_notify_event", lambda e: self.modele._ordre_affichage())
        l.addWidget(self.modele, 1)
        rang = QHBoxLayout()
        self.etiq_mode = QLabel()
        self.etiq_couple = QLabel()
        self.etiq_cible = QLabel()
        for e in (self.etiq_mode, self.etiq_couple, self.etiq_cible):
            e.setObjectName("badge")
            rang.addWidget(e)
        rang.addStretch()
        astuce = QLabel("Clic-glisser pour tourner la vue")
        astuce.setObjectName("aide")
        rang.addWidget(astuce)
        l.addLayout(rang)
        corps.addWidget(c, 1)

        # Colonne droite : données + terminal
        droite = QVBoxLayout()
        droite.setSpacing(12)
        corps.addLayout(droite, 0)

        c, l = carte("Données du servomoteur")
        grille = QGridLayout()
        grille.setSpacing(8)
        self.tuiles = {
            "position": TuileDonnee("Position", "°", "position", self),
            "vitesse": TuileDonnee("Vitesse", "", "vitesse", self),
            "temperature": TuileDonnee("Température", "°C", "temperature", self),
            "tension": TuileDonnee("Tension", "V", "tension", self),
            "charge": TuileDonnee("Charge", "", "charge", self),
        }
        for i, cle in enumerate(self.tuiles):
            grille.addWidget(self.tuiles[cle], i // 2, i % 2)
        self.tuiles["position"].auto.setChecked(True)
        l.addLayout(grille)
        rang = QHBoxLayout()
        b = QPushButton("Tout lire")
        b.setObjectName("secondaire")
        b.clicked.connect(self.tout_lire)
        rang.addWidget(b)
        self.c_auto = QCheckBox("Lecture auto toutes les")
        self.c_auto.toggled.connect(self._basculer_lecture_auto)
        rang.addWidget(self.c_auto)
        self.e_periode = QSpinBox()
        self.e_periode.setRange(100, 5000)
        self.e_periode.setSingleStep(100)
        self.e_periode.setValue(300)
        self.e_periode.setSuffix(" ms")
        self.e_periode.valueChanged.connect(lambda v: self.lecture_auto.setInterval(v))
        rang.addWidget(self.e_periode)
        rang.addStretch()
        l.addLayout(rang)
        droite.addWidget(c)

        c, l = carte()
        entete = QHBoxLayout()
        t = QLabel("Terminal")
        t.setObjectName("titreCarte")
        entete.addWidget(t)
        entete.addStretch()
        b = QPushButton("Effacer")
        b.setObjectName("petit")
        entete.addWidget(b)
        l.addLayout(entete)
        self.terminal = QPlainTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setObjectName("terminal")
        self.terminal.setMaximumBlockCount(2000)
        b.clicked.connect(self.terminal.clear)
        l.addWidget(self.terminal, 1)
        rang = QHBoxLayout()
        self.saisie = QLineEdit()
        self.saisie.setPlaceholderText("Commande libre (ex. p200, v300, POSITION)…")
        self.saisie.setObjectName("saisie")
        self.saisie.returnPressed.connect(self._envoyer_saisie)
        rang.addWidget(self.saisie, 1)
        b = QPushButton("Envoyer")
        b.setObjectName("secondaire")
        b.clicked.connect(self._envoyer_saisie)
        rang.addWidget(b)
        l.addLayout(rang)
        droite.addWidget(c, 1)
        for i in range(droite.count()):
            w = droite.itemAt(i).widget()
            if w:
                w.setFixedWidth(420)

        self._maj_badges()

    # ---------------- terminal ----------------
    def journal(self, genre, texte):
        couleurs = {"tx": "#7CC4FF", "rx": "#8EE3A8", "info": "#9AA5B1", "erreur": "#FF8A8A", "sim": "#F5C27A"}
        prefixes = {"tx": "→", "rx": "←", "info": "•", "erreur": "!", "sim": "→"}
        curseur = self.terminal.textCursor()
        curseur.movePosition(QTextCursor.End)
        f_heure = QTextCharFormat()
        f_heure.setForeground(QColor("#5C6773"))
        curseur.insertText(time.strftime("%H:%M:%S  "), f_heure)
        f = QTextCharFormat()
        f.setForeground(QColor(couleurs[genre]))
        suffixe = "   (simulé, non connecté)" if genre == "sim" else ""
        curseur.insertText(f"{prefixes[genre]} {texte}{suffixe}\n", f)
        self.terminal.setTextCursor(curseur)
        self.terminal.ensureCursorVisible()

    # ---------------- connexion ----------------
    def _actualiser_ports(self):
        actuel = self.ports.currentData()
        self.ports.clear()
        for p in serial.tools.list_ports.comports():
            nom = p.description if p.description and p.description != "n/a" else ""
            self.ports.addItem(f"{p.device}  {nom}".strip(), p.device)
        if self.ports.count() == 0:
            self.ports.addItem("Aucun port détecté", None)
        i = self.ports.findData(actuel)
        if i >= 0:
            self.ports.setCurrentIndex(i)
        else:  # proposer en priorité une carte Arduino
            for i in range(self.ports.count()):
                if "arduino" in self.ports.itemText(i).lower() or "mega" in self.ports.itemText(i).lower():
                    self.ports.setCurrentIndex(i)
                    break

    def _basculer_connexion(self):
        if self.connecte or self.liaison:
            self._deconnecter()
            return
        port = self.ports.currentData()
        if not port:
            self.journal("erreur", "Aucun port sélectionné.")
            return
        self.journal("info", f"Ouverture de {port} à {BAUDRATE} bauds…")
        self.v_arduino.etat("attente")
        self.b_connexion.setEnabled(False)
        self.liaison = LiaisonSerie(port)
        self.liaison.ligne_recue.connect(self._ligne_recue)
        self.liaison.erreur.connect(self._erreur_liaison)
        self.liaison.ouverte.connect(self._liaison_ouverte)
        self.liaison.start()

    def _liaison_ouverte(self):
        self.connecte = True
        self.b_connexion.setEnabled(True)
        self.b_connexion.setText("Déconnecter")
        self.b_connexion.setObjectName("danger")
        self.b_connexion.setStyle(self.b_connexion.style())
        self.v_arduino.etat("oui")
        self.journal("info", "Port ouvert. Redémarrage de la carte Arduino…")
        # la carte redémarre à l'ouverture du port : on attend avant le Ping
        QTimer.singleShot(DELAI_RESET_ARDUINO, self.ping)
        QTimer.singleShot(DELAI_RESET_ARDUINO + 300, self._synchroniser)

    def _synchroniser(self):
        """Met le servomoteur dans le même état que l'interface : mode servo, vitesse initiale."""
        if not self.connecte:
            return
        self.b_servo.setChecked(True)
        self._appliquer_mode(False)

    def _erreur_liaison(self, msg):
        self.journal("erreur", msg)
        self._deconnecter(silencieux=True)

    def _deconnecter(self, silencieux=False):
        self.c_auto.setChecked(False)
        if self.liaison:
            self.liaison.arreter()
            self.liaison = None
        self.connecte = False
        self.b_connexion.setEnabled(True)
        self.b_connexion.setText("Connecter")
        self.b_connexion.setObjectName("principal")
        self.b_connexion.setStyle(self.b_connexion.style())
        self.v_arduino.etat("inconnu")
        self.v_servo.etat("inconnu")
        if not silencieux:
            self.journal("info", "Déconnecté.")

    def envoyer(self, commande):
        if self.connecte and self.liaison and self.liaison.envoyer(commande):
            self.journal("tx", commande)
            return True
        self.journal("sim", commande)
        return False

    # ---------------- réception ----------------
    def _ligne_recue(self, ligne):
        self.journal("rx", ligne)
        m = REP_PING.match(ligne)
        if m:
            self.minuteur_ping.stop()
            present = m.group(1) == "1"
            self.v_servo.etat("oui" if present else "non")
            self.journal("info", "Servomoteur détecté." if present else "Aucun servomoteur ne répond.")
            return
        if "dynamixel detected" in ligne.lower():
            self.v_servo.etat("oui")
            return
        if ligne.lower().startswith("error"):
            self.v_servo.etat("non")
            return
        for cle, motif in REPONSES.items():
            m = motif.match(ligne)
            if m:
                self._afficher_donnee(cle, float(m.group(1).replace(",", ".")))
                return

    def _afficher_donnee(self, cle, v):
        t = self.tuiles[cle]
        if cle == "position":
            t.afficher(f"{v:.1f}")
            self.angle = v                                   # la mesure corrige le modèle
            if not self.couple:
                self.cible = v
        elif cle == "vitesse":
            if self.mode_continu:
                t.afficher(f"{v:.0f}", f"≈ {abs(v) * 0.1:.0f} % · {'anti-horaire' if v >= 0 else 'horaire'}")
                self.vitesse_mesuree = (v, time.perf_counter())
            else:
                t.afficher(f"{v:.0f}", f"≈ {abs(v) * UNITE_VITESSE_SERVO:.1f} tr/min")
        elif cle == "temperature":
            t.afficher(f"{v:.0f}", "limite de fonctionnement : 70 °C" if v < 70 else "⚠ trop chaud")
        elif cle == "tension":
            t.afficher(f"{v:.1f}", "plage conseillée : 9 à 12 V")
        elif cle == "charge":
            t.afficher(f"{v:.0f}", f"≈ {abs(v) * 0.1:.0f} % du couple max")

    # ---------------- Ping ----------------
    def ping(self):
        self.v_servo.etat("attente")
        if self.envoyer(CMD_PING):
            self.minuteur_ping.start(1500)
        else:
            QTimer.singleShot(400, lambda: self.v_servo.etat("oui"))   # simulation

    def _ping_sans_reponse(self):
        self.v_servo.etat("non")
        self.journal("erreur", "Pas de réponse au Ping (vérifiez que le programme Arduino gère « Ping »).")

    # ---------------- commandes ----------------
    def commande_couple(self, actif):
        self.couple = actif
        self.envoyer(CMD_ENABLE if actif else CMD_DISABLE)
        if not actif:
            self.cible = self.angle
        self._maj_badges()

    def _appliquer_mode(self, continu, envoyer=True):
        self.mode_continu = continu
        if envoyer:
            self.envoyer(CMD_WHEEL if continu else CMD_JOINT)
        # le programme Arduino remet la vitesse à 0 avant le changement de mode
        self.vitesse = 0
        self.vitesse_mesuree = None
        self.carte_pos.setEnabled(not continu)
        self.b_stop.setVisible(continu)
        self.s_vit.blockSignals(True)
        if continu:
            self.s_vit.setRange(-1023, 1023)
            self.e_vit.setRange(-1023, 1023)
            self.aide_mode.setText("Rotation sans fin. La consigne de vitesse fixe le sens et la vitesse.")
        else:
            self.s_vit.setRange(1, 1023)
            self.e_vit.setRange(1, 1023)
            self.aide_mode.setText("Rotation entre 0° et 300°, jusqu'à la position cible.")
            self.cible = self.angle % 360 if self.angle % 360 <= 300 else 300.0
            self.angle = self.cible
        self.s_vit.blockSignals(False)
        self.e_vit.setValue(0 if continu else VITESSE_INITIALE)
        self._maj_aide_vitesse(self.e_vit.value())
        if not continu:
            # Le programme Arduino laisse la vitesse à 1 (minimum) après JOINT :
            # on renvoie une vitesse utilisable.
            if envoyer:
                self.envoyer_vitesse()
            else:
                self.vitesse = VITESSE_INITIALE
        self._maj_badges()

    def _maj_aide_vitesse(self, v):
        if self.mode_continu:
            if v == 0:
                txt = "0 = arrêt"
            else:
                txt = f"≈ {abs(v) * 0.1:.0f} % de la vitesse max, sens {'anti-horaire' if v > 0 else 'horaire'}"
        else:
            txt = f"≈ {min(max(v, 1) * UNITE_VITESSE_SERVO, VITESSE_MAX_TR_MIN):.1f} tr/min"
        self.aide_vit.setText(txt)

    def envoyer_position(self):
        a = round(self.e_pos.value(), 1)
        self.cible = a
        self.envoyer(f"p{a:g}")
        self._maj_badges()

    def envoyer_vitesse(self):
        v = self.e_vit.value()
        self.vitesse = v
        self.vitesse_mesuree = None
        self.envoyer(f"v{v}")
        self._maj_badges()

    def arreter_rotation(self):
        self.e_vit.setValue(0)
        self.envoyer_vitesse()

    def demander(self, cle):
        self.envoyer(CMD_LECTURE[cle])
        if not self.connecte:                      # valeurs simulées
            sim = {"position": self.angle % 360, "vitesse": self.vitesse if self.mode_continu else 0,
                   "temperature": 38, "tension": 11.4, "charge": 0}
            QTimer.singleShot(120, lambda: self._afficher_donnee(cle, sim[cle]))

    def tout_lire(self):
        for i, cle in enumerate(self.tuiles):
            QTimer.singleShot(i * 120, lambda cle=cle: self.demander(cle))

    def _basculer_lecture_auto(self, actif):
        if actif:
            self.lecture_auto.start(self.e_periode.value())
        else:
            self.lecture_auto.stop()

    def _tick_lecture_auto(self):
        if not self._file_auto:
            self._file_auto = [c for c, t in self.tuiles.items() if t.auto.isChecked()]
            if not self._file_auto:
                return
        self.demander(self._file_auto.pop(0))

    def _envoyer_saisie(self):
        cmd = self.saisie.text().strip()
        if not cmd:
            return
        self.saisie.clear()
        # on applique aussi l'effet sur le modèle pour garder l'interface cohérente
        u = cmd.upper()
        if u in (CMD_ENABLE, CMD_DISABLE):
            (self.b_couple_on if u == CMD_ENABLE else self.b_couple_off).setChecked(True)
            self.commande_couple(u == CMD_ENABLE)
        elif u in (CMD_JOINT, CMD_WHEEL):
            (self.b_continu if u == CMD_WHEEL else self.b_servo).setChecked(True)
            self._appliquer_mode(u == CMD_WHEEL)
        elif u == CMD_PING.upper():
            self.ping()
        elif re.fullmatch(r"P-?\d+(\.\d+)?", u) and not self.mode_continu:
            self.e_pos.setValue(float(cmd[1:]))
            self.envoyer_position()
        elif re.fullmatch(r"V-?\d+", u):
            self.e_vit.setValue(int(cmd[1:]))
            self.envoyer_vitesse()
        else:
            self.envoyer(cmd)

    # ---------------- animation du modèle ----------------
    def _animer(self):
        maintenant = time.perf_counter()
        dt = min(maintenant - self._t, 0.2)
        self._t = maintenant
        avant = self.angle
        if self.couple:
            if self.mode_continu:
                v = self.vitesse
                if self.vitesse_mesuree and maintenant - self.vitesse_mesuree[1] < 1.5:
                    v = self.vitesse_mesuree[0]
                v = max(-1023, min(1023, v))
                self.angle += (v / 1023.0) * VITESSE_MAX_TR_MIN * 6.0 * dt
            else:
                # la librairie remplace la vitesse 0 par 1 en mode servo
                tr_min = min(max(self.vitesse, 1) * UNITE_VITESSE_SERVO, VITESSE_MAX_TR_MIN)
                pas = tr_min * 6.0 * dt
                ecart = self.cible - self.angle
                self.angle = self.cible if abs(ecart) <= pas else self.angle + math.copysign(pas, ecart)
        if abs(self.angle - avant) > 1e-3 or not hasattr(self, "_dessine"):
            self._dessine = True
            if self.mode_continu:
                info = f"Mode continu · consigne {self.vitesse}"
            else:
                info = f"Mode servo · cible {self.cible:.1f}°"
            self.modele.mettre_a_jour(self.angle, info)

    def _maj_badges(self):
        self.etiq_mode.setText("Mode continu" if self.mode_continu else "Mode servo")
        self.etiq_couple.setText("Couple actif" if self.couple else "Couple désactivé")
        self.etiq_couple.setProperty("etat", "ok" if self.couple else "off")
        self.etiq_couple.setStyle(self.etiq_couple.style())
        if self.mode_continu:
            self.etiq_cible.setText(f"Vitesse : {self.vitesse}")
        else:
            self.etiq_cible.setText(f"Cible : {self.cible:.1f}°")
        self._dessine_force()

    def _dessine_force(self):
        if hasattr(self, "modele"):
            info = (f"Mode continu · consigne {self.vitesse}" if self.mode_continu
                    else f"Mode servo · cible {self.cible:.1f}°")
            self.modele.mettre_a_jour(self.angle, info)

    def closeEvent(self, e):
        self._deconnecter(silencieux=True)
        super().closeEvent(e)


# =====================================================================
#  Style
# =====================================================================
STYLE = f"""
QWidget {{ font-family: 'Segoe UI', 'Inter', 'Helvetica Neue', Arial; font-size: 10pt; color: {TEXTE}; }}
QMainWindow, QWidget#centre {{ background: {FOND}; }}
QFrame#barre {{ background: {BLEU}; border-radius: 12px; }}
QFrame#barre QLabel {{ color: #DCE6EF; }}
QLabel#titre {{ color: white; font-size: 15pt; font-weight: 700; }}
QLabel#sousTitre {{ color: #9FB6CA; font-size: 9pt; }}
QFrame#sep {{ color: #3D6585; }}
QFrame#carte {{ background: {CARTE}; border: 1px solid {BORD}; border-radius: 12px; }}
QLabel#titreCarte {{ font-size: 11pt; font-weight: 700; color: {BLEU}; }}
QLabel#aide {{ color: {GRIS}; font-size: 9pt; }}
QLabel#badge {{ background: #EEF2F6; color: {BLEU}; border-radius: 10px; padding: 3px 10px; font-size: 9pt; font-weight: 600; }}
QLabel#badge[etat="off"] {{ background: #FBEAEA; color: {ROUGE}; }}
QLabel#badge[etat="ok"] {{ background: #E7F6EC; color: #1E7D43; }}

QPushButton {{ border-radius: 8px; padding: 7px 14px; font-weight: 600; border: 1px solid {BORD}; background: #F6F8FA; }}
QPushButton:hover {{ background: #EDF1F5; }}
QPushButton:disabled {{ color: #A9B2BC; }}
QPushButton#principal {{ background: {ORANGE}; color: white; border: none; }}
QPushButton#principal:hover {{ background: #CF6D17; }}
QPushButton#principal:disabled {{ background: #F0C39B; }}
QPushButton#danger {{ background: {ROUGE}; color: white; border: none; }}
QPushButton#danger:hover {{ background: #B93636; }}
QPushButton#secondaire {{ background: white; color: {BLEU}; border: 1px solid #B9C7D4; }}
QPushButton#secondaire:hover {{ background: #EEF3F8; }}
QFrame#barre QPushButton#secondaire {{ background: transparent; color: white; border: 1px solid #5F86A6; }}
QFrame#barre QPushButton#secondaire:hover {{ background: #164C72; }}
QPushButton#icone {{ background: transparent; color: white; border: 1px solid #5F86A6; padding: 6px 10px; }}
QPushButton#petit {{ padding: 3px 10px; font-size: 9pt; background: white; color: {BLEU}; }}
QPushButton#puce {{ padding: 4px 0px; font-size: 9pt; background: #F3F6F9; }}
QPushButton#segment {{ padding: 8px; background: #F3F6F9; color: {GRIS}; }}
QPushButton#segment:checked {{ background: {BLEU}; color: white; border: 1px solid {BLEU}; }}

QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit#saisie {{ border: 1px solid #C9D3DD; border-radius: 7px; padding: 5px 8px; background: white; }}
QFrame#barre QComboBox {{ color: {TEXTE}; }}
QSlider::groove:horizontal {{ height: 6px; background: #E3E8EE; border-radius: 3px; }}
QSlider::sub-page:horizontal {{ background: {ORANGE}; border-radius: 3px; }}
QSlider::handle:horizontal {{ background: white; border: 2px solid {ORANGE}; width: 14px; margin: -6px 0; border-radius: 8px; }}

QFrame#tuile {{ background: #F7F9FB; border: 1px solid {BORD}; border-radius: 10px; }}
QLabel#nomTuile {{ color: {GRIS}; font-size: 9pt; font-weight: 600; }}
QLabel#valeurTuile {{ font-size: 18pt; font-weight: 700; color: {BLEU}; }}
QLabel#uniteTuile {{ color: {GRIS}; font-size: 10pt; padding-bottom: 4px; }}
QLabel#detailTuile {{ color: {GRIS}; font-size: 8pt; }}
QCheckBox {{ color: {GRIS}; font-size: 9pt; }}

QPlainTextEdit#terminal {{ background: #111A24; color: #D8DEE5; border-radius: 8px; padding: 8px;
    font-family: 'Cascadia Mono', 'Consolas', 'DejaVu Sans Mono', monospace; font-size: 9.5pt; border: none; }}
"""


def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    f = Fenetre()
    f.centralWidget().setObjectName("centre")
    f.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()