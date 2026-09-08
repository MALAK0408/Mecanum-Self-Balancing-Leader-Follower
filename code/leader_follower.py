from ugot import ugot
import cv2
import numpy as np
import time


# ==============================
# CONFIGURATION
# ==============================

LEADER_IP = "YOUR_LEADER_IP"      # Mecanum robot (porte le tag)
FOLLOWER_IP = "YOUR_FOLLOWER_IP"    # Self-balancing robot (suit le tag)

TARGET_TAG_ID = 2

# Leader speeds
FORWARD_SPEED = 15          # vitesse d'avance normale du leader
SLOW_FORWARD_SPEED = 6      # vitesse lente du leader juste apres le virage
PRE_TURN_SLOW_SPEED = 6     # vitesse de ralentissement AVANT le virage (solution 11)
TURN_SPEED_LEADER = 40      # vitesse de virage du leader
LEADER_BACKUP_SPEED = -8    # vitesse de recul lent du leader (solution 2)

FORWARD_TIME = 4            # duree d'avance normale du leader (s)
SLOW_FORWARD_TIME = 3       # duree d'avance lente apres le virage (s)
PRE_TURN_SLOW_TIME = 1.5    # duree de ralentissement avant le virage (s)
TURN_TIME_LEADER = 1.5      # duree du virage pour faire ~90 degres
STOP_TIME = 0.3             # petite pause entre phases du leader

# Follower search speeds & timings
TURN_SPEED_FOLLOWER = 35    # vitesse de recherche du follower
PRIMARY_SEARCH_TIME = 5     # duree de recherche dans la direction "probable" (s)
SECONDARY_SEARCH_TIME = 10  # duree de recherche dans la direction opposee (s)
SEARCH_FORWARD_TIME = 2     # duree d'avance quand rien trouve apres gauche+droite (s)
SEARCH_FORWARD_SPEED = 10   # vitesse de cette avance de recherche

SEARCH_FORWARD_BIT_TIME = 1.5   # petite avance juste apres avoir perdu le tag
SEARCH_FORWARD_BIT_SPEED = 10   # vitesse de cette petite avance

LOST_GRACE_TIME = 0.6       # "freeze" : tolerance (s) avant de considerer le tag VRAIMENT perdu (solution 15)
BACKUP_SEARCH_TIMEOUT = 15  # (s) si le follower cherche depuis plus longtemps, le leader recule (solution 2/9)

# Direction codes utilises par follower_search()
DIR_LEFT = 2
DIR_RIGHT = 3

# Centre de l'image (a ajuster selon la resolution reelle de la camera du follower)
FRAME_CENTER_X = 320

# -------------------------------------------------------------
# 15 CM DISTANCE CONTROL CONFIGURATION
# -------------------------------------------------------------
TARGET_TAG_SIZE_15CM = 80.0

KP_DISTANCE = 0.45       # Gain factor for speed adjustment
MAX_FOLLOW_SPEED = 25    # Max speed moving forward
MIN_FOLLOW_SPEED = -20   # Max speed reversing (when closer than 15 cm)
DEADZONE_SIZE = 5        # Tolerance range around 15 cm to prevent oscillations


# ==============================
# INITIALIZATION - LEADER
# ==============================

leader = ugot.UGOT()
print("LEADER connect:", leader.initialize(LEADER_IP))
leader.open_camera()
time.sleep(1)
print("LEADER READY")


# ==============================
# INITIALIZATION - FOLLOWER
# ==============================

follower = ugot.UGOT()
print("FOLLOWER connect:", follower.initialize(FOLLOWER_IP))

follower.balance_start_balancing()
time.sleep(2)

follower.open_camera()
time.sleep(2)

follower.load_models(['apriltag_qrcode'])
time.sleep(3)

print("FOLLOWER READY - waiting for tag")


# ==============================
# FUNCTIONS
# ==============================

def show_camera(bot, window_name):
    frame = bot.read_camera_data()
    if frame is not None:
        img = cv2.imdecode(
            np.frombuffer(frame, np.uint8),
            cv2.IMREAD_COLOR
        )
        if img is not None:
            cv2.imshow(window_name, img)


def detect_tag_info():
    """Returns (True, tag_data) if target tag is detected, else (False, None)."""
    tags = follower.get_apriltag_total_info()
    if tags:
        for tag in tags:
            if tag[0] == TARGET_TAG_ID:
                return True, tag
    return False, None


def get_tag_center_x(tag_info):
    """Recupere la position horizontale du tag (a verifier/ajuster selon le format reel)."""
    try:
        return tag_info[1]
    except (IndexError, TypeError):
        return FRAME_CENTER_X


# ---- leader actions ----

def leader_forward():
    leader.mecanum_move_speed(0, FORWARD_SPEED)


def leader_slow_forward():
    leader.mecanum_move_speed(0, SLOW_FORWARD_SPEED)


def leader_pre_turn_slow():
    leader.mecanum_move_speed(0, PRE_TURN_SLOW_SPEED)


def leader_turn():
    leader.mecanum_turn_speed(3, TURN_SPEED_LEADER)  # 2 = gauche


def leader_backup():
    leader.mecanum_move_speed(0, LEADER_BACKUP_SPEED)


def leader_stop():
    leader.mecanum_stop()


# ---- follower actions ----

def follower_follow_distance(tag_info):
    """
    Maintains ~15 cm distance using proportional control (P-loop).
    Moves forward when >15 cm, reverses when <15 cm.
    """
    current_size = tag_info[3]
    error = TARGET_TAG_SIZE_15CM - current_size

    if abs(error) <= DEADZONE_SIZE:
        follower.balance_move_speed(0, 0)
        return

    speed = int(error * KP_DISTANCE)
    speed = max(MIN_FOLLOW_SPEED, min(MAX_FOLLOW_SPEED, speed))

    follower.balance_move_speed(0, speed)


def follower_search(direction):
    follower.balance_turn_speed(direction, TURN_SPEED_FOLLOWER)


def follower_search_forward():
    follower.balance_move_speed(0, SEARCH_FORWARD_SPEED)


def follower_search_forward_bit():
    follower.balance_move_speed(0, SEARCH_FORWARD_BIT_SPEED)


def follower_stop():
    follower.balance_move_speed(0, 0)


# ==============================
# STATE MACHINES
# ==============================

# --- LEADER ---
# Etats normaux: FORWARD -> PRE_TURN_SLOW -> STOP1 -> TURN -> WAIT_FOLLOWER -> SLOW_FORWARD -> FORWARD
# Etats speciaux pilotes par le follower: HOLD (arret immediat) et BACKUP (recul lent)
leader_state = "FORWARD"
leader_state_start = time.time()

# Sauvegarde pour pouvoir reprendre exactement ou on en etait apres un HOLD/BACKUP
leader_saved_state = None
leader_saved_elapsed = 0.0
leader_in_hold_or_backup = False

# --- FOLLOWER ---
follower_state = "WAIT_FIRST_TAG"
search_phase = "FORWARD_BIT"      # FORWARD_BIT -> PRIMARY -> SECONDARY -> FORWARD_BIT ...
search_state_start = time.time()
search_total_start = time.time()  # marque le debut de TOUTE la recherche (pour le timeout -> recul du leader)
last_seen_time = time.time()
last_seen_center_x = FRAME_CENTER_X

# Direction "probable" ou le tag a disparu (calculee au moment de la perte)
primary_direction = DIR_LEFT
secondary_direction = DIR_RIGHT


try:
    while True:
        now = time.time()

        # ---------------------------------
        # FOLLOWER: detection du tag (fait en premier, le leader en a besoin)
        # ---------------------------------
        tag_found, tag_info = detect_tag_info()

        # ---------------------------------
        # LEADER STATE MACHINE
        # ---------------------------------

        # Solution 1 : des que le follower cherche le tag, le leader s'arrete immediatement.
        # Solution 2 : si la recherche dure trop longtemps, le leader recule lentement.
        follower_is_searching = (follower_state == "SEARCH")
        search_duration_total = (now - search_total_start) if follower_is_searching else 0

        should_backup = follower_is_searching and (search_duration_total > BACKUP_SEARCH_TIMEOUT)
        should_hold = follower_is_searching and not should_backup

        if should_hold:
            if not leader_in_hold_or_backup or leader_state != "HOLD":
                # on entre en HOLD : on memorise l'etat courant pour y revenir plus tard
                if leader_state not in ("HOLD", "BACKUP"):
                    leader_saved_state = leader_state
                    leader_saved_elapsed = now - leader_state_start
                leader_state = "HOLD"
                leader_in_hold_or_backup = True
            leader_stop()

        elif should_backup:
            if leader_state != "BACKUP":
                if leader_state not in ("HOLD", "BACKUP"):
                    leader_saved_state = leader_state
                    leader_saved_elapsed = now - leader_state_start
                leader_state = "BACKUP"
                leader_in_hold_or_backup = True
                print("LEADER: recherche trop longue, je recule lentement pour aider le follower")
            leader_backup()

        else:
            # Le follower ne cherche plus : on reprend le fonctionnement normal du leader
            if leader_in_hold_or_backup:
                leader_state = leader_saved_state if leader_saved_state else "FORWARD"
                leader_state_start = now - leader_saved_elapsed
                leader_in_hold_or_backup = False
                print("LEADER: le follower a retrouve le tag, je reprends ma trajectoire")

            elapsed = now - leader_state_start

            if leader_state == "FORWARD":
                leader_forward()
                if elapsed > FORWARD_TIME:
                    leader_state, leader_state_start = "PRE_TURN_SLOW", now

            elif leader_state == "PRE_TURN_SLOW":
                # Solution 11 : on ralentit avant de tourner pour ne pas perdre le tag
                leader_pre_turn_slow()
                if elapsed > PRE_TURN_SLOW_TIME:
                    leader_stop()
                    leader_state, leader_state_start = "STOP1", now

            elif leader_state == "STOP1":
                if elapsed > STOP_TIME:
                    leader_state, leader_state_start = "TURN", now

            elif leader_state == "TURN":
                leader_turn()
                if elapsed > TURN_TIME_LEADER:
                    leader_stop()
                    leader_state, leader_state_start = "WAIT_FOLLOWER", now
                    print("LEADER: turn finished, waiting for follower to re-find the tag")

            elif leader_state == "WAIT_FOLLOWER":
                leader_stop()
                if follower_state == "FOLLOWING":
                    print("LEADER: follower re-found the tag, moving on")
                    leader_state, leader_state_start = "SLOW_FORWARD", now
                else:
                    # pas de securite MAX_WAIT ici : le follower gere deja le timeout via BACKUP
                    pass

            elif leader_state == "SLOW_FORWARD":
                leader_slow_forward()
                if elapsed > SLOW_FORWARD_TIME:
                    leader_state, leader_state_start = "FORWARD", now

        show_camera(leader, "Leader Camera")

        # ---------------------------------
        # FOLLOWER STATE MACHINE
        # ---------------------------------

        if follower_state == "WAIT_FIRST_TAG":
            follower_stop()
            if tag_found:
                print("FIRST TAG FOUND")
                last_seen_center_x = get_tag_center_x(tag_info)
                follower_state = "FOLLOWING"

        elif follower_state == "FOLLOWING":
            if tag_found:
                last_seen_time = now
                last_seen_center_x = get_tag_center_x(tag_info)
                if leader_state in ("HOLD", "STOP1", "TURN", "PRE_TURN_SLOW"):
                    follower_stop()
                else:
                    follower_follow_distance(tag_info)

            elif now - last_seen_time > LOST_GRACE_TIME:
                # Solution 15 : au-dela de la tolerance "freeze", on considere le tag perdu
                print("TAG LOST")
                follower_stop()
                follower_state = "SEARCH"
                search_total_start = now
                search_state_start = now
                search_phase = "FORWARD_BIT"

                # Solution 4/12 : on cherche d'abord du cote ou le tag a disparu
                if last_seen_center_x < FRAME_CENTER_X:
                    primary_direction = DIR_LEFT
                    secondary_direction = DIR_RIGHT
                    print("SEARCH: tag disparu a gauche -> recherche prioritaire a gauche")
                else:
                    primary_direction = DIR_RIGHT
                    secondary_direction = DIR_LEFT
                    print("SEARCH: tag disparu a droite -> recherche prioritaire a droite")
            else:
                # periode de "freeze" : on ne bouge pas, le tag revient peut-etre tout seul
                follower_stop()

        elif follower_state == "SEARCH":
            if tag_found:
                print("TAG FOUND AGAIN")
                last_seen_time = now
                last_seen_center_x = get_tag_center_x(tag_info)
                follower_state = "FOLLOWING"
            else:
                elapsed_search = now - search_state_start

                if search_phase == "FORWARD_BIT":
                    follower_search_forward_bit()
                    if elapsed_search > SEARCH_FORWARD_BIT_TIME:
                        follower_stop()
                        search_phase, search_state_start = "PRIMARY", now
                        print("SEARCH - PRIMARY DIRECTION")

                elif search_phase == "PRIMARY":
                    follower_search(primary_direction)
                    if elapsed_search > PRIMARY_SEARCH_TIME:
                        search_phase, search_state_start = "SECONDARY", now
                        print("NOT FOUND - SEARCH SECONDARY DIRECTION")

                elif search_phase == "SECONDARY":
                    follower_search(secondary_direction)
                    if elapsed_search > SECONDARY_SEARCH_TIME:
                        follower_stop()
                        search_phase, search_state_start = "FORWARD_BIT", now
                        print("NOT FOUND - MOVE FORWARD A BIT AGAIN")

        show_camera(follower, "Follower Camera")

        # ---------------------------------
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if follower_state == "SEARCH":
            time.sleep(0.02)
        else:
            time.sleep(0.05)

except KeyboardInterrupt:
    print("STOPPED")

finally:
    leader_stop()
    follower_stop()
    cv2.destroyAllWindows()
