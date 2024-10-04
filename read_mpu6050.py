import math
import time
from mpu6050 import mpu6050

class KalmanFilter:
    def __init__(self, Q_angle=0.001, Q_bias=0.003, R_measure=0.03):
        # Prozessvariablen
        self.Q_angle = Q_angle  # Prozessrauschen für den Winkel
        self.Q_bias = Q_bias    # Prozessrauschen für den Bias
        self.R_measure = R_measure  # Messrauschen für den Winkel

        # Kalman-Variablen
        self.angle = 0.0  # Winkel-Schätzung
        self.bias = 0.0   # Bias-Schätzung
        self.P = [[0, 0], [0, 0]]  # Fehlerkovarianzmatrix

    def update(self, new_angle, new_rate, dt):
        """ Aktualisiert die Winkel- und Bias-Schätzung basierend auf neuen Messungen. """
        # Prädiktion
        rate = new_rate - self.bias
        self.angle += dt * rate

        # Fehlerkovarianz-Prädiktion
        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        # Innovation
        S = self.P[0][0] + self.R_measure  # Innovationskovarianz
        K = [self.P[0][0] / S, self.P[1][0] / S]  # Kalman-Gewinne

        y = new_angle - self.angle  # Winkel-Differenz
        self.angle += K[0] * y
        self.bias += K[1] * y

        # Fehlerkovarianz-Update
        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return self.angle

class MPU6050SensorKalman:
    def __init__(self, address=0x68, calibration_samples=100, delta_threshold=2, stabilization_count=4):
        self.sensor = mpu6050(address)
        self.null_roll = 0
        self.null_pitch = 0
        self.last_time = time.time()

        # Letzte ausgegebene Werte
        self.last_output_roll = 0
        self.last_output_pitch = 0

        # Kalman-Filter für Roll und Pitch
        self.kalman_roll = KalmanFilter()
        self.kalman_pitch = KalmanFilter()

        # Dynamische Gyroskop-Bias-Werte
        self.gyro_bias_x = 0
        self.gyro_bias_y = 0

        # Schwellenwert für das Delta
        self.delta_threshold = delta_threshold
        self.stabilization_count = stabilization_count  # Anzahl der Ignorierten Werte
        self.current_count = 0  # Zählt, wie viele Werte bereits verarbeitet wurden

        # Kalibrierung
        self.calibrate_gyro(samples=calibration_samples)
        self.calibrate_position()

    def calibrate_gyro(self, samples):
        """Kalibriert den Gyroskop-Bias durch Mittelung mehrerer Messungen."""
        gyro_x_total = gyro_y_total = 0
        for _ in range(samples):
            gyro_data = self.sensor.get_gyro_data()
            gyro_x_total += gyro_data['x']
            gyro_y_total += gyro_data['y']
            time.sleep(0.01)  # Kleine Pause zwischen den Messungen
        self.gyro_bias_x = gyro_x_total / samples
        self.gyro_bias_y = gyro_y_total / samples
        print(f"Gyroskop-Kalibrierung abgeschlossen: Bias X = {self.gyro_bias_x:.4f}, Bias Y = {self.gyro_bias_y:.4f}")

    def calibrate_position(self):
        """Kalibriert den Roll- und Pitch-Wert, um den Nullpunkt zu setzen."""
        roll, pitch, _, _ = self.calculate_pitch_roll()
        self.null_roll = roll
        self.null_pitch = pitch
        print(f"Initiale Kalibrierung abgeschlossen: Null-Roll = {self.null_roll:.2f}°, Null-Pitch = {self.null_pitch:.2f}°")

    def calculate_pitch_roll(self):
        """Berechnet die aktuellen Roll- und Pitch-Werte unter Verwendung des Kalman-Filters."""
        accel_data = self.sensor.get_accel_data()
        roll_accel = math.atan2(accel_data['y'], math.sqrt(accel_data['x']**2 + accel_data['z']**2)) * 180 / math.pi
        pitch_accel = math.atan2(-accel_data['x'], math.sqrt(accel_data['y']**2 + accel_data['z']**2)) * 180 / math.pi

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        gyro_data = self.sensor.get_gyro_data()

        # Gyroskopdaten mit Bias-Korrektur
        roll_gyro = gyro_data['x'] - self.gyro_bias_x
        pitch_gyro = gyro_data['y'] - self.gyro_bias_y

        # Kalman-Filter Update
        roll = self.kalman_roll.update(roll_accel, roll_gyro, dt)
        pitch = self.kalman_pitch.update(pitch_accel, pitch_gyro, dt)

        # Berechne die Deltas relativ zu den zuletzt ausgegebenen Werten
        delta_roll = abs(roll - self.last_output_roll)
        delta_pitch = abs(pitch - self.last_output_pitch)

        # Nur wenn das Delta >= delta_threshold ist, werden die neuen Werte ausgegeben und gespeichert
        if delta_roll >= self.delta_threshold:
            self.last_output_roll = roll
        else:
            roll = self.last_output_roll

        if delta_pitch >= self.delta_threshold:
            self.last_output_pitch = pitch
        else:
            pitch = self.last_output_pitch

        return roll, pitch, accel_data, gyro_data

    def get_absolute_angles(self):
        """Berechnet die Delta-Werte relativ zu den initialen Kalibrierungswerten."""
        roll, pitch, _, _ = self.calculate_pitch_roll()

        # Zähle die Messungen hoch und ignoriere die Ausgabe für die ersten stabilisierenden Messungen
        self.current_count += 1

        if self.current_count <= self.stabilization_count:
            print(f"Stabilisiere... ({self.current_count}/{self.stabilization_count})")
            return None, None  # Noch keine Ausgabe, Werte stabilisieren sich

        # Werte nach Stabilisierung ausgeben
        #return roll - self.null_roll, pitch - self.null_pitch
        return  pitch - self.null_pitch, roll - self.null_roll  # Sensor is build in 90° wrong

if __name__ == "__main__":
    sensor = MPU6050SensorKalman()
    try:
        while True:
            # Berechne das Delta zu den initialen Kalibrierungswerten
            roll_abs, pitch_abs = sensor.get_absolute_angles()

            # Gib erst nach der Stabilisierungsphase aus
            if roll_abs is not None and pitch_abs is not None:
                print(f"Delta Roll: {roll_abs:.2f}°, Delta Pitch: {pitch_abs:.2f}°")
            time.sleep(1)
    except KeyboardInterrupt:
        print("Programm beendet.")

