import math
import time
from mpu6050 import mpu6050

class KalmanFilter:
    def __init__(self, Q_angle=0.001, Q_bias=0.003, R_measure=0.03):
        # Prozessrauschen und Messrauschen
        self.Q_angle = Q_angle
        self.Q_bias = Q_bias
        self.R_measure = R_measure

        # Kalman-Filter-Variablen
        self.angle = 0.0
        self.bias = 0.0
        self.P = [[0.0, 0.0], [0.0, 0.0]]  # Fehlerkovarianzmatrix

    def update(self, new_angle, new_rate, dt):
        """Aktualisiert die Schätzungen für Winkel und Bias basierend auf neuen Messungen."""
        # Prädiktionsschritt
        rate = new_rate - self.bias
        self.angle += dt * rate

        # Fehlerkovarianzmatrix aktualisieren
        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        # Innovationsschritt
        S = self.P[0][0] + self.R_measure
        K = [self.P[0][0] / S, self.P[1][0] / S]
        y = new_angle - self.angle

        # Schätzungen aktualisieren
        self.angle += K[0] * y
        self.bias += K[1] * y

        # Fehlerkovarianzmatrix anpassen
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
        self.null_roll = 0.0
        self.null_pitch = 0.0
        self.last_time = time.perf_counter()

        # Letzte ausgegebene Werte
        self.last_output_roll = 0.0
        self.last_output_pitch = 0.0

        # Kalman-Filter für Roll und Pitch
        self.kalman_roll = KalmanFilter()
        self.kalman_pitch = KalmanFilter()

        # Gyroskop-Bias-Werte
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0

        # Schwellenwerte und Zähler
        self.delta_threshold = delta_threshold
        self.stabilization_count = stabilization_count
        self.current_count = 0

        # Kalibrierung
        self.calibrate_gyro(samples=calibration_samples)
        self.calibrate_position(samples=calibration_samples)

    def calibrate_gyro(self, samples):
        """Kalibriert das Gyroskop durch Mittelung mehrerer Messungen."""
        gyro_x_total = 0.0
        gyro_y_total = 0.0
        for _ in range(samples):
            try:
                gyro_data = self.sensor.get_gyro_data()
                gyro_x_total += gyro_data['x']
                gyro_y_total += gyro_data['y']
                time.sleep(0.01)
            except Exception as e:
                print(f"Fehler bei der Gyroskop-Kalibrierung: {e}")
        self.gyro_bias_x = gyro_x_total / samples
        self.gyro_bias_y = gyro_y_total / samples
        print(f"Gyroskop-Kalibrierung abgeschlossen: Bias X = {self.gyro_bias_x:.4f}, Bias Y = {self.gyro_bias_y:.4f}")

    def calibrate_position(self, samples=100):
        """Kalibriert die Ausgangsposition durch Mittelung mehrerer Messungen."""
        roll_total = 0.0
        pitch_total = 0.0
        valid_samples = 0
        for _ in range(samples):
            try:
                roll, pitch, _, _ = self.calculate_pitch_roll()
                roll_total += roll
                pitch_total += pitch
                valid_samples += 1
                time.sleep(0.01)
            except Exception as e:
                print(f"Fehler bei der Positionskalibrierung: {e}")
        if valid_samples > 0:
            self.null_roll = roll_total / valid_samples
            self.null_pitch = pitch_total / valid_samples
            print(f"Positionskalibrierung abgeschlossen: Null-Roll = {self.null_roll:.2f}°, Null-Pitch = {self.null_pitch:.2f}°")
        else:
            print("Keine gültigen Daten für die Positionskalibrierung erhalten.")

    def calculate_pitch_roll(self):
        """Berechnet die aktuellen Roll- und Pitch-Werte unter Verwendung des Kalman-Filters."""
        try:
            accel_data = self.sensor.get_accel_data()
            # Anpassung für die 90°-Fehlausrichtung des Sensors
            pitch_accel = math.atan2(accel_data['y'], math.sqrt(accel_data['x']**2 + accel_data['z']**2)) * (180 / math.pi)
            roll_accel = math.atan2(-accel_data['x'], math.sqrt(accel_data['y']**2 + accel_data['z']**2)) * (180 / math.pi)

            current_time = time.perf_counter()
            dt = current_time - self.last_time
            self.last_time = current_time

            gyro_data = self.sensor.get_gyro_data()
            pitch_gyro = gyro_data['y'] - self.gyro_bias_y
            roll_gyro = gyro_data['x'] - self.gyro_bias_x

            # Aktualisierung des Kalman-Filters
            pitch = self.kalman_pitch.update(pitch_accel, pitch_gyro, dt)
            roll = self.kalman_roll.update(roll_accel, roll_gyro, dt)

            # Delta-Berechnung
            delta_pitch = abs(pitch - self.last_output_pitch)
            delta_roll = abs(roll - self.last_output_roll)

            # Aktualisierung der Ausgabe bei Überschreiten des Schwellenwerts
            if delta_pitch >= self.delta_threshold:
                self.last_output_pitch = pitch
            else:
                pitch = self.last_output_pitch

            if delta_roll >= self.delta_threshold:
                self.last_output_roll = roll
            else:
                roll = self.last_output_roll

            return roll, pitch, accel_data, gyro_data
        except Exception as e:
            print(f"Fehler bei der Berechnung von Roll und Pitch: {e}")
            return self.last_output_roll, self.last_output_pitch, None, None

    def get_absolute_angles(self):
        """Gibt die absoluten Winkel relativ zur kalibrierten Ausgangsposition zurück."""
        roll, pitch, _, _ = self.calculate_pitch_roll()

        # Stabilisierung der ersten Messungen
        self.current_count += 1
        if self.current_count <= self.stabilization_count:
            print(f"Stabilisiere... ({self.current_count}/{self.stabilization_count})")
            return None, None

        # Korrektur für die Sensororientierung
        corrected_roll = pitch - self.null_pitch
        corrected_pitch = roll - self.null_roll

        return corrected_roll, corrected_pitch

if __name__ == "__main__":
    sensor = MPU6050SensorKalman()
    try:
        while True:
            roll_abs, pitch_abs = sensor.get_absolute_angles()
            if roll_abs is not None and pitch_abs is not None:
                print(f"Delta Roll: {roll_abs:.2f}°, Delta Pitch: {pitch_abs:.2f}°")
            time.sleep(0.1)  # Aktualisierungsrate erhöht
    except KeyboardInterrupt:
        print("Programm beendet.")
