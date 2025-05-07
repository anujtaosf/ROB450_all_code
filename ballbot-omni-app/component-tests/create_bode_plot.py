import control as ctrl
import matplotlib.pyplot as plt

KP_THETA_X = 48.0  # Proportional gain for roll stability

KI_THETA_X = 12.0 #5.5  # Integral gain for roll stability

KD_THETA_X = 0.0  # Derivative gain for roll stability

# PID gains (use roll gains here, or plot both roll and pitch if you like)
Kp = KP_THETA_X
Ki = KI_THETA_X
Kd = KD_THETA_X

# Define PID transfer function: Kd*s^2 + Kp*s + Ki
PID = ctrl.TransferFunction([Kd, Kp, Ki], [1, 0])

# Define a simple first-order plant: G(s) = 1 / (s + 1)
# You can replace this with your actual robot model if known
plant = ctrl.TransferFunction([1], [1, 1])

# Open-loop and closed-loop systems
open_loop = PID * plant
closed_loop = ctrl.feedback(open_loop, 1)

# Plot Bode plot
plt.figure()
ctrl.bode(open_loop, dB=True, margins=True)
plt.suptitle("Bode Plot of Open-Loop System", fontsize=14)
print("showing bode plot")

plt.savefig("bode_plot.png")

# Plot step response
t, y = ctrl.step_response(closed_loop)
plt.figure()
plt.plot(t, y)
plt.title("Step Response of Closed-Loop System")
plt.xlabel("Time [s]")
plt.ylabel("Output")
plt.grid(True)

print("\nshowing step response")

plt.figure()
plt.plot(t, y)
plt.title("Step Response")
plt.xlabel("Time [s]")
plt.ylabel("Output")
plt.grid(True)
plt.savefig("step_response.png")
