import matplotlib.pyplot as plt
import csv


def parse_csv_output(filename):
    tick_nums = []
    speeds = []
    torques = []
    with open(filename) as csvfile:
        for row in csv.reader(csvfile):
            tick_nums.append(int(row[0]))
            speeds.append(int(row[1]))
            torques.append(int(row[2]))
    processed_ticks = [tick - min(tick_nums) for tick in tick_nums]

    setpoint = 1500
    setpoints = [setpoint] * len(processed_ticks)

    speed_plt, = plt.plot(processed_ticks, speeds, 'm-', label="Speed")
    torque_plt, = plt.plot(processed_ticks, torques, 'c-', label="Torque CMD")
    setpoint_plt, = plt.plot(processed_ticks, setpoints, 'b-', label="Setpoint CMD")
    plt.legend(handles=[speed_plt, torque_plt, setpoint_plt])

    plt.xlabel("Ticks")
    plt.show()


if __name__ == "__main__":
    parse_csv_output("test10.csv")
