import matplotlib.pyplot as plt
import csv
import sys


def parse_csv_output(filename, setpoint=1500, keep_zeros=False):
    tick_nums = []
    speeds = []
    torques = []
    with open(filename) as csvfile:
        non_zero_found = keep_zeros
        for row in csv.reader(csvfile):
            try:  # I'm lazy, doing this to deal w/ weird serial lines...
                if(int(row[1]) != 0 or int(row[2]) != 0):
                    non_zero_found = True
                if not non_zero_found:
                    continue
                tick_nums.append(int(row[0]))
                speeds.append(int(row[1]))
                torques.append(int(row[2]))
            except:
                pass

    processed_ticks = [tick - min(tick_nums) for tick in tick_nums]
    setpoints = [setpoint] * len(processed_ticks)

    # Create plots
    speed_plt, = plt.plot(processed_ticks, speeds, 'm-', label="Speed")
    torque_plt, = plt.plot(processed_ticks, torques, 'c-', label="Torque CMD")
    setpoint_plt, = plt.plot(processed_ticks, setpoints, 'b-', label="Setpoint")
    plt.legend(handles=[speed_plt, torque_plt, setpoint_plt])

    # Image config
    plt.xlabel("Ticks")

    fig = plt.gcf()
    fig.set_size_inches(20, 15)

    plt.savefig('imgs/' + filename.split('/')[-1][:-4] + '.png')

    plt.show()


if __name__ == "__main__":
    filename = sys.argv[1]
    setpoint = sys.argv[2]
    if len(sys.argv) > 3:
        keep_zeros = sys.argv[3]
        parse_csv_output(filename, int(setpoint), int(keep_zeros))
    else:
        parse_csv_output(filename, int(setpoint))
