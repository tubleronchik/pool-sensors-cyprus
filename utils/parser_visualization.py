#!/usr/bin/env python3
import matplotlib.pyplot as plt


def old_firmware():
    ph_list = [0]
    orp_list = [0]
    time_ph = [0]
    time_orp = [0]
    ph_min = 5
    ph_mmax = 9
    with open("../data/pool_19_03") as f:
        for line in f:
            data = line.split(":")
            if "temperature" in data:
                ph = float(data[2].rstrip())
                ph_list.append(ph)
                current_time = time_ph[-1]
                time_ph.append(current_time + 1)
            else:
                orp = float(data[1].split("m")[0].rstrip())
                orp_list.append(orp)
                current_time = time_orp[-1]
                time_orp.append(current_time + 1)

    ph_graph = plt.figure()
    plt.plot(time_ph, ph_list)
    plt.axis([0, time_ph[-1], ph_min, ph_mmax])
    plt.title("Ph 19th marth")
    plt.xlabel("time, s")
    plt.ylabel("pH")
    ph_graph.savefig("../data/graphs/ph_19_03")

    orp_graph = plt.figure()
    plt.plot(time_orp, orp_list)
    plt.title("ORP 19th marth")
    plt.xlabel("time, s")
    plt.ylabel("ORP, mV")
    orp_graph.savefig("../data/graphs/orp_19_03")


def stagnant_water():
    ph_list = [0]
    orp_list = [0]
    time_ph = [0]
    time_orp = [0]
    ph_min = 5
    ph_mmax = 8
    with open("../data/stagnant_water_from_pool") as f:
        for line in f:
            data = line.split(":")
            if "temperature" in data[2]:
                ph = float(data[4].rstrip())
                ph_list.append(ph)
                current_time = time_ph[-1]
                time_ph.append(current_time + 1)
            else:
                orp = float(data[3].split("m")[0].rstrip())
                orp_list.append(orp)
                current_time = time_orp[-1]
                time_orp.append(current_time + 1)

    ph_graph = plt.figure()
    plt.plot(time_ph, ph_list)
    plt.axis([0, time_ph[-1], ph_min, ph_mmax])
    plt.title("Ph stagnant water")
    plt.xlabel("time, s")
    plt.ylabel("pH")
    ph_graph.savefig("../data/graphs/ph_stagnant_water")

    orp_graph = plt.figure()
    plt.plot(time_orp, orp_list)
    plt.title("ORP stagnant water")
    plt.xlabel("time, s")
    plt.ylabel("ORP, mV")
    orp_graph.savefig("../data/graphs/orp_stagnant_water")


stagnant_water()
old_firmware()
