length = 0
counter = 0
with open('pcb_heater_element.kicad_pcb') as f:
    for line in f.readlines():
        if "F.Cu" in line:
            line = line.strip()
            if "segment" in line:
                line = line.split(" ")
                length += abs(float(line[5]) - float(line[2]))
            if "gr_arc" in line:
                line = line.split(" ")
                length += 3.14159265359 * abs(float(line[3].strip(")")) - float(line[6].strip(")")))
            counter += 1
print("Tracks read: " + str(counter))
print("Track length is: " + str(length) + " mm")
print("Heater length is: " + str(length * 3) + " mm")