




for path in ("drift", "accel", "boost", "brake"):
	speedTables = []
	lastSpeed = -999999
	lastAngle = 0
	with open("turn." + path + ".txt", "r+") as f:
		for line in f:
			inSpeed, angle, totalTicks, driftTicks, outSpeed, outX, outY = map(float, line.split("\t"))
			if inSpeed != lastSpeed:
				speedTables.append((int(driftTicks), []))
				lastSpeed = inSpeed
				lastAngle = 0
			if angle > lastAngle:
				lastAngle = angle
				timeStr = f"{int(totalTicks)},\t" if "drift" in path else ""
				speedTables[-1][1].append(f"{{{angle}f,\t{timeStr}{outSpeed}f,\t{outX}f,\t{outY}f}}")


	with open("turnTable" + path.capitalize() + ".cpp", "w+") as f:
		f.write(
"""#include <drive/turn.h>

ManouvreTable<""" + ("DriftManouvre" if "drift" in path else "Manouvre") + """, 2300/50+1> """ + path.upper() + """TABLE = {
	0,
	50,
	{
		""" + ",\n\t\t".join(map(lambda t: "AngleTable<" + ("DriftManouvre" if "drift" in path else "Manouvre") + ">{" + str(t[0]) + ", 5, {\n\t\t\t" + ",\n\t\t\t".join(t[1]) + "\n\t\t}}", speedTables)) + """
	}
};
""")
