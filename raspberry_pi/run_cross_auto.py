import json
import subprocess

with open("nearest_standard_crosswalk.json") as file:
    crosswalk = json.load(file)

length = float(crosswalk["length_m"])
speed = input("Walking speed test value (m/s): ").strip()

print("Crosswalk length:", length, "m")

subprocess.run(
    ["python3", "cross_auto8.py"],
    input=f"{length}\n{speed}\n",
    text=True
)
