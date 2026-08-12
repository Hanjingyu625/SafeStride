remaining_time = float(input("Remaining signal time (sec): "))
crosswalk_width = float(input("Crosswalk width (m): "))
walking_speed = float(input("Walking speed (m/s): "))

safety_margin = 5.0

crossing_time = crosswalk_width / walking_speed
required_time = crossing_time + safety_margin

print()
print("Crossing time:", round(crossing_time, 1), "sec")
print("Required time:", round(required_time, 1), "sec")
print("Remaining time:", round(remaining_time, 1), "sec")

if remaining_time >= required_time:
    print("Result: CAN CROSS")
else:
    print("Result: WAIT")

