# Integration roadmap

- [ ] Record part numbers, wiring diagram, power budget and fuse ratings.
- [ ] Bring up Drive Uno with motor power disconnected.
- [ ] Calibrate pressure channels and encoder polarity.
- [ ] Bring up Terrain Uno sensors without the leg motor.
- [ ] Add leg driver, limit switches, current sensing and timeout.
- [ ] Test TOF against ramps, feet, holes and reflective surfaces.
- [x] Migrate BE-220 NMEA and crosswalk v6 logic into ROS nodes.
- [ ] Add and validate the crosswalk-to-V2X `itstId` mapping source.
- [ ] Collect and label camera data; define surface classes.
- [ ] Export a pinned YOLO model and benchmark worst-case latency.
- [x] Connect the prototype TorchScript classifier to ROS safety supervision.
- [ ] Add hardware-in-loop watchdog and fault-injection tests.
- [ ] Run lifted-wheel, then tethered unloaded tests.
- [ ] Obtain mechanical/electrical safety review before human trials.
