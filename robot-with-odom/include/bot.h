#include "pros/adi.hpp"
#include "pros/motors.hpp"

inline pros::Motor intake(3);
inline pros::Motor hang(1);

inline pros::ADIDigitalOut left_wing('b');
inline bool left_toggle = false;
inline pros::ADIDigitalOut right_wing('a');
inline bool right_toggle = false;
inline bool wings = false;